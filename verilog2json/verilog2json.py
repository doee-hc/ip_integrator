from antlr4 import *
from SystemVerilogLexer import SystemVerilogLexer
from SystemVerilogParser import SystemVerilogParser
from SystemVerilogParserListener import SystemVerilogParserListener
from SystemVerilogParserVisitor import SystemVerilogParserVisitor
from SystemVerilogPreParser import SystemVerilogPreParser
from SystemVerilogPreParserVisitor import SystemVerilogPreParserVisitor
from SystemVerilogPreParserListener import SystemVerilogPreParserListener
import re
import json
import os
import argparse
import ast
import operator
import copy

def clog2(x):
    if x <= 0:
        raise ValueError("clog2 is undefined for non-positive values")
    return x.bit_length() - (1 if x & (x - 1) == 0 else 0)

# 安全环境，只包含我们允许的运算符和函数
safe_dict = {
    'clog2': clog2,
    'concat': lambda *args: int(''.join(format(arg, 'b') for arg in args), 2)
}

# 添加安全的运算符
safe_operators = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.Mod: operator.mod,
    ast.Pow: operator.pow,
    ast.BitXor: operator.xor,
    ast.USub: operator.neg
}

def convert_ternary_operator(expr):
    # 正则表达式匹配形式为 x ? y : z 的三目运算符
    pattern = re.compile(r'(.+?)\s*\?\s*(.+?)\s*:\s*(.+)')
    # 替换为 Python 的条件表达式形式：y if x else z
    return pattern.sub(r'(\2 if \1 else \3)', expr)

def eval_expr(expr, params):
    """
    安全地计算表达式的值。
    Args:
        expr (str): 要计算的表达式。
        params (dict): 参数名称和值的映射。
    Returns:
        int: 表达式的计算结果。
    """
    # 解析表达式
    expr = expr.replace('$', '')
    expr = convert_ternary_operator(expr)
    tree = ast.parse(expr, mode='eval').body

    # 计算表达式的值
    def _eval(node):
        if isinstance(node, ast.Num):  # <number>
            return node.n
        elif isinstance(node, ast.BinOp):  # <left> <operator> <right>
            return safe_operators[type(node.op)](_eval(node.left), _eval(node.right))
        elif isinstance(node, ast.UnaryOp):  # <operator> <operand> e.g., -1
            return safe_operators[type(node.op)](_eval(node.operand))
        elif isinstance(node, ast.Name):  # <variable>
            if node.id in params:
                value = params[node.id]["value"]
                try:
                    # 尝试将参数值从字符串转换为整数
                    value = int(value)

                except ValueError:
                    # 如果转换失败，保持原样
                    pass
                
                if isinstance(value, str):
                    # 如果参数值是字符串，递归计算其值
                    return eval_expr(value.replace('$', ''), params)
                else:
                    return value
            else:
                raise ValueError(f"Unknown identifier: {node.id}")
        elif isinstance(node, ast.Call):  # <func>(<args>)
            if node.func.id in safe_dict:
                func = safe_dict[node.func.id]
                args = [_eval(arg) for arg in node.args]
                return func(*args)
            else:
                raise ValueError(f"Unknown function: {node.func.id}")
        elif isinstance(node, ast.IfExp):
            test = _eval(node.test)
            if test:
                return _eval(node.body)
            else:
                return _eval(node.orelse)
        elif isinstance(node, ast.Compare):
            # 比较操作通常只有一个操作符和两个操作数
            left = _eval(node.left)
            # 处理可能存在的多个比较
            for operation, right_operand in zip(node.ops, node.comparators):
                right = _eval(right_operand)
                if isinstance(operation, ast.Eq):
                    result = left == right
                elif isinstance(operation, ast.NotEq):
                    result = left != right
                elif isinstance(operation, ast.Lt):
                    result = left < right
                elif isinstance(operation, ast.LtE):
                    result = left <= right
                elif isinstance(operation, ast.Gt):
                    result = left > right
                elif isinstance(operation, ast.GtE):
                    result = left >= right
                # 根据比较结果更新左侧操作数，以支持链式比较
                left = right
            if not result:
                return False
            return True
        else:
            raise TypeError("Unsupported expression type: {}".format(type(node)))

    return _eval(tree)

def parse_packed(exp, params):
    parts = exp.strip("[]").split(':')
    # 将width解析成纯数字
    if len(parts) == 2:
        exp_l = eval_expr(parts[0], params)
        exp_r = eval_expr(parts[1], params)
        result = f"[{exp_l}:{exp_r}]"
    else:
        result = f"[{int(eval_expr(exp, params)-1)}:0]"
    return result

# package::struct的互联匹配规则
PACKAGE_STRUCT_DICT = {
    # opentitan ip
}
DEFAULT_STYLE = {
            "width": 80,
            "height": 60,
            "borderRadius": "10",
            "fontSize": 12,
            "fontColor": "#fff",
            "stroke": "#926390",
            "fill": "#926390"
        }
DEFAULT_CONTENT = {
            "category": "Peripherial",
            "ipFullName": {},
            "vendor": "Unknown",
            "version": "1.0",
            "description": "No description.",
            "parameters": {},
            "defines": {},
            "ports": {}
        }
# BUS_PACKAGE ={
#     "tilelink" : {
#         "tlul_pkg::tl_d2h_t"
#         "package": "tlul_pkg",
#         "struct": "tl_d2h_t"
#     }
# }



# 自定义监听器类
class ModuleListener(SystemVerilogParserListener):
    def __init__(self):
        self.modules = {}
        self.currentModuleName = ""
        self.currentParamName = ""
        self.currentPortDirection = ""
        self.currentPortType = ""
        self.currentPortWidth = ""
        self.iteration = 0
        self.paramRegion = False
        self.localParam ={}

    # def enterSource_text(self, ctx:SystemVerilogParser.Source_textContext):
    #     print(f"source_text: {ctx.getText()}")


    def enterModule_identifier(self, ctx:SystemVerilogParser.Module_identifierContext):
        self.currentModuleName = ctx.getText()
        self.modules[self.currentModuleName] = {
            'style': copy.deepcopy(DEFAULT_STYLE),
            'content': copy.deepcopy(DEFAULT_CONTENT)
        }
        self.modules[self.currentModuleName]['content']['ipFullName'] = self.currentModuleName
        print(f"Module name: {self.currentModuleName}")
        self.localParam = {}

    def enterLocal_parameter_declaration(self, ctx:SystemVerilogParser.Local_parameter_declarationContext):
        localParamName = ctx.list_of_param_assignments().param_assignment()[0].parameter_identifier().getText()
        localParamValue = ctx.list_of_param_assignments().param_assignment()[0].constant_param_expression().getText()
        self.localParam[localParamName] = {'value': localParamValue}

    def enterParameter_declaration(self, ctx:SystemVerilogParser.Parameter_declarationContext):
        self.paramRegion = True
        self.currentParamName = ctx.list_of_param_assignments().param_assignment()[0].parameter_identifier().getText()
        # 判断是否存在 type
        if ctx.data_type_or_implicit():
            if ctx.data_type_or_implicit().data_type().packed_dimension():
                # 判断是否存在 width
                if ctx.data_type_or_implicit().data_type().integer_vector_type():
                    param_type = ctx.data_type_or_implicit().data_type().integer_vector_type().getText()
                elif ctx.data_type_or_implicit().data_type().integer_atom_type():
                    param_type = ctx.data_type_or_implicit().data_type().integer_atom_type().getText()
                param_width = ctx.data_type_or_implicit().data_type().packed_dimension()[0].getText()
            else:
                param_type = ctx.data_type_or_implicit().getText()
                param_width = None
        else:
            param_type = None
            param_width = None
        
        # 判断是否存在 dimension
        if ctx.list_of_param_assignments().param_assignment()[0].unpacked_dimension():
            dimension = ctx.list_of_param_assignments().param_assignment()[0].unpacked_dimension()
        else:
            dimension = None

        if dimension:
            param_dimension = []
            for dim in dimension:
                param_dimension.append(dim.getText())
        else:
            param_dimension = None
        

        if dimension:
            param_dimension = []
            for dim in dimension:
                param_dimension.append(dim.getText())
        else:
            param_dimension = None

        param_value = ctx.list_of_param_assignments().param_assignment()[0].constant_param_expression().getText()
        self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName] = {'value': param_value}

        # # 将value转为javascript表达式，便于前端处理
        # js_expression = re.sub(r'\$clog2\((.*?)\)', 'Math.log2(\\1)', param_value)
        # self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['js_expression'] = js_expression

        if param_type:
            self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['type'] = param_type
        if param_width:
            self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['width'] = param_width

        # print(f"Parameter name: {self.currentParamName}, type: {param_type}, width: {param_width}, dimension: {param_dimension}, value: {param_value}, js_expression: {js_expression}")
        print(f"Parameter name: {self.currentParamName}, type: {param_type}, width: {param_width}, dimension: {param_dimension}, value: {param_value}")

    def exitParameter_declaration(self, ctx:SystemVerilogParser.Parameter_declarationContext):
        self.paramRegion = False

    # def enterConstant_expression(self, ctx:SystemVerilogParser.Constant_expressionContext):
    #     self.iteration += 1
    #     if self.paramRegion:
    #         # 判断是否存在 assignment pattern
    #         if ctx.constant_primary():
    #             if ctx.constant_primary().constant_assignment_pattern_expression():
    #                 # TODO
    #                 self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['js_expression'] = None
    #             elif ctx.constant_primary().constant_multiple_concatenation():
    #                 # TODO
    #                 self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['js_expression'] = None
        
        # print(f"Constant expression iteration {self.iteration}: {ctx.getText()}")
    def exitConstant_expression(self, ctx:SystemVerilogParser.Constant_expressionContext):
        self.iteration -= 1
        # print(f"Constant expression iteration {self.iteration}: {ctx.getText()}")
    # def enterSystem_tf_call(self, ctx:SystemVerilogParser.System_tf_callContext):
        
    
    def enterPort_decl(self, ctx:SystemVerilogParser.Port_declContext):
        
        port_package = None
        port_struct = None
        if ctx.ansi_port_declaration():
            port_names = ctx.ansi_port_declaration().port_identifier().getText()
            if ctx.ansi_port_declaration().port_direction():
                port_direction = ctx.ansi_port_declaration().port_direction().getText()
                if ctx.ansi_port_declaration().data_type():
                    # For opentitan ip
                    if ctx.ansi_port_declaration().data_type().class_type():
                        if(ctx.ansi_port_declaration().data_type().packed_dimension()):
                            port_package = ctx.ansi_port_declaration().data_type().class_type().getText()
                            port_struct = ctx.ansi_port_declaration().data_type().type_identifier().getText()
                        else:
                            class_ref = ctx.ansi_port_declaration().data_type().class_type().class_ref()
                            if len(class_ref) == 1:
                                port_struct = class_ref[0].getText()
                            elif len(class_ref) == 2:
                                port_package = class_ref[0].getText()
                                port_struct = class_ref[1].getText()

                    if(ctx.ansi_port_declaration().data_type().packed_dimension()):
                        port_width = ctx.ansi_port_declaration().data_type().packed_dimension()[0].getText()
                    else:
                        port_width = '[0:0]'
                elif ctx.ansi_port_declaration().implicit_data_type():
                    if(ctx.ansi_port_declaration().implicit_data_type().packed_dimension()):
                        port_width = ctx.ansi_port_declaration().implicit_data_type().packed_dimension()[0].getText()
                    else:
                        port_width = '[0:0]'
                elif ctx.ansi_port_declaration().net_type():
                    if ctx.ansi_port_declaration().data_type_or_implicit():
                        port_width = ctx.ansi_port_declaration().data_type_or_implicit().getText()
                    else:
                        port_width = '[0:0]'
                else:
                    port_width = '[0:0]'
            else:
                port_direction = self.currentPortDirection
                port_width = self.currentPortWidth

            if ctx.ansi_port_declaration().variable_dimension():
                dimension = ctx.ansi_port_declaration().variable_dimension()
            elif ctx.ansi_port_declaration().unpacked_dimension():
                dimension = ctx.ansi_port_declaration().unpacked_dimension()
            else:
                dimension = None

            if dimension:
                port_dimension = []
                for dim in dimension:
                    port_dimension.append(dim.getText())
            else:
                port_dimension = None

        else:
            print("ERROR: Not support non-ansi port declaration")
        
        print(f"Port name: {port_names}, direction: {port_direction}, width: {port_width}, dimension: {port_dimension}")

        # # 临时将所有port、dimension转为数字
        # params = {**self.modules[self.currentModuleName]['content']['parameters'],**self.localParam}
        # port_width = parse_packed(port_width,params)
        # parsed_dimension = []
        # if port_dimension:
        #     for dim in port_dimension:
        #         parsed_dimension.append(parse_packed(dim,params))
        #     port_dimension = parsed_dimension
        
        # 添加端口名称到模块信息中
        self.modules[self.currentModuleName]['content']['ports'][port_names] = {'direction': port_direction, 'width': port_width}

        # for opentitan ip
        if port_package:
            self.modules[self.currentModuleName]['content']['ports'][port_names]['package'] = port_package
            self.modules[self.currentModuleName]['content']['ports'][port_names]['struct'] = port_struct
            
        if port_dimension:
            self.modules[self.currentModuleName]['content']['ports'][port_names]['dimension'] = port_dimension
        self.currentPortDirection = port_direction
        self.currentPortWidth = port_width

# 自定义Listener类，支持include展开和ifdef处理
class IncludeListener(SystemVerilogPreParserListener):
    def __init__(self,origin_source_text,incdirs,included_files=[],defines={}):

        # 源码
        self.source_lines = origin_source_text.split('\n')

        # include文件的查找路径，用户通过命令行传入
        self.incdirs = incdirs

        # 记录已经include过的文件名，防止循环include
        self.included_files = included_files

        # 用户定义的define
        self.defines = defines

        # 表示ifdef/ifndef的代码区是否有效
        self.valid_region = [True]

        # 记录相对于原文添加的行数，当include展开时或被ifdef/ifndef屏蔽的区域被删除时需要记录行数变动
        self.added_line_num = 0

        # 记录嵌套的ifdef/elsif/else的tocken，当进入一个ifdef或ifndef时记录与之匹配elsif、else、endif的tocken index，这些index是唯一的，用于在对应的回调函数中设置self.valid_region
        self.region_tocken_idx = []

        # 当连续的 ifdef/elsif 在某处条件成立时，后续的elsif/else被屏蔽
        self.elsif_mask = False

    # define回调函数
    def enterText_macro_definition(self, ctx:SystemVerilogPreParser.Text_macro_definitionContext):
        # 判断是否在有效区 (若被ifdef或ifndef屏蔽，则无效)
        if not self.valid_region[-1]:
            return
        
        # 删除这句预处理命令
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        # 获取define的名称和内容
        define_name = ctx.macro_name().getText()
        define_texts = ctx.macro_text().macro_text_()
        define_text = ""
        
        if define_texts:
            for text in define_texts:
                define_text += text.getText()
            text_to_replace = preparse_systemverilog(define_text,self.incdirs,self.included_files,self.defines)[0]
            print(f"{define_text} translate to {text_to_replace}")
            self.defines[define_name] = text_to_replace
        else:
            self.defines[define_name] = ''



        print(f"self.defines: {self.defines}")

    # 当define过的宏被使用时的回调函数
    def enterText_macro_usage(self, ctx:SystemVerilogPreParser.Text_macro_usageContext):
        # 判断是否在有效区 (若被ifdef或ifndef屏蔽，则无效)
        if not self.valid_region[-1]:
            return
        
        line_start = ctx.start.line + self.added_line_num
        line_stop = ctx.stop.line + self.added_line_num
        col_start = ctx.start.column
        col_stop = ctx.stop.column

        # 获取宏的名称
        macro_directive = ctx.getText()
        macro = ctx.macro_usage().getText()

        # 替换宏
        if macro in self.defines.keys():
            macro_text = self.defines[macro]
            if macro_text:
                print(f"Replace {macro} with {macro_text}")

                self.source_lines[line_start-1] = self.source_lines[line_start-1].replace(macro_directive,macro_text)
            else:
                print(f"Error: define {macro} has no text.")
        else:
            print(f"Error: {macro} is not defined")

    # undef回调函数
    def enterUndef_directive(self, ctx:SystemVerilogPreParser.Undef_directiveContext):
        if not self.valid_region[-1]:
            return
        # 删除这句预处理命令
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1
        # 获取undef的名称
        undef = ctx.macro_identifier().getText()
        # 删除define
        if undef in self.defines.keys():
            del self.defines[undef]
        else:
            print(f"Warnning: {undef} is not defined")
        print(f"self.defines: {self.defines}")

    # ifdef回调函数
    def enterIfdef_directive(self, ctx:SystemVerilogPreParser.Ifdef_directiveContext):
        # 判断是否在有效区 (若被ifdef或ifndef屏蔽，则无效)
        if not self.valid_region[-1]:
            self.valid_region.append(False)
            return
        
        # 删除这句预处理命令
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        # 判断 ifdef 条件， 设置self.valid_region
        ifdef = ctx.macro_identifier().getText()
        if ifdef in self.defines.keys():
            self.valid_region.append(True)
            print(f"ifdef: {ifdef} is defined")
        else:
            self.valid_region.append(False)
            print(f"ifdef: {ifdef} not define")

    # ifndef回调函数,与ifdef一致
    def enterIfndef_directive(self, ctx:SystemVerilogPreParser.Ifndef_directiveContext):
        if not self.valid_region[-1]:
            self.valid_region.append(False)
            return
        
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        ifndef = ctx.macro_identifier().getText()
        if ifndef in self.defines.keys():
            print(f"ifndef: {ifndef} is defined")
            self.valid_region.append(False)
        else:
            print(f"ifndef: {ifndef} not define")
            self.valid_region.append(True)
            
    # 遇到ifdef elsif else预处理命令之间的代码 的回调函数
    def enterGroup_of_lines(self, ctx:SystemVerilogPreParser.Group_of_linesContext):
        # 当前区域无效时，删除这段代码
        if not self.valid_region[-1] and self.valid_region[-2] :
            text_to_discard = ctx.getText()
            print(f"This Group_of_lines is discarded:\n{text_to_discard}")
            line_to_discard = text_to_discard.split('\n')

            line_start = ctx.start.line + self.added_line_num

            line_stop = ctx.stop.line + self.added_line_num
            # This code sloves Antlr bug: get the line stop error.
            if line_start == line_stop:
                line_stop = line_start + len(line_to_discard) - 1
            else:
                line_stop += 1

            del self.source_lines[line_start-1:line_stop-1]
            self.added_line_num -= (line_stop - line_start)
            return
        elif self.valid_region[-1]:
            print(f"This Group_of_lines is reserved:\n{ctx.getText()}")

    # elsif回调函数
    def enterElsif_directive(self, ctx:SystemVerilogPreParser.Elsif_directiveContext):
        if self.valid_region[-2]:
            # 删除这句预处理命令
            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1

            # 之前的if无效时，判断这个elsif条件
            if self.valid_region[-1] == False :
                if self.elsif_mask == False:
                    ifdef = ctx.macro_identifier().getText()
                    if ifdef in self.defines.keys():
                        print(f"ifdef: {ifdef} is defined")
                        self.valid_region[-1] = True
                    else:
                        print(f"ifdef: {ifdef} not define")
                        self.valid_region[-1] = False
            else:
                # 如果之前的if有效，则本次以及后续elsif和else都无效
                self.valid_region[-1] = False
                self.elsif_mask = True  # 屏蔽后续的elsif,直到endif

    # else 回调函数
    def enterElse_directive(self, ctx:SystemVerilogPreParser.Else_directiveContext):
        if self.valid_region[-2]:
            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1
            if self.elsif_mask == False:
                self.valid_region[-1] = not self.valid_region[-1]

    # endif 回调函数
    def enterEndif_directive(self, ctx:SystemVerilogPreParser.Endif_directiveContext):
        if self.valid_region[-2]:
            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1
            self.elsif_mask = False
        self.valid_region.pop()

    # include 回调函数
    def enterInclude_directive(self, ctx:SystemVerilogPreParser.Include_directiveContext):
        if not self.valid_region[-1]:
            return
        filename = ctx.filename().getText()
        start_line = ctx.start.line + self.added_line_num
        stop_line = ctx.stop.line + self.added_line_num

        if filename in self.included_files:
            print(f"Error: Detected a loop include. Include file {filename} is already included.")
            return

        # Find the file in the include directories
        file_content = None
        for dir_path in self.incdirs:
            potential_path = os.path.join(dir_path, filename)
            if os.path.isfile(potential_path):
                with open(potential_path, 'r') as file:
                    file_content = file.read()
                break

        if file_content is None:
            print(f"Error: Include file {filename} not found in include directories.")
            return
        
        self.included_files.append(filename)

        print(f"Enter include file {filename}")
        lines_to_add = preparse_systemverilog(file_content,self.incdirs,self.included_files,self.defines)
        print(f"Exit include file {filename}")

        # Insert the content of the include file into the processed source text
        # Assuming that the line numbers start at 1 and that the source text is a list of lines

        # source_lines = delete_lines_and_shift_remaining(source_lines,(start_line,start_col),(stop_line,stop_col))
        del self.source_lines[start_line-1:stop_line]

        # Adjust for 0-based indexing used in Python lists
        insertion_point = stop_line-1
        # Insert the include file content into the original source text
        self.source_lines[insertion_point:insertion_point] = lines_to_add

        # Update the processed source text
        # self.processed_source_text = '\n'.join(source_lines)

        self.added_line_num += (len(lines_to_add) - 1)

        print(f"Add {self.added_line_num} line.")

def preparse_systemverilog(content,include_dirs,included_files=[],defines=[]):



    input_stream = InputStream(content)
    lexer = SystemVerilogLexer(input_stream)
    stream = CommonTokenStream(lexer, channel=SystemVerilogLexer.DIRECTIVES)
    parser = SystemVerilogPreParser(stream)
    tree = parser.source_text()

    listener = IncludeListener(content,include_dirs,included_files)
    walker = ParseTreeWalker()
    walker.walk(listener, tree)

    return listener.source_lines


# 解析SystemVerilog文件
def parse_systemverilog(text,group_name,json_file_name):

    input_stream = InputStream(text)
    lexer = SystemVerilogLexer(input_stream)
    stream = CommonTokenStream(lexer)
    parser = SystemVerilogParser(stream)

    tree = parser.source_text()
    # 打印解析树
    #print(f'Parse Tree: {tree.toStringTree(recog=parser)}')

    listener = ModuleListener()
    walker = ParseTreeWalker()
    walker.walk(listener, tree)

    for module_name, module in listener.modules.items():
        if module_name == "ahb3lite_interconnect":
            print(f"Add bus tag to {module_name}")
            module['content']['category'] = "Bus"
            for port_name, port in module['content']['ports'].items():
                if port_name.startswith("slv_H") or port_name.startswith("mst_H"):
                    Tag = "AHBL::" + port_name
                    print(f"Port:{port_name} Tag: {Tag}")
                    port['busRef'] = Tag

        if module_name == "CorePkg_top":
            print(f"Add bus tag to {module_name}")
            module['content']['category'] = "Master"
            for port_name, port in module['content']['ports'].items():
                if "ibus_h" in port_name:
                    Tag = "AHBL::mst_" + port_name.rsplit("_",1)[-1].upper() + "::0"
                    print(f"Port:{port_name} Tag: {Tag}")
                    port['busRef'] = Tag
                if "dbus_h" in port_name:
                    Tag = "AHBL::mst_" + port_name.rsplit("_",1)[-1].upper() + "::1"
                    print(f"Port:{port_name} Tag: {Tag}")
                    port['busRef'] = Tag


    ip_lib = {"result":[{"groupName":group_name,"groupData":listener.modules}]}

    

    # Convert the parsed modules to JSON format
    json_output = json.dumps(ip_lib, indent=4)

    with open(json_file_name, 'a') as json_file:
        json_file.write(json_output)



# 示例：解析一个文件并打印结果
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert Verilog files to JSON.')
    parser.add_argument('-f', '--file', help='Single Verilog file to process')
    parser.add_argument('-l', '--list', help='File containing a list of Verilog files to process')
    parser.add_argument('-o', '--output', required=True, help='Output JSON file name')
    parser.add_argument('-g', '--group', default='IP_lib', help='Group name for the IPs in the JSON file')
    parser.add_argument('-I', '--incdir', action='append', help='Include directory path(s)')
    args = parser.parse_args()

    files_to_process = []
    include_dirs = []

    if args.file:
        files_to_process.append(args.file)
    elif args.list:
        pre_path = os.path.dirname(args.list)
        if not pre_path:
            pre_path = '.'
        # Read filelist to get the list of Verilog files and include directories
        with open(args.list, 'r') as filelist:
            for line in filelist:
                stripped_line = line.strip()
                # Check for include directory command in filelist
                if stripped_line.startswith('+incdir+'):
                    dir_path = stripped_line[len('+incdir+'):]
                    include_dirs.append(pre_path + '/' + dir_path)
                elif (stripped_line.endswith('.v') or stripped_line.endswith('.sv')) and not stripped_line.startswith('#'):
                    files_to_process.append(pre_path +'/'+stripped_line)

    # Include directories specified by command line arguments
    if args.incdir:
        include_dirs.extend(args.incdir)

    if not files_to_process:
        parser.error('No input files provided. Use -f or -l option.')

    with open(args.output, 'w') as json_file:
        json_file.write('')


    # 初始化一个空的字符串，用来存储所有文件内容
    combined_content = ""

    # 遍历所有文件路径，将文件内容连接起来
    for file_path in files_to_process:
        with open(file_path, 'r') as file:
            print(f"Preparsing {file_path}")
            preprocessed_source_text = '\n'.join(preparse_systemverilog(file.read(),include_dirs))
            combined_content += preprocessed_source_text+'\n'

    with open("preproced_verilog.v", 'w') as file:
        file.write(combined_content)
    
    print("============Preprocessed source text=============")
    # print(combined_content)
    print("============Preprocessed source text=============")


    # 删掉module中的逻辑内容只保留port list，避免过多tocken导致antlr处理速度过慢
    pattern = r'(module\s+.*?\(.*?\)\s?;).*?(endmodule)'
    filted_code = re.sub(pattern, r'\1 \n \2', combined_content, flags=re.DOTALL)
    print(f'Filted code :{filted_code}')
    with open("filed_verilog.v", 'w') as file:
        file.write(filted_code)
    parse_systemverilog(filted_code,args.group,args.output)


  

