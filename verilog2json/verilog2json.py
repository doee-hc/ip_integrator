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


# 自定义监听器类
class ModuleListener(SystemVerilogParserListener):
    def __init__(self):
        self.style = {
            "width": 80,
            "height": 60,
            "borderRadius": "10",
            "fontSize": 12,
            "fontColor": "#fff",
            "stroke": "#926390",
            "fill": "#926390"
        }
        self.content = {
            "category": "Peripherial",
            "ipFullName": {},
            "vendor": "Unknown",
            "version": "1.0",
            "description": "No description.",
            "parameters": {},
            "defines": {},
            "ports": {}
        }
        self.modules = {}
        self.currentModuleName = ""
        self.currentParamName = ""
        self.currentPortDirection = ""
        self.currentPortType = ""
        self.currentPortWidth = ""
        self.iteration = 0
        self.paramRegion = False

    # def enterSource_text(self, ctx:SystemVerilogParser.Source_textContext):
    #     print(f"source_text: {ctx.getText()}")


    def enterModule_identifier(self, ctx:SystemVerilogParser.Module_identifierContext):
        self.currentModuleName = ctx.getText()
        self.modules[self.currentModuleName] = {'style':self.style,'content':self.content}
        self.modules[self.currentModuleName]['content']['ipFullName'] = self.currentModuleName
        print(f"Module name: {self.currentModuleName}")

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

    def enterConstant_expression(self, ctx:SystemVerilogParser.Constant_expressionContext):
        self.iteration += 1
        if self.paramRegion:
            # 判断是否存在 assignment pattern
            if ctx.constant_primary():
                if ctx.constant_primary().constant_assignment_pattern_expression():
                    # TODO
                    self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['js_expression'] = None
                elif ctx.constant_primary().constant_multiple_concatenation():
                    # TODO
                    self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['js_expression'] = None
        
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
                        port_width = '1'
                elif ctx.ansi_port_declaration().implicit_data_type():
                    if(ctx.ansi_port_declaration().implicit_data_type().packed_dimension()):
                        port_width = ctx.ansi_port_declaration().implicit_data_type().packed_dimension()[0].getText()
                    else:
                        port_width = '1'
                elif ctx.ansi_port_declaration().net_type():
                    if ctx.ansi_port_declaration().data_type_or_implicit():
                        port_width = ctx.ansi_port_declaration().data_type_or_implicit().getText()
                    else:
                        port_width = '1'
                else:
                    port_width = '1'
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
        self.valid_region = True

        # 记录相对于原文添加的行数，当include展开时或被ifdef/ifndef屏蔽的区域被删除时需要记录行数变动
        self.added_line_num = 0

        # 记录嵌套的ifdef/elsif/else的tocken，当进入一个ifdef或ifndef时记录与之匹配elsif、else、endif的tocken index，这些index是唯一的，用于在对应的回调函数中设置self.valid_region
        self.region_tocken_idx = []

        # 当连续的 ifdef/elsif 在某处条件成立时，后续的elsif/else被屏蔽
        self.elsif_mask = False

    # define回调函数
    def enterText_macro_definition(self, ctx:SystemVerilogPreParser.Text_macro_definitionContext):
        # 判断是否在有效区 (若被ifdef或ifndef屏蔽，则无效)
        if not self.valid_region:
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
                # TODO 多行define的处理:下例
                # '                                                             `ifdef UVM                                                                                 uvm_pkg::uvm_report_error(, `PRIM_STRINGIFY(__name), uvm_pkg::UVM_NONE,                             `__FILE__, `__LINE__, , 1);                                `else                                                                                      $error(, $time, `__FILE__, `__LINE__,                  `PRIM_STRINGIFY(__name));                                                       `endif'
            text_to_replace = preparse_systemverilog(define_text,self.incdirs,self.included_files,self.defines)[0]
            print(f"{define_text} translate to {text_to_replace}")
            self.defines[define_name] = text_to_replace
        else:
            self.defines[define_name] = ''



        print(f"self.defines: {self.defines}")

    # 当define过的宏被使用时的回调函数
    def enterText_macro_usage(self, ctx:SystemVerilogPreParser.Text_macro_usageContext):
        # 判断是否在有效区 (若被ifdef或ifndef屏蔽，则无效)
        if not self.valid_region:
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
        if not self.valid_region:
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
        if not self.valid_region:
            return
        
        # 删除这句预处理命令
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        # 获取这句ifdef匹配的所有elsif、else、endif
        elsif_directive = ctx.elsif_directive()
        else_directive = ctx.else_directive()
        endif_directive = ctx.endif_directive()

        # 记录tocken index顺序
        if elsif_directive:
            for elsif in elsif_directive:
                self.region_tocken_idx.append(elsif.start.tokenIndex)
        if else_directive:
            self.region_tocken_idx.append(else_directive.start.tokenIndex)
        if endif_directive:
            self.region_tocken_idx.append(endif_directive.start.tokenIndex)
        else:
            print(f"Error: Not found endif directive. Line:{line_start}")

        # 判断 ifdef 条件， 设置self.valid_region
        ifdef = ctx.macro_identifier().getText()
        if ifdef in self.defines.keys():
            print(f"ifdef: {ifdef} is defined")
            self.valid_region = True
        else:
            print(f"ifdef: {ifdef} not define")
            self.valid_region = False

    # ifndef回调函数,与ifdef一致
    def enterIfndef_directive(self, ctx:SystemVerilogPreParser.Ifndef_directiveContext):
        if not self.valid_region:
            return
        
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        elsif_directive = ctx.elsif_directive()
        else_directive = ctx.else_directive()
        endif_directive = ctx.endif_directive()

        if elsif_directive:
            for elsif in elsif_directive:
                self.region_tocken_idx.append(elsif.start.tokenIndex)
        if else_directive:
            self.region_tocken_idx.append(else_directive.start.tokenIndex)
        if endif_directive:
            self.region_tocken_idx.append(endif_directive.start.tokenIndex)
        else:
            print(f"Error: Not found endif directive. Line:{line_start}")

        ifndef = ctx.macro_identifier().getText()
        if ifndef in self.defines.keys():
            print(f"ifndef: {ifndef} is defined")
            self.valid_region = False
        else:
            print(f"ifndef: {ifndef} not define")
            self.valid_region = True
            
    # 遇到ifdef elsif else预处理命令之间的代码 的回调函数
    def enterGroup_of_lines(self, ctx:SystemVerilogPreParser.Group_of_linesContext):
        # 当前区域无效时，删除这段代码
        if not self.valid_region:
            print(f"This Group_of_lines is discarded:\n{ctx.getText()}")
            line_start = ctx.start.line + self.added_line_num
            line_stop = line_start + len(ctx.getText().split('\n')) - 1
            # line_stop = ctx.stop.line + self.added_line_num
            del self.source_lines[line_start-1:line_stop]
            self.added_line_num -= (line_stop - line_start + 1)
            return
        print(f"Group_of_lines\n{ctx.getText()}")

    # elsif回调函数
    def enterElsif_directive(self, ctx:SystemVerilogPreParser.Elsif_directiveContext):
        # 获取当前的tocken index
        this_tocken_idx = ctx.start.tokenIndex
        # 判断与此前的indef的匹配关系
        if this_tocken_idx == self.region_tocken_idx[0]:
            # 如果匹配则pop
            self.region_tocken_idx.pop(0)

            # 删除这句预处理命令
            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1

            # 之前的if无效时，判断这个elsif条件
            if self.valid_region == False :
                if self.elsif_mask == False:
                    ifdef = ctx.macro_identifier().getText()
                    if ifdef in self.defines.keys():
                        print(f"ifdef: {ifdef} is defined")
                        self.valid_region = True
                    else:
                        print(f"ifdef: {ifdef} not define")
                        self.valid_region = False
            else:
                # 如果之前的if有效，则本次以及后续elsif和else都无效
                self.valid_region = False
                self.elsif_mask = True  # 屏蔽后续的elsif,直到endif

    # else 回调函数
    def enterElse_directive(self, ctx:SystemVerilogPreParser.Else_directiveContext):
        this_tocken_idx = ctx.start.tokenIndex
        if this_tocken_idx == self.region_tocken_idx[0]:
            self.region_tocken_idx.pop(0)
            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1
            if self.elsif_mask == False:
                self.valid_region = not self.valid_region

    # endif 回调函数
    def enterEndif_directive(self, ctx:SystemVerilogPreParser.Endif_directiveContext):
        this_tocken_idx = ctx.start.tokenIndex
        if this_tocken_idx == self.region_tocken_idx[0]:
            self.region_tocken_idx.pop(0)

            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1
            self.elsif_mask = False
            self.valid_region = True

    # include 回调函数
    def enterInclude_directive(self, ctx:SystemVerilogPreParser.Include_directiveContext):
        if not self.valid_region:
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

    print("Enter preparse_systemverilog")

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
            preprocessed_source_text = '\n'.join(preparse_systemverilog(file.read(),include_dirs))
            combined_content += preprocessed_source_text+'\n'

    with open("preproced_verilog.v", 'w') as json_file:
        json_file.write(combined_content)
    
    print("============Preprocessed source text=============")
    # print(combined_content)
    print("============Preprocessed source text=============")


    # 删掉module中的逻辑内容只保留port list，避免过多tocken导致antlr处理速度过慢
    pattern = r'(module\s+.*?\(.*?\)\s?;).*?(endmodule)'
    filted_code = re.sub(pattern, r'\1 \n \2', combined_content, flags=re.DOTALL)
    print(f'Filted code :{filted_code}')

    parse_systemverilog(filted_code,args.group,args.output)


  

