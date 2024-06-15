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

def delete_lines_and_shift_remaining(source_lines, start_pos, end_pos):
    """
    删除行列表中从指定的开始位置到结束位置之间的文本，并将剩余的文本移动到新的一行。

    :param source_lines: 源文本行列表。
    :param start_pos: 开始位置，形式为 (行号, 列号)。
    :param end_pos: 结束位置，形式为 (行号, 列号)。
    :return: 处理后的行列表。
    """
    # 调整索引以适应Python的0索引
    start_line, start_col = start_pos[0]-1 , start_pos[1]
    end_line, end_col = end_pos[0]-1, end_pos[1]+1

    # 删除指定范围的文本
    if start_line == end_line:
        # 如果开始和结束在同一行，将剩余文本移动到新的一行
        remaining_text = source_lines[start_line][end_col:]
        source_lines[start_line] = source_lines[start_line][:start_col]
        source_lines.insert(start_line + 1, remaining_text)
        # if remaining_text:
        #     # 如果有剩余文本，将其添加为新的一行
        #     source_lines.insert(start_line + 1, remaining_text)
    else:
        # 如果开始和结束不在同一行，修改开始和结束行，并删除中间的行
        source_lines[start_line] = source_lines[start_line][:start_col]
        source_lines[end_line] = source_lines[end_line][end_col:]
        # 删除中间的行以及原结束行
        del source_lines[start_line + 1:end_line -1]

    return source_lines

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

        # 将value转为javascript表达式，便于前端处理
        js_expression = re.sub(r'\$clog2\((.*?)\)', 'Math.log2(\\1)', param_value)
        self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['js_expression'] = js_expression

        if param_type:
            self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['type'] = param_type
        if param_width:
            self.modules[self.currentModuleName]['content']['parameters'][self.currentParamName]['width'] = param_width

        print(f"Parameter name: {self.currentParamName}, type: {param_type}, width: {param_width}, dimension: {param_dimension}, value: {param_value}, js_expression: {js_expression}")

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
        
        if ctx.ansi_port_declaration():
            port_names = ctx.ansi_port_declaration().port_identifier().getText()
            if ctx.ansi_port_declaration().port_direction():
                port_direction = ctx.ansi_port_declaration().port_direction().getText()
                if ctx.ansi_port_declaration().data_type():
                    port_width = ctx.ansi_port_declaration().data_type().packed_dimension()[0].getText()
                elif ctx.ansi_port_declaration().implicit_data_type():
                    port_width = ctx.ansi_port_declaration().implicit_data_type().packed_dimension()[0].getText()
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
        if port_dimension:
            self.modules[self.currentModuleName]['content']['ports'][port_names]['dimension'] = port_dimension
        self.currentPortDirection = port_direction
        self.currentPortWidth = port_width


class IncludeListener(SystemVerilogPreParserListener):
    def __init__(self,origin_source_text,incdirs,included_files=[],defines={}):
        self.source_lines = origin_source_text.split('\n')
        self.incdirs = incdirs
        self.included_files = included_files
        self.defines = defines
        self.valid_region = True
        self.added_line_num = 0
        self.region_tocken_idx = []
        self.elsif_mask = False



    # def enterSource_text(self, ctx:SystemVerilogPreParser.Source_textContext):
    #     print(f"source_text: {ctx.getText()}")

    def enterText_macro_definition(self, ctx:SystemVerilogPreParser.Text_macro_definitionContext):
        if not self.valid_region:
            return
        
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1
        define_name = ctx.macro_name().getText()
        define_text = ctx.macro_text().macro_text_()

        if define_text:
            self.defines[define_name] = define_text[0].getText()
        else:
            self.defines[define_name] = ''
        
        print(f"self.defines: {self.defines}")

    def exitMacro_quote(self, ctx:SystemVerilogPreParser.Macro_quoteContext):
        pass
    def exitMacro_text(self, ctx:SystemVerilogPreParser.Macro_textContext):
        pass
    def enterMacro_text_(self, ctx:SystemVerilogPreParser.Macro_text_Context):
        pass
    def enterText_macro_usage(self, ctx:SystemVerilogPreParser.Text_macro_usageContext):
        if not self.valid_region:
            return
        line_start = ctx.start.line + self.added_line_num
        line_stop = ctx.stop.line + self.added_line_num
        col_start = ctx.start.column
        col_stop = ctx.stop.column
        macro_directive = ctx.getText()
        macro = ctx.macro_usage().getText()
        if macro in self.defines.keys():
            macro_text = self.defines[macro]
            if macro_text:
                self.source_lines[line_start-1] = self.source_lines[line_start-1].replace(macro_directive,macro_text)
            else:
                print(f"Warrning: define {macro} has no text.")
        else:
            print(f"Error: {macro} is not defined")

        pass
    def enterIfdef_directive(self, ctx:SystemVerilogPreParser.Ifdef_directiveContext):
        if not self.valid_region:
            return
        
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        else_directive = ctx.else_directive()
        elsif_directive = ctx.elsif_directive()
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

        ifdef = ctx.macro_identifier().getText()
        if ifdef in self.defines.keys():
            print(f"ifdef: {ifdef} is defined")
            self.valid_region = True
        else:
            print(f"ifdef: {ifdef} not define")
            self.valid_region = False

    def enterGroup_of_lines(self, ctx:SystemVerilogPreParser.Group_of_linesContext):
        if not self.valid_region:
            print(f"This Group_of_lines is discarded: {ctx.getText()}")
            line_start = ctx.start.line + self.added_line_num
            line_stop = ctx.stop.line + self.added_line_num
            del self.source_lines[line_start-1:line_stop]
            self.added_line_num -= (line_stop - line_start + 1)
            return
        print(f"Group_of_lines{ctx.getText()}")


    def enterIfndef_directive(self, ctx:SystemVerilogPreParser.Ifndef_directiveContext):
        if not self.valid_region:
            return
        
        line_start = ctx.start.line + self.added_line_num
        del self.source_lines[line_start-1]
        self.added_line_num -= 1

        else_directive = ctx.else_directive()
        elsif_directive = ctx.elsif_directive()
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

    def enterElsif_directive(self, ctx:SystemVerilogPreParser.Elsif_directiveContext):
        this_tocken_idx = ctx.start.tokenIndex
        if this_tocken_idx == self.region_tocken_idx[0]:
            self.region_tocken_idx.pop(0)

            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1

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
                self.valid_region = False
                self.elsif_mask = True  # 屏蔽后续的elsif,直到endif

    def enterElse_directive(self, ctx:SystemVerilogPreParser.Else_directiveContext):
        this_tocken_idx = ctx.start.tokenIndex
        if this_tocken_idx == self.region_tocken_idx[0]:
            self.region_tocken_idx.pop(0)
            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1
            if self.elsif_mask == False:
                self.valid_region = not self.valid_region


    def enterEndif_directive(self, ctx:SystemVerilogPreParser.Endif_directiveContext):
        this_tocken_idx = ctx.start.tokenIndex
        if this_tocken_idx == self.region_tocken_idx[0]:
            self.region_tocken_idx.pop(0)

            line_start = ctx.start.line + self.added_line_num
            del self.source_lines[line_start-1]
            self.added_line_num -= 1
            self.elsif_mask = False
            self.valid_region = True

    def enterInclude_directive(self, ctx:SystemVerilogPreParser.Include_directiveContext):
        if not self.valid_region:
            return
        filename = ctx.filename().getText()
        start_line = ctx.start.line + self.added_line_num
        stop_line = ctx.stop.line + self.added_line_num
        # start_col = ctx.start.column
        # stop_col = ctx.stop.column


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
        lines_to_add = preparse_systemverilog(file_content,self.incdirs,self.included_files,self.defines)

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


# 自定义Visitor类
# class IncludeFinder(SystemVerilogPreParserVisitor):
#     def __init__(self):
#         # 初始化一个列表来保存找到的include语句
#         self.includes = []

#     def visitInclude_directive(self, ctx:SystemVerilogPreParser.Include_directiveContext):
#         # 从上下文中提取include语句并添加到列表中
#         include_statement = ctx.getText()
#         self.includes.append(include_statement)
#         return self.visitChildren(ctx)

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
def parse_systemverilog(files,group_name,json_file_name):
    # 初始化一个空的字符串，用来存储所有文件内容
    combined_content = ""
    
    # 遍历所有文件路径，将文件内容连接起来
    for file_path in files:
        with open(file_path, 'r') as file:
            # 读取文件内容并连接
            combined_content += file.read() + "\n"  # 添加换行符确保文件间内容分隔

    input_stream = InputStream(combined_content)
    lexer = SystemVerilogLexer(input_stream)

    # 获取所有tokens，包括非默认通道上的
    # all_tokens = lexer.getAllTokens()
    # for token in all_tokens:
    #     print(token)
    # directive_tokens = [token for token in all_tokens if token.channel == lexer.DIRECTIVES]
    # tokens_on_default_channel = [token for token in all_tokens if token.channel == lexer.DEFAULT_TOKEN_CHANNEL]
    # stream = BufferedTokenStream(directive_tokens)

    stream = CommonTokenStream(lexer, channel=SystemVerilogLexer.DIRECTIVES)

    parser = SystemVerilogPreParser(stream)
    tree = parser.source_text()
    
    listener = IncludeListener()
    walker = ParseTreeWalker()
    walker.walk(listener, tree)




    # visitor = IncludeFinder()
    # # 遍历解析树
    # visitor.visit(tree)
    # 打印找到的include语句
    # print(visitor.includes)



    
    # lexer = SystemVerilogLexer(input_stream)
    # stream = CommonTokenStream(lexer)
    # parser = SystemVerilogParser(stream)
    # tree = parser.source_text()

    # listener = ModuleListener()
    # walker = ParseTreeWalker()
    # walker.walk(listener, tree)

    # ip_lib = {"result":[{"groupName":group_name,"groupData":listener.modules}]}

    # # Convert the parsed modules to JSON format
    # json_output = json.dumps(ip_lib, indent=4)

    # with open(json_file_name, 'a') as json_file:
    #     json_file.write(json_output)



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
        # Read filelist to get the list of Verilog files and include directories
        with open(args.list, 'r') as filelist:
            for line in filelist:
                stripped_line = line.strip()
                # Check for include directory command in filelist
                if stripped_line.startswith('+incdir+'):
                    dir_path = stripped_line[len('+incdir+'):]
                    include_dirs.append(dir_path)
                elif (stripped_line.endswith('.v') or stripped_line.endswith('.sv')) and not stripped_line.startswith('#'):
                    files_to_process.append(stripped_line)

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
            # 读取文件内容并连接
            combined_content += file.read() + "\n"  # 添加换行符确保文件间内容分隔

    preprocessed_source_text = '\n'.join(preparse_systemverilog(combined_content,include_dirs))
    print("============Preprocessed source text=============")
    print(preprocessed_source_text)
    print("============Preprocessed source text=============")


    #parse_systemverilog(files_to_process,args.group,args.output)

