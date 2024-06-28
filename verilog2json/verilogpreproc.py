from antlr4 import *
from SystemVerilogLexer import SystemVerilogLexer
from SystemVerilogPreParser import SystemVerilogPreParser
from SystemVerilogPreParserListener import SystemVerilogPreParserListener
import os
import argparse

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

# 示例：解析一个文件并打印结果
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert Verilog files to JSON.')
    parser.add_argument('-f', '--file', help='Single Verilog file to process')
    parser.add_argument('-l', '--list', help='File containing a list of Verilog files to process')
    parser.add_argument('-o', '--output', help="Output Name(This arg is not used),default target dir :'preproced_code'")
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

    combined_content = ""

    target_dir = 'preproced_code'
    # 遍历所有文件路径
    for file_path in files_to_process:
        print("Processing file: " + file_path)
        with open(file_path, 'r') as file:
            preprocessed_source_text = '\n'.join(preparse_systemverilog(file.read(),include_dirs))

        processed_file_name = 'preproced_' + os.path.basename(file_path)

        target_path = os.path.join(target_dir, processed_file_name)

        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        with open(target_path, 'w') as output_file:
            output_file.write(preprocessed_source_text)

  

