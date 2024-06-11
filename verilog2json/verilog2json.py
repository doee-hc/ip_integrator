from antlr4 import *
from SystemVerilogLexer import SystemVerilogLexer
from SystemVerilogParser import SystemVerilogParser
from SystemVerilogParserListener import SystemVerilogParserListener
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
        self.currentPortDirection = ""
        self.currentPortType = ""
        self.currentPortWidth = ""

    def enterModule_identifier(self, ctx:SystemVerilogParser.Module_identifierContext):
        self.currentModuleName = ctx.getText()
        self.modules[self.currentModuleName] = {'style':self.style,'content':self.content}
        self.modules[self.currentModuleName]['content']['ipFullName'] = self.currentModuleName
        print(f"Module name: {self.currentModuleName}")

    def enterParameter_declaration(self, ctx:SystemVerilogParser.Parameter_declarationContext):
        param_name = ctx.list_of_param_assignments().param_assignment()[0].parameter_identifier().getText()

        if ctx.data_type_or_implicit():
            param_type = ctx.data_type_or_implicit().getText()
        else:
            param_type = None

        param_value = ctx.list_of_param_assignments().param_assignment()[0].constant_param_expression().getText()
        self.modules[self.currentModuleName]['content']['parameters'][param_name] = {'type': param_type, 'value': param_value}
        print(f"Parameter name: {param_name}, type: {param_type}, value: {param_value}")

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
            port_dimension = []
            if dimension:
                for dim in dimension:
                    port_dimension.append(dim.getText())
        else:
            print("ERROR: Not support non-ansi port declaration")
        
        print(f"Port name: {port_names}, direction: {port_direction}, width: {port_width}, dimention: {port_dimension}")

        # 添加端口名称到模块信息中
        self.modules[self.currentModuleName]['content']['ports'][port_names] = {'direction': port_direction, 'width': port_width, 'dimention': port_dimension}
        self.currentPortDirection = port_direction
        self.currentPortWidth = port_width

# def pack_to_json(modules):
#     for module in modules:
#         print(module)


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
    stream = CommonTokenStream(lexer)
    parser = SystemVerilogParser(stream)
    tree = parser.source_text()

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
    args = parser.parse_args()

    files_to_process = []
    if args.file:
        files_to_process.append(args.file)
    elif args.list:
        # Read filelist to get the list of Verilog files
        with open(args.list, 'r') as filelist:
            files_to_process.extend([line.strip() for line in filelist.readlines() if line.strip().endswith('.v') and not line.strip().startswith('#')])

    if not files_to_process:
        parser.error('No input files provided. Use -f or -l option.')

    with open(args.output, 'w') as json_file:
        json_file.write('')
    parse_systemverilog(files_to_process,args.group,args.output)

