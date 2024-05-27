import re
import json
import os
def parse_verilog(verilog_file,lib_modules):
    with open(verilog_file, 'r') as file:
        verilog_code = file.read()

    # Regular expressions to match module with parameters, ports, defines, etc.
    module_re = re.compile(
        r'module\s+(\w+)(?:\s*#\s*\((.*?)\))?\s*\((.*?)\);\s*(.*?)endmodule',
        re.DOTALL
    )
    param_re = re.compile(r'parameter\s+(\w+)\s*=\s*(.*?)(?:,|\s*$)', re.DOTALL)
    port_re = re.compile(r'(input|output|inout)\s+(wire\s+)?(\[.*?:.*?\])?\s*(\w+)', re.DOTALL)
    define_re = re.compile(r'`define\s+(\w+)')
    ifdefine_re = re.compile(r'`ifdef\s+(\w+)')


    for module_match in module_re.finditer(verilog_code):
        module_name, param_section, ports_section, internals = module_match.groups()
        # Extract parameters
        param_matches = param_re.findall(param_section) if param_section else {}
        port_matches = port_re.findall(ports_section)
        define_matches = define_re.findall(verilog_code)
        ifdefine_matches = ifdefine_re.findall(verilog_code)

        parameters = {name : {"value":value} for name, value in param_matches}
        ports = {name : {"width":width.strip('[]') if width else '1','direction': direction} for direction, _,width,name in port_matches}
        # ports = []
        # for port_match in port_matches:
        #     direction, _, width, name = port_match
        #     ports.append(
        #         {
        #             name : {
        #                 'width': width.strip('[]') if width else '1',
        #                 'direction': direction
        #             }
        #         }
        #         'name': name,
        #         'width': width.strip('[]') if width else '1',
        #         'direction': direction
        #     })

        defines = ' '.join("define " + define + ";" for define in define_matches)
        ifdefines = ' '.join("ifdef " + ifdef + ";" for ifdef in ifdefine_matches)

        module_info = {
            "ipName": module_name,
            "style": {
                "width": 80,
                "height": 60,
                "borderRadius": "10",
                "fontSize": 12,
                "fontColor": "#fff",
                "stroke": "#926390",
                "fill": "#926390"
            },
            "content": {
                "category": "Peripherial",
                "ipFullName": module_name,
                "vendor": "Unknown",
                "version": "1.0",
                "description": "No description.",
                "parameters": parameters,
                "defines": defines + ifdefines,
                "ports": ports,
                # "outputs": [{port['name']:port['width']} for port in ports if port['direction'] == 'output'],
                # "inouts": [{port['name']:port['width']} for port in ports if port['direction'] == 'inout']
            }
        }
        lib_modules.append(module_info)

def process_filelist(filelist_path,json_file_name):
    # Read filelist to get the list of Verilog files
    with open(filelist_path, 'r') as filelist:
        verilog_files = [line.strip() for line in filelist.readlines() if line.strip().endswith('.v')]

    lib_modules = []
    # Process each Verilog file and generate JSON
    for verilog_file_path in verilog_files:
        if not os.path.exists(verilog_file_path):
            print(f"File {verilog_file_path} not found.")
            continue
        

        # Parse the Verilog code
        parse_verilog(verilog_file_path,lib_modules)

        # Generate JSON file name
        # json_file_name = os.path.splitext(os.path.basename(verilog_file_path))[0] + '.json'

        print(f"Generated JSON for {verilog_file_path}")

    ip_lib = {"result":[{"groupName":group_name,"groupData":lib_modules}]}
    # Convert the parsed modules to JSON format
    json_output = json.dumps(ip_lib, indent=4)
    # Write the JSON output to a file
    with open(json_file_name, 'a') as json_file:
        json_file.write(json_output)

json_file_name = 'lib_map_form.json'
group_name = 'IP_lib0'
with open(json_file_name, 'w') as json_file:
    json_file.write('')

process_filelist('rtl.lst',json_file_name)


