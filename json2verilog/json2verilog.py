import json
import argparse

def generate_verilog(json_file_name,verilog_file_name):

    with open(json_file_name, 'r') as json_file:
        json_input = json_file.read()

    # Parse JSON
    data = json.loads(json_input)

    # Start generating Verilog code
    verilog_code = "module top_module();\n\n"

    for instance_name,instance in data["instances"].items():
        # Generate parameter declarations for every instance
        verilog_code += "    // Parameters for " + instance_name + "\n"
        for param,value in instance["parameters"].items():
            verilog_code += f"    parameter {instance_name}_{param} = {value};\n"
        verilog_code += "\n"

    # Generate port declaration
    for instance_name,instance in data["instances"].items():
        verilog_code += f"    // Ports for {instance_name}\n"
        for port_name,port in instance["ports"].items():
            # port type
            if port.get('struct'):
                if port.get('package'):
                    port_type = port['package'] + "::" + port['struct']
                else:
                    port_type = port['struct']
            else:
                port_type = "logic"
            # port dimension
            if port.get('dimension'):
                verilog_code += f"    {port_type} {port['width']} {instance_name}_{port_name} {port['dimension']};\n"
            else:
                verilog_code += f"    {port_type} {port['width']} {instance_name}_{port_name};\n"
        verilog_code += "\n"

    # Generate instance declaration 
    for instance_name,instance in data["instances"].items():
        verilog_code += f"    {instance['ipRef']} #(\n"
        params = data["instances"].items()
        for i, (param, value) in enumerate(params):
            comma = "," if i < len(params) - 1 else ""
            verilog_code += f"        .{param}({instance_name}_{param}){comma}\n"
        verilog_code += f"    ) {instance_name} (\n"
        for port_name,port in instance["ports"].items():
            verilog_code += f"      .{port_name}({instance_name}_{port_name}),\n"
        verilog_code = verilog_code.rstrip(",\n") + "\n    );\n\n"

    # Generate assignment
    for connection_name,connection in data['adHocConnections'].items():
        inst0,inst1 = connection_name.split("#")
        verilog_code += f"    // Connection {inst0} & inst1\n"
        print(f"Generate adHocConnection: {inst0} , {inst1}")
        for port_pair in connection['l2r']:
            port_pair_list = list(port_pair.items())
            initiator_port,initiator_width = port_pair_list[0]
            target_port,target_width = port_pair_list[1]
            verilog_code += f"    assign {inst1}_{target_port}{target_width} = {inst0}_{initiator_port}{initiator_width};\n"


    verilog_code += "\nendmodule"

    with open(verilog_file_name, 'w') as verilog_file:
        verilog_file.write(verilog_code)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert JSON files to Verilog.')
    parser.add_argument('-f', '--file', help='Single JSON file to process')
    parser.add_argument('-o', '--output', required=True, help='Output Verilog file name')
    args = parser.parse_args()


    if not args.file:
        parser.error('No input files provided. Use -f option.')

    generate_verilog(args.file, args.output)
    # Output Verilog code
    print(f"{args.file} translate to {args.output}")

