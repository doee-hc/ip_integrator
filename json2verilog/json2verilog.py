import json
import argparse

def get_port_type(port):
    if port.get('struct'):
        if port.get('package'):
            port_type = port['package'] + "::" + port['struct']
        else:
            port_type = port['struct']
    else:
        port_type = "logic"
    return port_type

def generate_verilog(json_file_name,verilog_file_name):

    with open(json_file_name, 'r') as json_file:
        json_input = json_file.read()

    # Parse JSON
    data = json.loads(json_input)

    # Start generating Verilog code
    verilog_code = "module top_module(\n"

    # Generate IO declaration
    for connection_name,connection in data['adHocConnections'].items():
        inst0,inst1 = connection_name.split("#")
        l_r = {'l_tie_io':inst0,'r_tie_io':inst1}
        io = {'set_i':"input",'set_o':"output",'set_io':"inout"}
        for l_r_key,l_r_value in l_r.items():
            if connection.get(l_r_key):
                for io_key,io_value in io.items():
                    if connection[l_r_key].get(io_key):
                        for port_name, width in connection[l_r_key][io_key].items():
                            port_type = get_port_type(data['instances'][l_r_value]['ports'][port_name])
                            width_l,width_r = width.strip('[]').split(':')
                            length_m1 = int(width_l) - int(width_r)
                            verilog_code += f"    {io_value:6s} {port_type} {io_value}_{l_r_value}_{port_name}[{length_m1}:0],\n"
    verilog_code = verilog_code.rstrip(",\n") + "\n);\n\n"


    # Generate parameter declarations for every instance
    for instance_name,instance in data["instances"].items():
        verilog_code += "    // Parameters for " + instance_name + "\n"
        for param,value in instance["parameters"].items():
            verilog_code += f"    parameter {instance_name}_{param} = {value};\n"
        verilog_code += "\n"

    # Generate net declaration
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

            # TODO: fix it in lib.json
            if port['width'] == '1':
                port['width'] = ''


            # port dimension
            if port.get('dimension'):
                dimension = ''
                for dim in port['dimension']:
                    dimension += dim
                verilog_code += f"    {port_type} {port['width']} {instance_name}_{port_name} {dimension};\n"
            else:
                verilog_code += f"    {port_type} {port['width']} {instance_name}_{port_name};\n"
        verilog_code += "\n"

    # Generate instance declaration 
    for instance_name,instance in data["instances"].items():
        verilog_code += f"    {instance['ipRef']} #(\n"
        params = data["instances"][instance_name]['parameters'].items()
        for param, value in params:
            verilog_code += f"        .{param}({instance_name}_{param}),\n"
        verilog_code = verilog_code.rstrip(",\n") + f"\n    ) {instance_name} (\n"
        for port_name,port in instance["ports"].items():
            verilog_code += f"      .{port_name}({instance_name}_{port_name}),\n"
        verilog_code = verilog_code.rstrip(",\n") + "\n    );\n\n"

    # Generate assignment
    for connection_name,connection in data['adHocConnections'].items():
        inst0,inst1 = connection_name.split("#")
        verilog_code += f"    // Connect {inst0} & {inst1}\n"
        print(f"Generate adHocConnection: {inst0} , {inst1}")
        for port_pair in connection['l2r']:
            port_pair_list = list(port_pair.items())
            initiator_port,initiator_width = port_pair_list[0]
            target_port,target_width = port_pair_list[1]
            verilog_code += f"    assign {inst1}_{target_port}{target_width} = {inst0}_{initiator_port}{initiator_width};\n"
        for port_pair in connection['r2l']:
            port_pair_list = list(port_pair.items())
            initiator_port,initiator_width = port_pair_list[0]
            target_port,target_width = port_pair_list[1]
            verilog_code += f"    assign {inst0}_{target_port}{target_width} = {inst1}_{initiator_port}{initiator_width};\n"

        l_r = {'l_tie_io':inst0,'r_tie_io':inst1}
        tie = {'tie0':'\'0','tie1':'\'1'}
        io = {'set_i':"input",'set_o':"output",'set_io':"inout"}
        for l_r_key,l_r_value in l_r.items():
            verilog_code += f"    // {l_r_value} Tie\n"
            if connection.get(l_r_key):
                for tie_key, tie_value in tie.items():
                    if connection[l_r_key].get(tie_key):
                        for port_name, width in connection[l_r_key][tie_key].items():
                            verilog_code += f"    assign {l_r_value}_{port_name}{width} = {tie_value};\n"
                verilog_code += f"    // {l_r_value} IO\n"
                for io_key,io_value in io.items():
                    if connection[l_r_key].get(io_key):
                        for port_name, width in connection[l_r_key][io_key].items():
                            if io_key == 'set_i':
                                verilog_code += f"    assign {l_r_value}_{port_name}{width} = {io_value}_{l_r_value}_{port_name};\n"
                            elif io_key == 'set_o':
                                verilog_code += f"    assign {io_value}_{l_r_value}_{port_name} = {l_r_value}_{port_name}{width};\n"
                            elif io_key == 'set_io':
                                # TODO: confirm io behavior
                                verilog_code += f"    assign {io_value}_{l_r_value}_{port_name} = {l_r_value}_{port_name}{width};\n"

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

