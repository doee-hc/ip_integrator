import json
import ast
import operator
import argparse

def clog2(x):
    if x <= 0:
        raise ValueError("clog2 is undefined for non-positive values")
    return x.bit_length() - (1 if x & (x - 1) == 0 else 0)

# 安全环境，只包含我们允许的运算符和函数
safe_dict = {
    'clog2': clog2,
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
                value = params[node.id]['value']
                try:
                    # 尝试将参数值从字符串转换为整数
                    value = int(value)
                except ValueError:
                    # 如果转换失败，保持原样
                    pass
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
        else:
            raise TypeError(node)

    return _eval(tree)

def generate_wire_declaration(connection, data):
    instance_ref = connection['instanceRef']
    params = data['instances'][instance_ref]['parameters']
    port = connection['portRef']
    width = connection['width']
    package = connection['package']
    struct = connection['struct']

    if struct:
        if package:
            port_type = package + "::" + struct
        else:
            port_type = struct
    else:
        port_type = "logic"

    parts = width.split(':')
    if len(parts) == 1:
        # 1bit
        return f"      {port_type} {instance_ref}_{port};\n"
    elif len(parts) == 2:
        # 多位宽的情况
        width_l = eval_expr(parts[0], params)
        width_r = eval_expr(parts[1], params)
        return f"      {port_type} [{width_l}:{width_r}] {instance_ref}_{port};\n"
    else:
        # 如果分割后的部分不是1也不是2，抛出异常
        raise ValueError(f"Invalid width format: {width}")

def generate_verilog(json_file_name,verilog_file_name):

    with open(json_file_name, 'r') as json_file:
        json_input = json_file.read()

    # Parse JSON
    data = json.loads(json_input)

    # Start generating Verilog code
    verilog_code = "module top_module();\n\n"


    instances_ports = {}

    # Generate parameter and port declarations
    for instance_name, instance in data['instances'].items():
        instances_ports.update({instance_name:[]})
        # Parameters
        for param, value in instance['parameters'].items():
            verilog_code += f"      localparam {instance_name}_{param} = {value};\n"
        verilog_code += "\n"


    for connection in data['adHocConnections'].values():
        target_inst = connection['target']['instanceRef']
        target_port = connection['target']['port']
        initiator_inst = connection['initiator']['instanceRef']
        initiator_port = connection['initiator']['port']
        instances_ports[target_inst].append(target_port)
        instances_ports[initiator_inst].append(initiator_port)
        verilog_code += generate_wire_declaration(connection['initiator'], data)
        verilog_code += generate_wire_declaration(connection['target'], data)
        verilog_code += "\n"
        verilog_code += f"      assign {target_inst}_{target_port} = {initiator_inst}_{initiator_port};\n"
        verilog_code += "\n"
        

    # Generate instance declarations
    for instance_name, instance in data['instances'].items():
        verilog_code += f"    // Instance of {instance_name}\n"
        verilog_code += f"    {instance['ipRef']} #(\n"
        params = instance['parameters']
        for i, (param, value) in enumerate(params.items()):
            comma = "," if i < len(params) - 1 else ""
            verilog_code += f"        .{param}({instance_name}_{param}){comma}\n"
        verilog_code += f"    ) {instance_name} (\n"
        for port in instances_ports[instance_name]:
            verilog_code += f"      .{port}({instance_name}_{port}),\n"
        verilog_code = verilog_code.rstrip(",\n") + "\n    );\n\n"

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

