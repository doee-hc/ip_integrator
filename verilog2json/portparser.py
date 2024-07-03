import re
import json
import os
import argparse
import ast
import operator

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

def parse_width_dimension(exp, params):
    parts = exp.split(':')
    # 将width解析成纯数字
    if len(parts) == 2:
        exp_l = eval_expr(parts[0], params)
        exp_r = eval_expr(parts[1], params)
        result = f"[{exp_l}:{exp_r}]"
    else:
        result = f"[{int(eval_expr(exp, params)-1)}:0]"
    return result

def process_instance_parameters(params):
    """
    Process and evaluate expressions for instance parameters.
    Args:
        params (dict): A dictionary of parameters with expressions as values.
    Returns:
        dict: A dictionary of parameters with evaluated values.
    """
    for param_name, content in params.items():
        expr = content['value']
        # Evaluate the expression and update the parameter's value
        params[param_name]['value'] = str(eval_expr(expr.replace('$', ''), params))
    return params

def process_instance_ports(ports, params):
    """
    Process and update port dimensions based on evaluated parameters.
    Args:
        ports (dict): A dictionary of ports with width and dimension expressions.
        params (dict): A dictionary of evaluated parameters.
    Returns:
        dict: A dictionary of ports with updated width and dimension values.
    """
    for port_name, port in ports.items():
        # Update the width of the port
        port['width'] = parse_width_dimension(port['width'].strip("[]"), params)
        
        # Update the dimension of the port, if it exists
        dimension = port['dimension'].strip("[]")
        if dimension:
            port['dimension'] = parse_width_dimension(dimension, params)
        
        ports[port_name] = port
    return ports

def process_ports(file_path, output_file_path):
    """
    Process ports and parameters in a JSON file and output the results to another file.
    Args:
        file_path (str): Path to the input JSON file.
        output_file_path (str): Path to the output JSON file.
    """
    with open(file_path, 'r') as file:
        data = json.load(file)

    for instance_name, instance in data['instances'].items():
        # Process and evaluate parameters
        # params = process_instance_parameters(instance['parameters'])
        params = instance['parameters']
        # Process ports with updated parameters
        ports = process_instance_ports(instance['ports'], params)
        
        # Update the instance with processed parameters and ports
        # data['instances'][instance_name]['parameters'] = params
        data['instances'][instance_name]['ports'] = ports

    # Write the processed data to the output file
    with open(output_file_path, 'w') as outfile:
        json.dump(data, outfile, indent=4)

# 使用示例
process_ports('ports.json', 'ports_parsed.json')