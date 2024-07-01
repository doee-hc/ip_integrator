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
                value = params[node.id]["value"]
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

def parse_width_dimension(exp, params):
    parts = exp.split(':')
    # 将width解析成纯数字
    if len(parts) == 2:
        # 多位宽的情况
        exp_l = eval_expr(parts[0], params)
        exp_r = eval_expr(parts[1], params)
        if exp_l == exp_r == 0:
            result = '1'
        else:
            result = f"[{exp_l}:{exp_r}]"
    else:
        result = f"[{int(eval_expr(exp, params) - 1)}:0]"
    return result

def process_ports(file_path, output_file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)

    for instance_name, instance in data['instances'].items():
        params = instance['parameters']
        ports = instance['ports']
        
        # 处理依赖其他参数的表达式
        for param_name,content in params.items():
            expr = content['value']
            params[param_name]['value'] = str(eval_expr(expr.replace('$', ''), params))
        
        # 处理端口定义
        for port_name, port in ports.items():
            port['width'] = parse_width_dimension(port['width'].strip("[]"), params)
            
            dimension = port['dimension'].strip("[]")
            if dimension :
                port['dimension'] = parse_width_dimension(dimension.strip("[]"), params)
            
            ports[port_name] = port
        
        data['instances'][instance_name]['ports'] = ports
        data['instances'][instance_name]['parameters'] = params
            
            

    # 将处理后的数据写入到输出文件
    with open(output_file_path, 'w') as outfile:
        json.dump(data, outfile, indent=4)

# 使用示例
process_ports('ports.json', 'ports_parsed.json')