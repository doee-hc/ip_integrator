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

    def _eval(node):
        if isinstance(node, ast.Num):  # <number>
            return node.n
        elif isinstance(node, ast.BinOp):  # <left> <operator> <right>
            return safe_operators[type(node.op)](_eval(node.left), _eval(node.right))
        elif isinstance(node, ast.UnaryOp):  # <operator> <operand> e.g., -1
            return safe_operators[type(node.op)](_eval(node.operand))
        elif isinstance(node, ast.Name):  # <variable>
            if node.id in params:
                value = params[node.id]
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

# 示例使用
params = {'ADDR_WIDTH': '63','ID_WIDTH': 5}  # 假设我们有一个参数ADDR_WIDTH
expr = 'clog2(ADDR_WIDTH)-1'  # 假设我们要解析的表达式
value = eval_expr(expr, params)  # 计算表达式的值
print(value)