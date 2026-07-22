"""Safe mathematical expression evaluator for ProtoGen XML attributes.

Supports: +, -, *, /, ^, **, parentheses, pi, e, integers, floats
Does NOT support: function calls, variable references, arbitrary code

This module provides a secure alternative to eval() for evaluating
mathematical expressions found in ProtoGen XML attributes like:
  scaler="180*10000000/pi"
  max="2^15"
  min="-3*pi/2"
"""

import ast
import math
import operator
from typing import Union, Optional

# Operators we allow
_BINARY_OPS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.Div: operator.truediv,
    ast.Pow: operator.pow,
    ast.FloorDiv: operator.floordiv,
    ast.Mod: operator.mod,
}

_UNARY_OPS = {
    ast.USub: operator.neg,
    ast.UAdd: operator.pos,
}

# Named constants
_CONSTANTS = {
    'pi': math.pi,
    'e': math.e,
}


def safe_eval(expr: str) -> float:
    """Safely evaluate a mathematical expression.

    Args:
        expr: Expression string like "180*10000000/pi" or "2^15"

    Returns:
        Evaluated float value

    Raises:
        ValueError: If expression contains unsupported operations or syntax

    Examples:
        >>> safe_eval("180*10000000/pi")
        572957795.1308232
        >>> safe_eval("2^15")
        32768.0
        >>> safe_eval("pi/2")
        1.5707963267948966
        >>> safe_eval("-3*pi/2")
        -4.71238898038469
    """
    if not expr or not expr.strip():
        return 0.0

    # Normalize the expression
    expr = expr.strip()

    # Convert ^ to ** for Python exponentiation
    expr = expr.replace('^', '**')

    # Parse into AST
    try:
        tree = ast.parse(expr, mode='eval')
    except SyntaxError as e:
        raise ValueError(f"Invalid expression '{expr}': {e}")

    return float(_eval_node(tree.body))


def _eval_node(node: ast.AST) -> Union[int, float]:
    """Recursively evaluate an AST node."""

    # Numeric literal (Python 3.8+)
    if isinstance(node, ast.Constant):
        if isinstance(node.value, (int, float)):
            return node.value
        raise ValueError(f"Unsupported constant type: {type(node.value).__name__}")

    # Numeric literal (Python 3.7 compatibility)
    if hasattr(ast, 'Num') and isinstance(node, ast.Num):
        return node.n

    # Named constant (pi, e)
    if isinstance(node, ast.Name):
        if node.id in _CONSTANTS:
            return _CONSTANTS[node.id]
        raise ValueError(f"Unknown constant: '{node.id}' (allowed: {', '.join(_CONSTANTS.keys())})")

    # Binary operation (+, -, *, /, **, etc.)
    if isinstance(node, ast.BinOp):
        op_type = type(node.op)
        if op_type not in _BINARY_OPS:
            raise ValueError(f"Unsupported operator: {op_type.__name__}")
        left = _eval_node(node.left)
        right = _eval_node(node.right)
        return _BINARY_OPS[op_type](left, right)

    # Unary operation (-, +)
    if isinstance(node, ast.UnaryOp):
        op_type = type(node.op)
        if op_type not in _UNARY_OPS:
            raise ValueError(f"Unsupported unary operator: {op_type.__name__}")
        operand = _eval_node(node.operand)
        return _UNARY_OPS[op_type](operand)

    raise ValueError(f"Unsupported expression element: {type(node).__name__}")


def safe_eval_or_none(expr: Optional[str]) -> Optional[float]:
    """Evaluate expression, returning None if empty/None.

    Args:
        expr: Expression string or None

    Returns:
        Evaluated float value, or None if expr is None/empty
    """
    if expr is None or not expr.strip():
        return None
    return safe_eval(expr)


def safe_eval_or_default(expr: Optional[str], default: float = 0.0) -> float:
    """Evaluate expression, returning default if empty/None.

    Args:
        expr: Expression string or None
        default: Value to return if expr is None/empty

    Returns:
        Evaluated float value, or default if expr is None/empty
    """
    if expr is None or not expr.strip():
        return default
    return safe_eval(expr)
