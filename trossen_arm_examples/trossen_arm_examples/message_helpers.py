import ast
from collections.abc import Sequence

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def declare_dynamic_parameter(node: Node, name: str, default_value):
    descriptor = ParameterDescriptor(dynamic_typing=True)
    node.declare_parameter(name, default_value, descriptor)


def get_float_array_parameter(node: Node, name: str, length: int) -> list[float]:
    value = node.get_parameter(name).value
    values = _coerce_sequence(value, name)
    if len(values) != length:
        raise ValueError(f'Parameter `{name}` must contain exactly {length} values.')

    try:
        return [float(item) for item in values]
    except (TypeError, ValueError) as exc:
        raise ValueError(f'Parameter `{name}` must contain only numeric values.') from exc


def get_string_parameter(node: Node, name: str) -> str:
    return str(node.get_parameter(name).value)


def get_bool_parameter(node: Node, name: str) -> bool:
    value = node.get_parameter(name).value
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def make_float64_array(values: Sequence[float]) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return msg


def _coerce_sequence(value, name: str) -> Sequence:
    if isinstance(value, str):
        try:
            parsed = ast.literal_eval(value)
        except (SyntaxError, ValueError) as exc:
            raise ValueError(
                f'Parameter `{name}` must be a list such as "[0.35, 0.0, 0.22, 0, 0, 0]".'
            ) from exc
        return _coerce_sequence(parsed, name)

    if isinstance(value, Sequence):
        return value

    raise ValueError(f'Parameter `{name}` must be a numeric list.')
