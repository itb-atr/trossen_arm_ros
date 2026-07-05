import ast
import math
from collections.abc import Sequence

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from trossen_arm_msgs.msg import CartesianPoseCommand, CartesianWrenchCommand


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


def make_cartesian_pose_command(
    node: Node,
    values: Sequence[float],
    frame_id: str,
    goal_time: float,
    interpolation_space: str,
) -> CartesianPoseCommand:
    if len(values) != 6:
        raise ValueError('Pose command must contain exactly 6 values: [x, y, z, roll, pitch, yaw].')

    msg = CartesianPoseCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(values[0])
    msg.pose.position.y = float(values[1])
    msg.pose.position.z = float(values[2])
    msg.goal_time = float(goal_time)
    msg.interpolation_space = interpolation_space

    qx, qy, qz, qw = _quaternion_from_rpy(float(values[3]), float(values[4]), float(values[5]))
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def make_cartesian_wrench_command(
    node: Node,
    values: Sequence[float],
    frame_id: str,
    goal_time: float,
    interpolation_space: str,
) -> CartesianWrenchCommand:
    if len(values) != 6:
        raise ValueError('Wrench command must contain exactly 6 values: [fx, fy, fz, tx, ty, tz].')

    msg = CartesianWrenchCommand()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.wrench.force.x = float(values[0])
    msg.wrench.force.y = float(values[1])
    msg.wrench.force.z = float(values[2])
    msg.wrench.torque.x = float(values[3])
    msg.wrench.torque.y = float(values[4])
    msg.wrench.torque.z = float(values[5])
    msg.goal_time = float(goal_time)
    msg.interpolation_space = interpolation_space
    return msg


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = (sr * cp * cy) - (cr * sp * sy)
    qy = (cr * sp * cy) + (sr * cp * sy)
    qz = (cr * cp * sy) - (sr * sp * cy)
    qw = (cr * cp * cy) + (sr * sp * sy)
    return qx, qy, qz, qw


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
