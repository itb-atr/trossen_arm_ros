import time

import rclpy
from rclpy.node import Node
from trossen_arm_msgs.msg import CartesianPoseCommand

from trossen_arm_examples.controller_manager_helpers import ControllerManagerClient
from trossen_arm_examples.message_helpers import (
    declare_dynamic_parameter,
    get_bool_parameter,
    get_float_array_parameter,
    get_string_parameter,
    make_cartesian_pose_command,
)


class CartesianPositionDemo(Node):
    def __init__(self) -> None:
        super().__init__('cartesian_position_demo')

        declare_dynamic_parameter(self, 'command_topic', '/cartesian_position_controller/command')
        declare_dynamic_parameter(self, 'command_frame_id', 'base_link')
        declare_dynamic_parameter(self, 'command_goal_time', 2.0)
        declare_dynamic_parameter(self, 'command_interpolation_space', 'cartesian')
        declare_dynamic_parameter(self, 'auto_activate_controller', False)
        declare_dynamic_parameter(self, 'controller_manager_name', '/controller_manager')
        declare_dynamic_parameter(self, 'controller_name', 'cartesian_position_controller')
        declare_dynamic_parameter(self, 'deactivate_controller_name', 'cartesian_external_effort_controller')
        declare_dynamic_parameter(self, 'start_pose', '[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'base_frame_step', '[0.02, 0.0, 0.0, 0.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'step_count', 5)
        declare_dynamic_parameter(self, 'return_pose', '[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'publish_delay_sec', 0.5)
        declare_dynamic_parameter(self, 'start_settle_sec', 2.0)
        declare_dynamic_parameter(self, 'step_settle_sec', 1.0)
        declare_dynamic_parameter(self, 'return_wait_sec', 5.0)
        declare_dynamic_parameter(self, 'return_settle_sec', 2.0)

        self._command_topic = get_string_parameter(self, 'command_topic')
        self._command_frame_id = get_string_parameter(self, 'command_frame_id')
        self._command_goal_time = float(self.get_parameter('command_goal_time').value)
        self._command_interpolation_space = get_string_parameter(self, 'command_interpolation_space')
        self._start_pose = get_float_array_parameter(self, 'start_pose', 6)
        self._base_frame_step = get_float_array_parameter(self, 'base_frame_step', 6)
        self._step_count = int(self.get_parameter('step_count').value)
        if self._step_count < 0:
            raise ValueError('Parameter `step_count` must be zero or greater.')
        self._return_pose = get_float_array_parameter(self, 'return_pose', 6)
        self._publish_delay_sec = float(self.get_parameter('publish_delay_sec').value)
        self._start_settle_sec = float(self.get_parameter('start_settle_sec').value)
        self._step_settle_sec = float(self.get_parameter('step_settle_sec').value)
        self._return_wait_sec = float(self.get_parameter('return_wait_sec').value)
        self._return_settle_sec = float(self.get_parameter('return_settle_sec').value)

        self._publisher = self.create_publisher(CartesianPoseCommand, self._command_topic, 10)

    def run(self) -> None:
        if get_bool_parameter(self, 'auto_activate_controller'):
            manager = ControllerManagerClient(
                self, get_string_parameter(self, 'controller_manager_name')
            )
            manager.ensure_active(
                [get_string_parameter(self, 'controller_name')],
                deactivate_controller_names=[
                    get_string_parameter(self, 'deactivate_controller_name')
                ],
            )

        self._wait_for_command_subscription()
        self._sleep_with_spin(self._publish_delay_sec)

        self.get_logger().info(
            'Driving the tool point to the start pose: '
            f'{self._format_values(self._start_pose)}'
        )
        self._publish_command(self._start_pose)
        self._sleep_with_spin(self._start_settle_sec)
        self.get_logger().info('Reached start position')

        self.get_logger().info(
            'Now demonstrating base-frame Cartesian position commands: each command '
            'adds 2 cm along base-frame +X to the start pose.'
        )

        for step_index in range(1, self._step_count + 1):
            target_pose = [
                self._start_pose[index] + (self._base_frame_step[index] * step_index)
                for index in range(6)
            ]
            self.get_logger().info(
                f'Step {step_index}/{self._step_count}: publishing absolute '
                f'base-frame target {self._format_values(target_pose)}'
            )
            self._publish_command(target_pose)
            self._sleep_with_spin(self._step_settle_sec)

            self.get_logger().info(
                f'Completed base-frame +X step {step_index}/{self._step_count}'
            )

        self.get_logger().info(
            f'Waiting for {self._return_wait_sec:.1f} s before returning to the start pose.'
        )
        self._sleep_with_spin(self._return_wait_sec)

        self.get_logger().info(
            f'Driving back to start pose: {self._format_values(self._return_pose)}'
        )
        self._publish_command(self._return_pose)
        self._sleep_with_spin(self._return_settle_sec)
        self.get_logger().info('Cartesian position demo complete.')

    def _publish_command(self, command: list[float]) -> None:
        self._publisher.publish(
            make_cartesian_pose_command(
                self,
                command,
                self._command_frame_id,
                self._command_goal_time,
                self._command_interpolation_space,
            )
        )
        self.get_logger().info(
            'Published CartesianPoseCommand to '
            f'`{self._command_topic}` in frame `{self._command_frame_id}` '
            f'with goal_time={self._command_goal_time:.3f} s and '
            f'interpolation_space=`{self._command_interpolation_space}`: '
            f'{self._format_values(command)}'
        )

    def _wait_for_command_subscription(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'No subscribers detected on `{self._command_topic}` before publishing.'
        )

    def _sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _format_values(self, values: list[float]) -> str:
        rounded = [round(value, 4) for value in values]
        return str(rounded)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianPositionDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
