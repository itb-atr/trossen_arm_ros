import time

from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from trossen_arm_examples.controller_manager_helpers import ControllerManagerClient
from trossen_arm_examples.message_helpers import (
    declare_dynamic_parameter,
    get_bool_parameter,
    get_float_array_parameter,
    get_string_parameter,
    make_float64_array,
)


class CartesianButtonPressDemo(Node):
    def __init__(self) -> None:
        super().__init__('cartesian_button_press_demo')

        declare_dynamic_parameter(self, 'position_command_topic', '/cartesian_position_controller/command')
        declare_dynamic_parameter(self, 'effort_command_topic', '/cartesian_external_effort_controller/command')
        declare_dynamic_parameter(self, 'auto_activate_controllers', False)
        declare_dynamic_parameter(self, 'controller_manager_name', '/controller_manager')
        declare_dynamic_parameter(self, 'position_controller_name', 'cartesian_position_controller')
        declare_dynamic_parameter(self, 'effort_controller_name', 'cartesian_external_effort_controller')
        declare_dynamic_parameter(self, 'state_topic', '/dynamic_joint_states')
        declare_dynamic_parameter(self, 'cartesian_state_name', 'cartesian')
        declare_dynamic_parameter(self, 'start_pose', '[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'press_direction', '[1.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'press_force_n', 4.0)
        declare_dynamic_parameter(self, 'torque_xyz_nm', '[0.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'max_press_travel_m', 0.1)
        declare_dynamic_parameter(self, 'press_duration_sec', 10.0)
        declare_dynamic_parameter(self, 'press_rate_hz', 100.0)
        declare_dynamic_parameter(self, 'stiffness', '[120.0, 120.0, 120.0, 8.0, 8.0, 8.0]')
        declare_dynamic_parameter(self, 'damping', '[3.0, 3.0, 3.0, 0.25, 0.25, 0.25]')
        declare_dynamic_parameter(self, 'ramp_time_sec', 0.5)
        declare_dynamic_parameter(self, 'max_force_norm_n', 8.0)
        declare_dynamic_parameter(self, 'max_torque_norm_nm', 1.0)
        declare_dynamic_parameter(self, 'return_pose', '[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]')
        declare_dynamic_parameter(self, 'start_settle_sec', 2.0)
        declare_dynamic_parameter(self, 'return_settle_sec', 5.0)
        declare_dynamic_parameter(self, 'return_rate_hz', 10.0)
        declare_dynamic_parameter(self, 'zero_wrench_count', 10)
        declare_dynamic_parameter(self, 'zero_wrench_dt_sec', 0.05)

        self._position_topic = get_string_parameter(self, 'position_command_topic')
        self._effort_topic = get_string_parameter(self, 'effort_command_topic')
        self._state_topic = get_string_parameter(self, 'state_topic')
        self._cartesian_state_name = get_string_parameter(self, 'cartesian_state_name')
        self._start_pose = get_float_array_parameter(self, 'start_pose', 6)
        self._press_direction = self._unit_vector(
            get_float_array_parameter(self, 'press_direction', 3)
        )
        if self._press_direction is None:
            raise ValueError('Parameter `press_direction` must not be zero.')
        self._press_force_n = float(self.get_parameter('press_force_n').value)
        self._torque_xyz_nm = get_float_array_parameter(self, 'torque_xyz_nm', 3)
        self._max_press_travel_m = float(self.get_parameter('max_press_travel_m').value)
        self._press_duration_sec = float(self.get_parameter('press_duration_sec').value)
        self._press_rate_hz = float(self.get_parameter('press_rate_hz').value)
        self._stiffness = get_float_array_parameter(self, 'stiffness', 6)
        self._damping = get_float_array_parameter(self, 'damping', 6)
        self._ramp_time_sec = float(self.get_parameter('ramp_time_sec').value)
        self._max_force_norm_n = float(self.get_parameter('max_force_norm_n').value)
        self._max_torque_norm_nm = float(self.get_parameter('max_torque_norm_nm').value)
        self._return_pose = get_float_array_parameter(self, 'return_pose', 6)
        self._start_settle_sec = float(self.get_parameter('start_settle_sec').value)
        self._return_settle_sec = float(self.get_parameter('return_settle_sec').value)
        self._return_rate_hz = float(self.get_parameter('return_rate_hz').value)
        self._zero_wrench_count = int(self.get_parameter('zero_wrench_count').value)
        self._zero_wrench_dt_sec = float(self.get_parameter('zero_wrench_dt_sec').value)

        if self._max_press_travel_m <= 0.0:
            raise ValueError('Parameter `max_press_travel_m` must be greater than zero.')
        if self._press_force_n <= 0.0:
            raise ValueError('Parameter `press_force_n` must be greater than zero.')
        if self._press_duration_sec <= 0.0:
            raise ValueError('Parameter `press_duration_sec` must be greater than zero.')
        if self._press_rate_hz <= 0.0:
            raise ValueError('Parameter `press_rate_hz` must be greater than zero.')
        if self._ramp_time_sec < 0.0:
            raise ValueError('Parameter `ramp_time_sec` must be zero or greater.')
        if self._max_force_norm_n <= 0.0:
            raise ValueError('Parameter `max_force_norm_n` must be greater than zero.')
        if self._max_torque_norm_nm <= 0.0:
            raise ValueError('Parameter `max_torque_norm_nm` must be greater than zero.')
        if self._return_settle_sec <= 0.0:
            raise ValueError('Parameter `return_settle_sec` must be greater than zero.')
        if self._return_rate_hz <= 0.0:
            raise ValueError('Parameter `return_rate_hz` must be greater than zero.')
        if self._zero_wrench_count < 1:
            raise ValueError('Parameter `zero_wrench_count` must be at least one.')

        self._latest_cartesian_pose = None
        self._latest_cartesian_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._press_stop_pose = None

        self._position_pub = self.create_publisher(Float64MultiArray, self._position_topic, 10)
        self._effort_pub = self.create_publisher(Float64MultiArray, self._effort_topic, 10)
        self._state_sub = self.create_subscription(
            DynamicJointState,
            self._state_topic,
            self._dynamic_joint_state_callback,
            10,
        )

    def run(self) -> None:
        manager = None
        position_controller_name = get_string_parameter(self, 'position_controller_name')
        effort_controller_name = get_string_parameter(self, 'effort_controller_name')

        if get_bool_parameter(self, 'auto_activate_controllers'):
            manager = ControllerManagerClient(
                self, get_string_parameter(self, 'controller_manager_name')
            )
            manager.ensure_loaded_configured(
                [position_controller_name, effort_controller_name]
            )

        if manager is not None:
            manager.ensure_active(
                [position_controller_name],
                deactivate_controller_names=[effort_controller_name],
            )
            self._wait_for_position_subscription()
        else:
            self._wait_for_subscriptions()

        self.get_logger().info(
            'Driving to start pose before applying external effort: '
            f'{self._format_values(self._start_pose)}'
        )
        self._position_pub.publish(make_float64_array(self._start_pose))
        self._sleep_with_spin(self._start_settle_sec)
        self.get_logger().info('Reached external-effort start pose.')

        if manager is not None:
            manager.ensure_active(
                [effort_controller_name],
                deactivate_controller_names=[position_controller_name],
            )
            self._wait_for_effort_subscription()

        press_error = None
        try:
            self._run_press_loop()
        except Exception as exc:
            press_error = exc
            self.get_logger().error(f'External-effort phase failed: {exc}')
        finally:
            self.get_logger().info('Ramping external effort back to zero.')
            self._publish_zero_wrench_sequence()

        if manager is not None:
            manager.ensure_active(
                [position_controller_name],
                deactivate_controller_names=[effort_controller_name],
            )
            self._wait_for_position_subscription()

        self._slow_return_to_start()
        self.get_logger().info('Cartesian external effort demo complete.')

        if press_error is not None:
            raise press_error

    def _run_press_loop(self) -> None:
        reference_pose = self._wait_for_cartesian_pose(timeout_sec=1.0) or self._start_pose
        hold_pose = list(reference_pose)
        if self._latest_cartesian_pose is None:
            raise RuntimeError(
                f'No Cartesian state received on `{self._state_topic}`. '
                'Cannot run impedance-stabilized external effort safely.'
            )

        start_press_coordinate = self._dot(hold_pose[:3], self._press_direction)
        dt = 1.0 / self._press_rate_hz
        start_time = time.monotonic()

        self.get_logger().info(
            'Captured hold pose for virtual impedance: '
            f'{self._format_values(hold_pose)}'
        )
        self.get_logger().info(
            'Applying ramped press force with impedance stabilization. '
            f'press_direction={self._format_values(self._press_direction)}, '
            f'press_force={self._press_force_n:.3f} N'
        )
        self.get_logger().info(
            f'Stopping at {self._max_press_travel_m:.3f} m travel or '
            f'{self._press_duration_sec:.1f} s, whichever comes first.'
        )

        while rclpy.ok():
            elapsed = time.monotonic() - start_time
            if elapsed >= self._press_duration_sec:
                self._record_press_stop_pose()
                self.get_logger().info(
                    f'Stopping press: reached duration limit {elapsed:.2f} s.'
                )
                return

            current_pose = list(self._latest_cartesian_pose)
            current_velocity = list(self._latest_cartesian_velocity)
            current_coordinate = self._dot(current_pose[:3], self._press_direction)
            press_travel = current_coordinate - start_press_coordinate
            if press_travel >= self._max_press_travel_m:
                self._record_press_stop_pose()
                self.get_logger().info(
                    f'Stopping press: reached travel limit {press_travel:.4f} m.'
                )
                return

            effort = self._compute_impedance_press_effort(
                hold_pose=hold_pose,
                current_pose=current_pose,
                current_velocity=current_velocity,
                elapsed=elapsed,
            )
            self._effort_pub.publish(make_float64_array(effort))
            self._sleep_with_spin(dt)

    def _compute_impedance_press_effort(
        self,
        hold_pose: list[float],
        current_pose: list[float],
        current_velocity: list[float],
        elapsed: float,
    ) -> list[float]:
        ramp = min(1.0, elapsed / max(self._ramp_time_sec, 1e-6))

        effort = []
        for index in range(6):
            pose_error = hold_pose[index] - current_pose[index]
            impedance_effort = (
                self._stiffness[index] * pose_error
                - self._damping[index] * current_velocity[index]
            )
            effort.append(impedance_effort)

        for index in range(3):
            effort[index] += ramp * self._press_force_n * self._press_direction[index]
            effort[index + 3] += ramp * self._torque_xyz_nm[index]

        effort[:3] = self._clamp_norm(
            effort[:3],
            max(self._max_force_norm_n, self._press_force_n + 3.0),
        )
        effort[3:] = self._clamp_norm(effort[3:], self._max_torque_norm_nm)
        return effort

    def _publish_zero_wrench_sequence(self) -> None:
        zero_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for _ in range(self._zero_wrench_count):
            self._effort_pub.publish(make_float64_array(zero_wrench))
            self._sleep_with_spin(self._zero_wrench_dt_sec)

    def _slow_return_to_start(self) -> None:
        if self._press_stop_pose is not None:
            return_start_pose = list(self._press_stop_pose)
        elif self._latest_cartesian_pose is not None:
            return_start_pose = list(self._latest_cartesian_pose)
        else:
            return_start_pose = list(self._return_pose)

        self.get_logger().info(
            'Slowly returning to start pose over '
            f'{self._return_settle_sec:.1f} s: {self._format_values(self._return_pose)}'
        )

        steps = max(1, int(self._return_settle_sec * self._return_rate_hz))
        dt = self._return_settle_sec / steps
        for step_index in range(1, steps + 1):
            alpha = step_index / steps
            target_pose = [
                start + ((goal - start) * alpha)
                for start, goal in zip(return_start_pose, self._return_pose)
            ]
            self._position_pub.publish(make_float64_array(target_pose))
            self._sleep_with_spin(dt)

    def _record_press_stop_pose(self) -> None:
        if self._latest_cartesian_pose is not None:
            self._press_stop_pose = list(self._latest_cartesian_pose)
        else:
            self._press_stop_pose = list(self._return_pose)

    def _wait_for_subscriptions(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            have_position = self._position_pub.get_subscription_count() > 0
            have_effort = self._effort_pub.get_subscription_count() > 0
            if have_position and have_effort:
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            'One or more command topics had no subscribers before publishing.'
        )

    def _wait_for_cartesian_pose(self, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._latest_cartesian_pose is not None:
                return list(self._latest_cartesian_pose)
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def _wait_for_position_subscription(self, timeout_sec: float = 5.0) -> None:
        self._wait_for_single_subscription(
            self._position_pub, self._position_topic, timeout_sec
        )

    def _wait_for_effort_subscription(self, timeout_sec: float = 5.0) -> None:
        self._wait_for_single_subscription(
            self._effort_pub, self._effort_topic, timeout_sec
        )

    def _wait_for_single_subscription(self, publisher, topic_name: str, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            f'No subscribers detected on `{topic_name}` before publishing.'
        )

    def _sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _dynamic_joint_state_callback(self, msg: DynamicJointState) -> None:
        try:
            state_index = list(msg.joint_names).index(self._cartesian_state_name)
        except ValueError:
            return

        interface_values = msg.interface_values[state_index]
        values_by_name = dict(
            zip(interface_values.interface_names, interface_values.values)
        )
        position_keys = [
            ('position.x', 'positions.x', 'x'),
            ('position.y', 'positions.y', 'y'),
            ('position.z', 'positions.z', 'z'),
            ('position.rx', 'positions.rx', 'rx'),
            ('position.ry', 'positions.ry', 'ry'),
            ('position.rz', 'positions.rz', 'rz'),
        ]
        velocity_keys = [
            ('velocity.x', 'velocities.x'),
            ('velocity.y', 'velocities.y'),
            ('velocity.z', 'velocities.z'),
            ('velocity.rx', 'velocities.rx'),
            ('velocity.ry', 'velocities.ry'),
            ('velocity.rz', 'velocities.rz'),
        ]

        pose = self._read_dynamic_values(values_by_name, position_keys)
        if pose is None:
            return

        velocity = self._read_dynamic_values(values_by_name, velocity_keys)
        if velocity is None:
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self._latest_cartesian_pose = pose
        self._latest_cartesian_velocity = velocity

    def _read_dynamic_values(self, values_by_name: dict[str, float], key_options):
        values = []
        for options in key_options:
            value = None
            for key in options:
                if key in values_by_name:
                    value = float(values_by_name[key])
                    break
            if value is None:
                return None
            values.append(value)
        return values

    def _unit_vector(self, values: list[float]):
        norm = sum(value * value for value in values) ** 0.5
        if norm < 1e-9:
            return None
        return [value / norm for value in values]

    def _clamp_norm(self, values: list[float], max_norm: float) -> list[float]:
        norm = sum(value * value for value in values) ** 0.5
        if norm <= max_norm or norm < 1e-9:
            return list(values)
        scale = max_norm / norm
        return [value * scale for value in values]

    def _dot(self, lhs: list[float], rhs: list[float]) -> float:
        return sum(left * right for left, right in zip(lhs, rhs))

    def _format_values(self, values: list[float]) -> str:
        rounded = [round(value, 4) for value in values]
        return str(rounded)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianButtonPressDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
