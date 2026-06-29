from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    demo_start_delay_sec = float(LaunchConfiguration('demo_start_delay_sec').perform(context))

    demo_node = Node(
        package='trossen_arm_examples',
        executable='cartesian_button_press_demo',
        name='cartesian_button_press_demo',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('trossen_arm_examples'),
                    'config',
                    'cartesian_button_press_demo.yaml',
                ]
            ),
            {
                'position_command_topic': LaunchConfiguration('position_command_topic'),
                'effort_command_topic': LaunchConfiguration('effort_command_topic'),
                'auto_activate_controllers': LaunchConfiguration('auto_activate_controllers'),
                'controller_manager_name': LaunchConfiguration('controller_manager_name'),
                'position_controller_name': LaunchConfiguration('position_controller_name'),
                'effort_controller_name': LaunchConfiguration('effort_controller_name'),
                'state_topic': LaunchConfiguration('state_topic'),
                'cartesian_state_name': LaunchConfiguration('cartesian_state_name'),
                'start_pose': LaunchConfiguration('start_pose'),
                'press_direction': LaunchConfiguration('press_direction'),
                'press_force_n': LaunchConfiguration('press_force_n'),
                'torque_xyz_nm': LaunchConfiguration('torque_xyz_nm'),
                'max_press_travel_m': LaunchConfiguration('max_press_travel_m'),
                'press_duration_sec': LaunchConfiguration('press_duration_sec'),
                'press_rate_hz': LaunchConfiguration('press_rate_hz'),
                'stiffness': LaunchConfiguration('stiffness'),
                'damping': LaunchConfiguration('damping'),
                'ramp_time_sec': LaunchConfiguration('ramp_time_sec'),
                'max_force_norm_n': LaunchConfiguration('max_force_norm_n'),
                'max_torque_norm_nm': LaunchConfiguration('max_torque_norm_nm'),
                'return_pose': LaunchConfiguration('return_pose'),
                'start_settle_sec': LaunchConfiguration('start_settle_sec'),
                'return_settle_sec': LaunchConfiguration('return_settle_sec'),
                'return_rate_hz': LaunchConfiguration('return_rate_hz'),
                'zero_wrench_count': LaunchConfiguration('zero_wrench_count'),
                'zero_wrench_dt_sec': LaunchConfiguration('zero_wrench_dt_sec'),
            },
        ],
    )

    return [
        TimerAction(
            period=demo_start_delay_sec,
            actions=[demo_node],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('trossen_arm_examples'),
                    'launch',
                    'cartesian_control.launch.py',
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration('start_controller_manager')),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'arm_variant': LaunchConfiguration('arm_variant'),
            'arm_side': LaunchConfiguration('arm_side'),
            'ip_address': LaunchConfiguration('ip_address'),
            'ros2_control_hardware_type': LaunchConfiguration('ros2_control_hardware_type'),
            'use_world_frame': LaunchConfiguration('use_world_frame'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'controller_manager_name': LaunchConfiguration('controller_manager_name'),
            'controllers_file': LaunchConfiguration('controllers_file'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_controller_manager',
                default_value='true',
                choices=['true', 'false'],
                description='Start ros2_control_node and spawn the Cartesian controllers.',
            ),
            DeclareLaunchArgument('robot_model', default_value='wxai', choices=['wxai']),
            DeclareLaunchArgument('arm_variant', default_value='base', choices=['base', 'leader', 'follower']),
            DeclareLaunchArgument('arm_side', default_value='none', choices=['none', 'left', 'right']),
            DeclareLaunchArgument('ip_address', default_value='192.168.1.2'),
            DeclareLaunchArgument('ros2_control_hardware_type', default_value='real', choices=['real', 'mock_components']),
            DeclareLaunchArgument('use_world_frame', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('controller_manager_name', default_value='/controller_manager'),
            DeclareLaunchArgument(
                'controllers_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('trossen_arm_examples'),
                        'config',
                        'cartesian_controllers.yaml',
                    ]
                ),
            ),
            DeclareLaunchArgument('position_command_topic', default_value='/cartesian_position_controller/command'),
            DeclareLaunchArgument('effort_command_topic', default_value='/cartesian_external_effort_controller/command'),
            DeclareLaunchArgument('auto_activate_controllers', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('position_controller_name', default_value='cartesian_position_controller'),
            DeclareLaunchArgument('effort_controller_name', default_value='cartesian_external_effort_controller'),
            DeclareLaunchArgument('state_topic', default_value='/dynamic_joint_states'),
            DeclareLaunchArgument('cartesian_state_name', default_value='cartesian'),
            DeclareLaunchArgument('start_pose', default_value='[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]'),
            DeclareLaunchArgument('press_direction', default_value='[1.0, 0.0, 0.0]'),
            DeclareLaunchArgument('press_force_n', default_value='4.0'),
            DeclareLaunchArgument('torque_xyz_nm', default_value='[0.0, 0.0, 0.0]'),
            DeclareLaunchArgument('max_press_travel_m', default_value='0.1'),
            DeclareLaunchArgument('press_duration_sec', default_value='10.0'),
            DeclareLaunchArgument('press_rate_hz', default_value='100.0'),
            DeclareLaunchArgument('stiffness', default_value='[120.0, 120.0, 120.0, 8.0, 8.0, 8.0]'),
            DeclareLaunchArgument('damping', default_value='[3.0, 3.0, 3.0, 0.25, 0.25, 0.25]'),
            DeclareLaunchArgument('ramp_time_sec', default_value='0.5'),
            DeclareLaunchArgument('max_force_norm_n', default_value='8.0'),
            DeclareLaunchArgument('max_torque_norm_nm', default_value='1.0'),
            DeclareLaunchArgument('return_pose', default_value='[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]'),
            DeclareLaunchArgument('start_settle_sec', default_value='2.0'),
            DeclareLaunchArgument('return_settle_sec', default_value='5.0'),
            DeclareLaunchArgument('return_rate_hz', default_value='10.0'),
            DeclareLaunchArgument('zero_wrench_count', default_value='10'),
            DeclareLaunchArgument('zero_wrench_dt_sec', default_value='0.05'),
            DeclareLaunchArgument(
                'demo_start_delay_sec',
                default_value='5.0',
                description='Delay before publishing the demo sequence after bringup starts.',
            ),
            bringup_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
