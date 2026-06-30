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
        executable='cartesian_position_demo',
        name='cartesian_position_demo',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('trossen_arm_examples'),
                    'config',
                    'cartesian_position_demo.yaml',
                ]
            ),
            {
                'command_topic': LaunchConfiguration('command_topic'),
                'command_frame_id': LaunchConfiguration('command_frame_id'),
                'auto_activate_controller': LaunchConfiguration('auto_activate_controller'),
                'controller_manager_name': LaunchConfiguration('controller_manager_name'),
                'controller_name': LaunchConfiguration('controller_name'),
                'deactivate_controller_name': LaunchConfiguration('deactivate_controller_name'),
                'start_pose': LaunchConfiguration('start_pose'),
                'base_frame_step': LaunchConfiguration('base_frame_step'),
                'step_count': LaunchConfiguration('step_count'),
                'return_pose': LaunchConfiguration('return_pose'),
                'publish_delay_sec': LaunchConfiguration('publish_delay_sec'),
                'start_settle_sec': LaunchConfiguration('start_settle_sec'),
                'step_settle_sec': LaunchConfiguration('step_settle_sec'),
                'return_wait_sec': LaunchConfiguration('return_wait_sec'),
                'return_settle_sec': LaunchConfiguration('return_settle_sec'),
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
            DeclareLaunchArgument('command_topic', default_value='/cartesian_position_controller/command'),
            DeclareLaunchArgument('command_frame_id', default_value='base_link'),
            DeclareLaunchArgument('auto_activate_controller', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('controller_name', default_value='cartesian_position_controller'),
            DeclareLaunchArgument('deactivate_controller_name', default_value='cartesian_external_effort_controller'),
            DeclareLaunchArgument('start_pose', default_value='[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]'),
            DeclareLaunchArgument('base_frame_step', default_value='[0.02, 0.0, 0.0, 0.0, 0.0, 0.0]'),
            DeclareLaunchArgument('step_count', default_value='5'),
            DeclareLaunchArgument('return_pose', default_value='[0.25, 0.0, 0.35, 0.0, 0.0, 0.0]'),
            DeclareLaunchArgument('publish_delay_sec', default_value='0.5'),
            DeclareLaunchArgument('start_settle_sec', default_value='2.0'),
            DeclareLaunchArgument('step_settle_sec', default_value='1.0'),
            DeclareLaunchArgument('return_wait_sec', default_value='5.0'),
            DeclareLaunchArgument('return_settle_sec', default_value='2.0'),
            DeclareLaunchArgument(
                'demo_start_delay_sec',
                default_value='5.0',
                description='Delay before publishing the demo command after bringup starts.',
            ),
            bringup_launch,
            OpaqueFunction(function=launch_setup),
        ]
    )
