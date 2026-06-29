from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    controllers_file = ParameterFile(
        param_file=LaunchConfiguration('controllers_file'),
        allow_substs=True,
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        remappings=[('~/robot_description', '/robot_description')],
        output={'both': 'screen'},
    )

    joint_state_broadcaster_spawner = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            LaunchConfiguration('controller_manager_name'),
        ],
        output={'both': 'screen'},
    )

    cartesian_controller_spawners = []
    for controller_name in [
        'cartesian_position_controller',
        'cartesian_external_effort_controller',
    ]:
        cartesian_controller_spawners.append(
            Node(
                name=f'{controller_name}_spawner',
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller_name,
                    '--controller-manager',
                    LaunchConfiguration('controller_manager_name'),
                    '--inactive',
                ],
                output={'both': 'screen'},
            )
        )

    description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('trossen_arm_description'),
                    'launch',
                    'display.launch.py',
                ]
            )
        ),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'robot_description': LaunchConfiguration('robot_description'),
            'use_joint_pub_gui': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    return [
        controller_manager_node,
        description_launch_include,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[joint_state_broadcaster_spawner] + cartesian_controller_spawners,
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='wxai',
            choices=['wxai'],
            description='Trossen Arm model codename.',
        ),
        DeclareLaunchArgument(
            'arm_variant',
            default_value='base',
            choices=['base', 'leader', 'follower'],
            description='End effector variant of the Trossen Arm.',
        ),
        DeclareLaunchArgument(
            'arm_side',
            default_value='none',
            choices=['none', 'left', 'right'],
            description='Side of the Trossen Arm.',
        ),
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.2',
            description='IP address of the robot.',
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            choices=['real', 'mock_components'],
            description='Use the real Trossen hardware interface or mock components.',
        ),
        DeclareLaunchArgument(
            'use_world_frame',
            default_value='false',
            choices=['true', 'false'],
            description='Include the world frame in the robot description.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz through the description launch file.',
        ),
        DeclareLaunchArgument(
            'controller_manager_name',
            default_value='/controller_manager',
            description='Controller manager node name used by spawner commands.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('trossen_arm_examples'),
                    'config',
                    'cartesian_controllers.yaml',
                ]
            ),
            description='Controller manager YAML for the Cartesian examples.',
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command(
                [
                    FindExecutable(name='xacro'),
                    ' ',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('trossen_arm_description'),
                            'urdf',
                            LaunchConfiguration('robot_model'),
                        ]
                    ),
                    '.urdf.xacro ',
                    'use_world_frame:=',
                    LaunchConfiguration('use_world_frame'),
                    ' ',
                    'variant:=',
                    LaunchConfiguration('arm_variant'),
                    ' ',
                    'arm_side:=',
                    LaunchConfiguration('arm_side'),
                    ' ',
                    'ros2_control_hardware_type:=',
                    LaunchConfiguration('ros2_control_hardware_type'),
                    ' ',
                    'ip_address:=',
                    LaunchConfiguration('ip_address'),
                ]
            ),
            description='Robot description XML. Defaults to the Trossen xacro output.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
