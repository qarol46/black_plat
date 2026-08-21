from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare

PKG = FindPackageShare('tracked_description')

def generate_launch_description() -> LaunchDescription:

    declare_env_path_cmd = DeclareLaunchArgument(
        'env_path', default_value='/data',
        description='Root directory containing configs/, maps/, etc.',
    )
    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix', default_value='',
    )
    declare_use_lidar_cmd = DeclareLaunchArgument(
        'use_lidar', default_value='True',
        description='Use VLP-16',
    )
    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera', default_value='True',
        description='Run RGBD camera nodes',
    )
    declare_use_imu_cmd = DeclareLaunchArgument(
        'use_imu', default_value='True',
        description='Use Xsens IMU',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='False',
        description='Use RViz',
    )

    env_path_arg = [('env_path', LaunchConfiguration('env_path'))]

    start_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('t21_lidar'), 'launch', 't21_lidar.launch.py'
        ])]),
        launch_arguments=env_path_arg,
        condition=IfCondition(LaunchConfiguration('use_lidar')),
    )

    start_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('t21_camera'), 'launch', 't21_camera.launch.py'
        ])]),
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('bluespace_ai_xsens_mti_driver'), 'launch', 'xsens_mti_node.launch.py'
        ])]),
        launch_arguments=env_path_arg,
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([PKG, 'urdf', 't21.urdf.xacro']), ' ',
            'prefix:=', LaunchConfiguration('prefix'),
        ])
    }

    controller_parameters = PathJoinSubstitution([PKG, 'config', 'controllers.yaml'])
    cm_ns = '/controller_manager'
    rviz_config_file = PathJoinSubstitution([LaunchConfiguration('env_path'), 'rviz2', 'navigation.rviz'])
    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', cm_ns,
                '--controller-type', 'joint_state_broadcaster/JointStateBroadcaster',
                '--param-file', controller_parameters,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', cm_ns,
                '--controller-type', 'diff_drive_controller/DiffDriveController',
                '--param-file', controller_parameters,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=[
                'geom_position_controller',
                '--controller-manager', cm_ns,
                '--controller-type', 'forward_command_controller/ForwardCommandController',
                '--param-file', controller_parameters,
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]

    return LaunchDescription([
        declare_env_path_cmd,
        declare_prefix_cmd,
        declare_use_lidar_cmd,
        declare_use_camera_cmd,
        declare_use_imu_cmd,
        declare_use_rviz,
        start_lidar,
        start_camera,
        imu_launch,
    ] + nodes)