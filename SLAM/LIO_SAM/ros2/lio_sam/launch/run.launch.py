import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    env_path       = LaunchConfiguration('env_path')
    parameter_file = LaunchConfiguration('params_file')

    declare_env_path_cmd = DeclareLaunchArgument(
        'env_path',
        default_value='/data',
        description='Root directory containing configs/, maps/, etc.',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            env_path, 'configs', 'slam', 'lio_sam_config', 'params.yaml'
        ]),
        description='Full path to the lio_sam parameters file.',
    )

    return LaunchDescription([

        declare_env_path_cmd,
        declare_params_file_cmd,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
            ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='-0.111 0.092 0.118 0.0 0.0 0.0 lidar_link base_link'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        #     ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': Command(['xacro', ' ', xacro_path])
        #     }]
        # ),
        # Node(
        #     package='lio_sam',
        #     executable='lio_sam_imuPreintegration',
        #     name='lio_sam_imuPreintegration',
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        Node(
            package='lio_sam',
            executable='lio_sam_wheelInertialPreintegration',
            name='lio_sam_WI_Preintegration',
            parameters=[parameter_file],
            output='screen',
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),
    ])
