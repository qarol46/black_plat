import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(get_package_share_directory('tracked_description'), 'urdf', 'tracked_robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')
    slam_params_file = os.path.join(share_dir, 'config', 'mapper_params.yaml')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'min_height': 0.05,      
                'max_height': 0.5,
                'angle_min': -3.14159,   
                'angle_max': 3.14159,   
                'angle_increment': 0.0087, 
                'scan_time': 0.1,
                'range_min': 0.8,
                'range_max': 40.0,       
                'use_inf': True,
                'inf_epsilon': 1.0,
                'target_frame': 'base_link',
                'transform_tolerance': 0.5,   
                'concurrency_level': 1,
                # QoS for publisher
                #'qos_overrides./scan.publisher.reliability': 'best_effort',
                #'qos_overrides./scan.publisher.durability': 'volatile',
                #'qos_overrides./scan.publisher.depth': 50,
            }],
            remappings=[
                ('cloud_in', '/lio_sam/deskew/cloud_deskewed'),
                ('scan', '/scan_velodyne')
            ],
            output='screen'
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
            remappings=[
                ('scan', '/scan_velodyne'),
                ('/odom', '/lio_sam/mapping/odometry'),
                ('map', '/map'),
                ('map_metadata', '/map_metadata')
            ]
        ),
    ])
