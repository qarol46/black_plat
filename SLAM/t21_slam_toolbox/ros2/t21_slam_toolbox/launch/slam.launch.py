# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path


package_name = 't21_slam_toolbox'

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    declare_env_path_argument = DeclareLaunchArgument(
        name='env_path',
        default_value='/data',
        description='Path to the directory containing the config files'
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            LaunchConfiguration('env_path'),
            'configs',
            'slam',
            'slam-toolbox_config',
            'slam.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node =  Node(
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
    )
    translate =  Node(
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
    ) 
    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        declare_use_sim_time_argument,
        declare_env_path_argument,
        declare_slam_params_file_cmd,
        translate,
        start_async_slam_toolbox_node
    ])