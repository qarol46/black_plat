import os
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node

M_PI=3.14159265359
package_name = 't21_slam_toolbox'
def generate_launch_description():
    
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
    ground_filter = Node(
    package=package_name,
    executable='ground_filter_node',
    name='ground_filter',
    parameters=[
            {
            # Основные параметры фильтрации
            'input_topic': '/velodyne_points',             # Входной топик облака точек
            'output_topic': '/velodyne_points_filtered',   # Выходной топик отфильтрованных точек
            'ground_threshold': 0.12,                     # Макс. расстояние до плоскости земли (аналог max_distance)
            'min_ground_points': 5000,                     # Мин. точек для определения плоскости земли
            'min_height': -0.3,                            # Мин. высота препятствий
            
            # Параметры TF-преобразований
            'use_tf': False,                              # Использовать ли преобразование координат
            'target_frame': 'base_link',                   # Целевая система координат (если use_tf=True)
            }
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),
        translate,
        #ground_filter,

    ])