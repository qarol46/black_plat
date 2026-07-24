from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    dsor_filter_node = Node(
        package='pcl_filter',
        executable='dsor_filter',
        name='dsor_filter',
        output='screen',
    )
    
    return LaunchDescription([
        dsor_filter_node
    ])
