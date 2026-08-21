from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_dict = {
        'clahe_clip_limit_sky': 2.0,
        'clahe_grid_size_sky': 2,
        'rect_sky_x': 70,
        'rect_sky_y': 0,
        'rect_sky_w': 500,
        'rect_sky_h': 150,
        'clahe_clip_limit_asphalt': 4.5,
        'clahe_grid_size_asphalt': 6,
        'rect_asphalt_x': 0,
        'rect_asphalt_y': 230,
        'rect_asphalt_w': 640,
        'rect_asphalt_h': 250,
        'gamma': 0.80,
    }


    clahe_node1 = Node(
        package='clahe_ros',
        executable='clahe_ros_node',
        name='clahe_ros_1',
        output='screen',
        parameters=[param_dict],
        remappings=[
            ('/image/raw', '/camera/infra1/image_rect_raw'),
            ('/image/filtered', '/camera/infra1/image_rect_filtered'),
        ]
    )
    clahe_node2 = Node(
        package='clahe_ros',
        executable='clahe_ros_node',
        name='clahe_ros_2',
        output='screen',
        parameters=[param_dict],
        remappings=[
            ('/image/raw', '/camera/infra2/image_rect_raw'),
            ('/image/filtered', '/camera/infra2/image_rect_filtered'),
        ]
    )
    depth = Node(
        package='depth_eval',
        executable='depth_quality_eval',
        name='depth_eval',
        output='screen',
    )
    return LaunchDescription([
        clahe_node1,
        clahe_node2,
        depth,
    ])
