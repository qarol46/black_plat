from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

package_name='t21_rtabmap'

def generate_launch_description():
    # Путь к YAML-файлу параметров
    default_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'camera.yaml'
    )
    
    # Corrected remappings list
    # camera_remaps = [
    #     ('/camera/color/camera_info', '/camera/camera_info'),
    #     ('/camera/color/image_raw', '/camera/image_raw'),
    #     ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
    #     ('/camera/depth/color/points', '/camera/points'),
        # ('/camera/color/metadata','/camera0/color/metadata'),
        # ('/camera/depth/camera_info','/camera0/depth/camera_info'),
        # ('/camera/depth/metadata','/camera0/depth/metadata'),
        # ('/camera/extrinsics/depth_to_color', '/camera0/extrinsics/depth_to_color'),
        # ('/camera/extrinsics/depth_to_infra1', '/camera0/extrinsics/depth_to_infra1'),
        # ('/camera/extrinsics/depth_to_infra2', '/camera0/extrinsics/depth_to_infra2'),
        # ('/camera/infra1/camera_info','/camera0/infra1/camera_info'),
        # ('/camera/infra1/image_rect_raw','/camera0/infra1/image_rect_raw'),
        # ('/camera/infra1/metadata','/camera0/infra1/metadata'),
        # ('/camera/infra2/camera_info','/camera0/infra2/camera_info'),
        # ('/camera/infra2/image_rect_raw','/camera0/infra2/image_rect_raw'),
        # ('/camera/infra2/metadata','/camera0/infra2/metadata'),
        # ('/camera/camera_info','/camera0/camera_info'),
        # ('/camera/depth/image_rect_raw','/camera0/depth/image_rect_raw'),
        # ('/camera/image_raw','/camera0/image_raw'),
        # ('/camera/points','/camera0/points'),
        # ('/camera/color/image_raw','camera0/color/image_raw'),
        # ('/camera/color/camera_info','/camera0/color/camera_info'),

    #]
    
    return LaunchDescription([
        # Аргументы для переопределения
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value=''),
        DeclareLaunchArgument('config_file', default_value=default_params_file),
        
        # Corrected camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name=LaunchConfiguration('camera_name'),
            namespace=LaunchConfiguration('camera_namespace'),
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
        )
    ])