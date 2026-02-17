# t21_lidar.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    # Получаем пути с помощью подстановок
    pkg_share = FindPackageShare("t21_lidar").perform(context)
    velodyne_share = FindPackageShare("velodyne_pointcloud").perform(context)
    
    # Формируем пути к файлам
    convert_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'VLP16-velodyne_transform_node-params.yaml'
    ]).perform(context)
    
    calibration_file = PathJoinSubstitution([
        velodyne_share, 'params', 'VLP16db.yaml'
    ]).perform(context)
    
    lidar_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'VLP16-velodyne_driver_node-params.yaml'
    ]).perform(context)

    nodes = [
        # Драйвер лидара
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            output='both',
            parameters=[ParameterFile(lidar_params_file)]
        ),

        # Трансформация точек
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            output='both',
            parameters=[
                ParameterFile(convert_params_file),
                {"calibration": calibration_file}
            ]
        ),
    ]
    return nodes