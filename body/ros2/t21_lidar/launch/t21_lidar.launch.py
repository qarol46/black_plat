from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description() -> LaunchDescription:
    declare_env_path_cmd = DeclareLaunchArgument(
        'env_path',
        default_value='/data',
        description='Root directory containing configs/, maps/, etc.',
    )

    return LaunchDescription([
        declare_env_path_cmd,
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    env_path       = LaunchConfiguration('env_path').perform(context)
    velodyne_share = FindPackageShare('velodyne_pointcloud').perform(context)

    # Конфиги берутся из env_path, калибровка — из пакета (не меняется)
    lidar_params_file   = PathJoinSubstitution([
        env_path, 'configs', 'body', 'lidar_config', 'VLP16-velodyne_driver_node-params.yaml'
    ]).perform(context)

    convert_params_file = PathJoinSubstitution([
        env_path, 'configs', 'body', 'lidar_config', 'VLP16-velodyne_transform_node-params.yaml'
    ]).perform(context)

    calibration_file    = PathJoinSubstitution([
        velodyne_share, 'params', 'VLP16db.yaml'
    ]).perform(context)

    return [
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            output='both',
            parameters=[ParameterFile(lidar_params_file)],
        ),
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            output='both',
            parameters=[
                ParameterFile(convert_params_file),
                {'calibration': calibration_file},
            ],
        ),
    ]