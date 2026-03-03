from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    env_path                = LaunchConfiguration('env_path')
    use_sim_time            = LaunchConfiguration('use_sim_time')
    resolution              = LaunchConfiguration('resolution')
    publish_period_sec      = LaunchConfiguration('publish_period_sec')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename  = LaunchConfiguration('configuration_basename')

    declare_env_path_cmd = DeclareLaunchArgument(
        'env_path',
        default_value='/data',
        description='Root directory containing configs/, maps/, etc.',
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )
    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of the occupancy grid',
    )
    declare_publish_period_cmd = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1',
        description='Occupancy grid publish period (seconds)',
    )
    declare_configuration_directory_cmd = DeclareLaunchArgument(
        'configuration_directory',
        default_value=PathJoinSubstitution([
            env_path, 'configs', 'slam', 'cartographer_config'
        ]),
        description='Directory containing cartographer .lua config files',
    )
    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer.lua',
        description='Cartographer configuration filename',
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('/points2', '/velodyne_points'),
            ('/imu',     '/imu/data'),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
    )

    ld = LaunchDescription()
    ld.add_action(declare_env_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_cmd)
    ld.add_action(declare_configuration_directory_cmd)
    ld.add_action(declare_configuration_basename_cmd)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    return ld