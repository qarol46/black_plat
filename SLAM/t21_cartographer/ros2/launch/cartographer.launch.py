import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    pkg_name = 't21_cartographer'
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1')

    # Get the package share directory
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    
    # Set configuration directory using actual path
    configuration_directory = LaunchConfiguration(
        'configuration_directory',
        default=os.path.join(pkg_share, 'config')#, 'configuration_files')
    )
    # Configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer.lua')

    #Nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        remappings=[
                ( '/points2', '/velodyne_points'),
                ('/imu', 'imu/data')],
        )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
        )

    #Launch file
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld