from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        DeclareLaunchArgument(
            'env_path', default_value='/data',
            description='Root directory containing configs/, maps/, etc.',
        ),
        Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                LaunchConfiguration('env_path'),
                'configs', 'body', 'imu_config', 'xsens_mti_node.yaml'
            ])],
        ),
    ])