from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{'deadzone': 0.05, 'autorepeat_rate': 30.0}]
    )
    teleop = Node(
        package='t21_teleop', executable='joy_flipper_teleop',
        parameters=[{
            'axis_index':   1,    # левый стик вертикаль
            'scale':        3.14,  # ±0.7 рад
            'deadzone':     0.05,
            'command_size': 1,
            'joint_index':  0
        }]
    )
    return LaunchDescription([joy, teleop])
