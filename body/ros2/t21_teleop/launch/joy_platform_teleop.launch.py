from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ─── драйвер геймпада ───
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0      # важно: кнопки опрашиваются 20 Гц
            }]
        ),

        # ─── наш teleop ───
        Node(
            package='t21_teleop',
            executable='joy_platform_teleop',
            output='screen',
            parameters=[{
                # оси движения
                'axis_lin': 1,
                'axis_ang': 0,
                'scale_lin': 0.3,
                'scale_ang': 0.5,

                'deadzone': 0.05
            }]
        ),
    ])
