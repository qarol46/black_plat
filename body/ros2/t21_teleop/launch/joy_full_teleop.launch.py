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
            executable='joy_full_teleop',
            output='screen',
            parameters=[{
                # оси движения
                'axis_lin': 1,
                'axis_ang': 0,
                'scale_lin': 0.5,
                'scale_ang': 1.0,

                # кнопки флиппера
                'btn_flip_up':   5,          # RB
                'btn_flip_down': 4,          # LB
                'flip_step_deg': 2.0,        # шаг, град

                'deadzone': 0.05
            }]
        ),
    ])
