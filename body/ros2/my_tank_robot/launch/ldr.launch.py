import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Запускаем драйвер Velodyne
    vlp_cmd = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'velodyne', 'velodyne-all-nodes-VLP16-launch.py'
        ],
        output='screen'
    )

    # 2) Нода записи лидара (ваш Python-скрипт), но обёрнута в TimerAction
    #    Чтобы подождать 5 секунд (пример) после запуска vlp_cmd.
    ldr_spawn = TimerAction(
        period=5.0,  # задержка в секундах
        actions=[
            Node(
                package='my_tank_robot',
                executable='ldr',    # ваш исполняемый файл
                name='lidar_data_record',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        #vlp_cmd,
        ldr_spawn,
    ])
