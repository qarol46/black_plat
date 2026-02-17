import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('my_tank_robot')
    rviz_config_file_name = 'super_config_file.rviz'
    xacro_path = PathJoinSubstitution([
        FindPackageShare("my_tank_robot"),
        "urdf",
        "tank_robot.xacro"
    ])
    rviz_config_path = os.path.join(
        get_package_share_directory('my_tank_robot'),
        'config',
        rviz_config_file_name)

    # Генерируем описание робота из xacro
    robot_desc = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_path
    ])

    # gazebo_cmd = ExecuteProcess(
    #     cmd=[
    #         'gazebo',
    #         '--verbose',
    #         os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world'),
    #         '-s', 'libgazebo_ros_factory.so'
    #     ],
    #     output='screen'
    # )
        # Запускаем Gazebo, но теперь вместо empty.world берём stairs.world
    gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            os.path.join(pkg_share, 'worlds', 'stairs.world'),  # <-- Заменили на stairs.world
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Указываем, что robot_description - это строка, а не YAML
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(robot_desc)
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_tank_robot',
            '-x', '0', '-y', '2.5', '-z', '0.3',
            '-R','0','-P','0','-Y','-1.57'
        ],
        output='screen'
    )

    spawner_broad = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{
            'use_sim_time': ParameterValue(True, value_type=bool)
        }],
        output='screen'
    )

    spawner_flippers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        parameters=[{
            'use_sim_time': ParameterValue(True, value_type=bool)
        }],
        output='screen'
    )
    rviz_spawner = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    
    spawner_wheels = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{
            'use_sim_time': ParameterValue(True, value_type=bool)
        }],
        output='screen'
    )
    or_spawner = Node(
            package='my_tank_robot',
            executable='or',
            name='or',
            output='screen'
    )
        # Добавление ноды ldv
    ldv_spawner =  Node(
            package='my_tank_robot',
            executable='ldv',
            name='ldv',
            output='screen'
    )
    cart_spawner = Node(
            package='my_tank_robot',
            executable='cart',
            name='cart',
            output='screen'
    )
    alg_spawner = Node(
            package='my_tank_robot',
            executable='alg',
            name='alg',
            output='screen'
    )
    vlp16_spawner = Node(
            package='my_tank_robot',
            executable='VLP_16_publisher_points',
            name='VLP_16_publisher_points',
            output='screen'
    )
    return LaunchDescription([
         gazebo_cmd,
         rsp_node,
         spawn_entity,
         spawner_broad,
         spawner_flippers,
         rviz_spawner,
        # #spawner_wheels,
         or_spawner,
        ldv_spawner,
        cart_spawner,
        alg_spawner,
        # vlp16_spawner
    ])


# ros2 launch tracked_description bringup_t21.launch.py
# ros2 launch my_tank_robot spawn_tank_robot.launch.py
# ros2 run tracked_description dummy 
# ros2 run my_tank_robot publisher_angle 






    
