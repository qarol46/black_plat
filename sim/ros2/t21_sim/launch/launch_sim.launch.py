import os, yaml
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    package_name = 't21_sim'
    
    # Запуск rsp.launch.py для публикации robot_description

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': LaunchConfiguration("use_sim_time"), 'use_ros2_control': 'true'}.items()
    )
    
    #joystick = IncludeLaunchDescription(
    #           PythonLaunchDescriptionSource([os.path.join(
    #               get_package_share_directory(package_name),'launch','joystick.launch.py'
    #           )]), launch_arguments={'use_sim_time': 'true'}.items()
    #)

    #ekf_config = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    #robot_localization_node = Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[ekf_config, {'use_sim_time': LaunchConfiguration("use_sim_time")}],
    #    remappings=[('odometry/filtered', 'odom')],
    #)


    # Пути к файлам запуска

    # rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'description.rviz')
    # start_rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen',
    #     parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
    # )

    #twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    #twist_mux = Node(
    #        package="twist_mux",
    #        executable="twist_mux",
    #        parameters=[twist_mux_params, {'use_sim_time': LaunchConfiguration("use_sim_time")}],
    #        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #)
    
    # Запуск Gazebo
    gazebo_world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'playground.world')
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
        'world': gazebo_world_file,
        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
        }.items()
    )
    # Спавн робота в Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot1'],
        output='screen'
    )

    # Запуск Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory(package_name),
                "config", "my_controllers.yaml"
            ),
           {"use_sim_time": LaunchConfiguration("use_sim_time")}  # Использование симуляционного времени
        ],
        output="screen",
    )

    # Загрузка и запуск контроллера для публикации состояний суставов
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Загрузка и запуск контроллера дифференциального привода
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"]
    )

    geom_pos_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["geom_position_controller"]
    )

    odometry_fus_config = os.path.join(get_package_share_directory(package_name), 'config', 'odometry_fus.yaml')
    odometry_fus_node = Node(
            package='odometry_fus',
            executable='odometry_fus_node',
            name='odometry_fus_node',        
            output='screen',
            parameters=[odometry_fus_config],
    )
    #TODO add to root launch
    # start_translate = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     PathJoinSubstitution([FindPackageShare("t21_navigation"), "launch", "translate.launch.py"])
    # ]),
    # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )

    # start_slam = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     PathJoinSubstitution([FindPackageShare("t21_navigation"), "launch", "slam.launch.py"])
    # ]),
    # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )
    # start_nav = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     PathJoinSubstitution([FindPackageShare("t21_navigation"), "launch", "navigation.launch.py"])
    # ]),
    # launch_arguments={'sim': LaunchConfiguration('sim')}.items()
    # )
    # start_rtabmap = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     PathJoinSubstitution([FindPackageShare("t21_rtabmap"), "launch", "t21_sim_vslam.launch.py"])
    # ]),
    # launch_arguments={'sim': LaunchConfiguration('sim')}.items()
    # )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        rsp,
        gazebo,
        control_node,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        geom_pos_spawner,
        #odometry_fus_node,
        #robot_localization_node,
        #start_rviz_cmd,
        #joystick,
        #twist_mux,
        # start_translate,
        # start_slam,
        # start_nav,
        # start_rtabmap,
    ])