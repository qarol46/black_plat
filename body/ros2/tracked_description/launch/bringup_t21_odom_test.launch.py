from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
import os

PKG = FindPackageShare("tracked_description")

def generate_launch_description() -> LaunchDescription:
    prefix_arg = DeclareLaunchArgument("prefix", default_value="")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Запускать RViz (true/false)"
    )

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([PKG, "urdf", "t21.urdf.xacro"]), " ",
            "prefix:=", LaunchConfiguration("prefix")
        ])
    }

    yaml_file = PathJoinSubstitution([PKG, "config", "controllers.yaml"])
    cm_ns = "/controller_manager"

    # Добавление Xsens launch
    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bluespace_ai_xsens_mti_driver'),
                'launch',
                'xsens_mti_node.launch.py'
            ])
        ])
    )

    # Добавление odometry fusion node
    odometry_fus_config = PathJoinSubstitution([
        PKG, 'config', 'odometry_fus.yaml'
    ])
    odometry_fus_node = Node(
        package='odometry_fus',
        executable='odometry_fus_node',
        name='odometry_fus_node',
        output='screen',
        parameters=[odometry_fus_config],
    )

    nodes = [
        # публикуем описания робота
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),

        # запускаем ros2_control
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description],
            output="screen"
        ),

        # бродкастер состояний
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", cm_ns,
                "--controller-type", "joint_state_broadcaster/JointStateBroadcaster",
                "--param-file", yaml_file
            ],
            output="screen"
        ),

        # контроллер дифференциального привода
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_drive_controller",
                "--controller-manager", cm_ns,
                "--controller-type", "diff_drive_controller/DiffDriveController",
                "--param-file", yaml_file
            ],
            output="screen"
        ),

        # контроллер флиппера
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "geom_position_controller",
                "--controller-manager", cm_ns,
                "--controller-type", "forward_command_controller/ForwardCommandController",
                "--param-file", yaml_file
            ],
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([PKG, "config", "t21.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen"
        ),

        # Добавленные ноды
        #xsens_launch,
        #odometry_fus_node,
    ]

    return LaunchDescription([prefix_arg, rviz_arg] + nodes)