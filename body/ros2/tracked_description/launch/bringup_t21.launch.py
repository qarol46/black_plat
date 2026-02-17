# bringup_t21_with_diff_drive.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition,  UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.substitutions import FindPackageShare

PKG = FindPackageShare("tracked_description")

def generate_launch_description() -> LaunchDescription:
    
    prefix_arg = DeclareLaunchArgument(
    name = "prefix", 
    default_value=""
    )
    
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([PKG, "urdf", "t21.urdf.xacro"]), " ",
            "prefix:=", LaunchConfiguration("prefix"), " ",
        ])
    }

    controller_parameters = PathJoinSubstitution([PKG, "config", "controllers.yaml"])
    cm_ns = "/controller_manager"
    nodes = [
            # публикуем описания робота
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),

            # запускаем ros2_control
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description],
                output="screen",
            ),

            # бродкастер состояний
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", cm_ns,
                    "--controller-type", "joint_state_broadcaster/JointStateBroadcaster",
                    "--param-file", controller_parameters
                ],
                output="screen",
            ),

            # контроллер дифференциального привода
            Node(package="controller_manager", executable="spawner",
                arguments=[
                    "diff_drive_controller",
                    "--controller-manager", cm_ns,
                    "--controller-type", "diff_drive_controller/DiffDriveController",
                    "--param-file", controller_parameters
                ],
                output="screen",
            ),
                

            # контроллер флиппера
            Node(package="controller_manager", executable="spawner",
                arguments=[
                    "geom_position_controller",
                    "--controller-manager", cm_ns,
                    "--controller-type", "geom_position_controller/ForwardCommandController",
                    "--param-file", controller_parameters
                ],
                output="screen",
            ),
    ]


    return LaunchDescription([
                              prefix_arg,
                              ] 
                              + nodes)
