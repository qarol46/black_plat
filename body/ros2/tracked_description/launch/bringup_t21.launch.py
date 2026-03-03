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
    rviz_arg   = DeclareLaunchArgument(
        name = "rviz", 
        default_value="False",
        description="Запускать RViz (True/False)"
    )
    use_lidar_arg  = DeclareLaunchArgument(
        name = "use_lidar",
        default_value = "True",
        description = "Use VLP-16" 
    )
    use_camera_arg = DeclareLaunchArgument(
        name="use_camera",
        default_value="True",
        description="Run RGBD camera nodes and enable URDF description"
    )
    use_imu_arg = DeclareLaunchArgument(
        name = "use_imu",
        default_value = "True",
        description = "use Xsens IMU"
    )
    use_rtabmap_arg = DeclareLaunchArgument(
        name="use_rtabmap",
        default_value="False",
        description="Use rtabmap SLAM approach"
    )
    use_sim_time = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock'
    )

    start_lidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("t21_lidar"), "launch", "t21_lidar.launch.py"])
    ]),
    condition=IfCondition(PythonExpression([LaunchConfiguration("use_lidar"), ' and not ', LaunchConfiguration("use_sim_time")]))
    )
    start_camera = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("t21_camera"), "launch", "t21_camera.launch.py"])
    ]),
    condition=IfCondition(PythonExpression([LaunchConfiguration("use_camera"), ' and not ', LaunchConfiguration("use_sim_time")]))
    )
    start_rtabmap = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("t21_rtabmap"), "launch", "t21_vslam.launch.py"])
    ]),
    condition=IfCondition(
        PythonExpression([
            LaunchConfiguration("use_rtabmap"), ' and not ', LaunchConfiguration("use_sim_time")
            ])
        )
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bluespace_ai_xsens_mti_driver'), 'launch', 'xsens_mti_node.launch.py'])
        ]),
        launch_arguments={
            # Основные настройки публикации данных
            'pub_angular_velocity': 'True',      # Включить угловую скорость
            'pub_acceleration': 'True',          # Включить линейное ускорение
            'pub_mag': 'False',                  # Отключить магнитометр (если не нужен)
            'pub_free_acceleration': 'False',    # Отключить свободное ускорение
            'pub_gnss': 'False',                 # Отключить GNSS данные
            'pub_twist': 'False',                # Отключить twist
            'pub_positionLLA': 'False',          # Отключить координаты LLA
            'pub_transform': 'False',            # Отключить трансформы
            'pub_dq': 'False',                   # Отключить delta quaternion
            'pub_dv': 'False',                   # Отключить delta velocity
        
            # Другие настройки (опционально)
            'frame_id': 'imu_link',              # Задать frame_id
            'scan_for_devices': 'True',          # Автопоиск устройства
            'device_id': '',                     # Пустой ID - использовать любое устройство

            # Настройки tf2 положения IMU в СК родительского звена
            'parent_id': 'big_box_link', 
            'x_translation': '0.0',
            'y_translation': '0.08',
            'z_translation': '0.01',
        }.items(),
        condition = IfCondition(
            PythonExpression([
                LaunchConfiguration("use_imu"), ' and not ', LaunchConfiguration("use_sim_time")
            ])
        )
    )
    
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([PKG, "urdf", "t21.urdf.xacro"]), " ",
            "prefix:=", LaunchConfiguration("prefix"), " ",
            "use_sim_time:=", LaunchConfiguration("use_sim_time")
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
                condition = UnlessCondition(LaunchConfiguration("use_sim_time"))
            ),

            # запускаем ros2_control
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description],
                output="screen",
                condition = UnlessCondition(LaunchConfiguration("use_sim_time"))
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
                condition = UnlessCondition(LaunchConfiguration("use_sim_time"))
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
                condition = UnlessCondition(LaunchConfiguration("use_sim_time"))
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
                condition = UnlessCondition(LaunchConfiguration("use_sim_time"))
            ),

            # RViz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", PathJoinSubstitution([PKG, "config", "t21.rviz"])],
                parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen"
            ),
    ]
    simulate_robot = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("t21_sim"), "launch", "launch_sim.launch.py"])
    ]),
    condition = IfCondition(LaunchConfiguration("use_sim_time"))
    )


    return LaunchDescription([use_sim_time,
                              simulate_robot,
                              prefix_arg, 
                              rviz_arg, 
                              use_lidar_arg,
                              use_camera_arg,
                              use_imu_arg,
                              use_rtabmap_arg,
                              start_lidar,
                              start_camera,
                              imu_launch,
                              start_rtabmap,
                              ] 
                              + nodes)
