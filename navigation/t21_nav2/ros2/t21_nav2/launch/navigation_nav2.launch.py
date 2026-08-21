import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

package_name = 't21_nav2'


def generate_launch_description():
    namespace      = LaunchConfiguration('namespace')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    autostart      = LaunchConfiguration('autostart')
    params_file    = LaunchConfiguration('params_file')
    use_respawn    = LaunchConfiguration('use_respawn')
    log_level      = LaunchConfiguration('log_level')
    env_path       = LaunchConfiguration('env_path')

    lifecycle_nodes = [
        # 'collision_monitor',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/cmd_vel', 'cmd_vel_nav'),
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart':    autostart,
    }

    # ── Пути к конфигам берутся из env_path ──────────────────────────────────
    params_file_full = PathJoinSubstitution([
        env_path, 'configs', 'navigation', 'nav2_configs', params_file
    ])

    params_file_collision = PathJoinSubstitution([
        env_path, 'configs', 'navigation', 'nav2_configs', 'collision_monitor.yaml'
    ])
    # ─────────────────────────────────────────────────────────────────────────

    configured_params = RewrittenYaml(
        source_file=params_file_full,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # ── Аргументы ─────────────────────────────────────────────────────────────
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_env_path_cmd = DeclareLaunchArgument(
        'env_path',
        default_value='/data',
        description='Root directory that contains configs/, maps/, etc.',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='navigation_mppi.yaml',
        description='Config filename inside <env_path>/configs/',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn nodes if they die',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)',
    )

    # ── Ноды ──────────────────────────────────────────────────────────────────
    load_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace=namespace),
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                emulate_tty=True,
                remappings=remappings,
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[params_file_collision, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
            Node(
                package='t21_nav2',
                executable='speed_constraint',
                name='SpeedConstraint',
                output='screen',
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_env_path_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)
    return ld