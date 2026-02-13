import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    localization = LaunchConfiguration('localization')

    rtabmap_package='t21_rtabmap'
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # With the simulator, the imu is not published fast enough 
    # and have a huge delay, disabling imu usage from VO
    #use_imu = use_sim_time.perform(context) in ["false", "False"]
    
    # Due to enormous amount of rtabmap nodes params they all was being extraced in files
    # Some params are displayed there for fast tunning
    common_params = {
        # Fixed frame id of the robot (base frame), you may set "base_link" or "base_footprint" if they are published. For camera-only config, this could be "camera_link".
        'frame_id': 'base_link',  # default_value='base_link'
        # If set, TF is used to get odometry instead of the topic.
        'odom_frame_id': 'odom',  # default_value=''
        # Set to true if using simulation
        'use_sim_time': False,
        # false=exact synchronization.
        'approx_sync': True,
        # 0 means infinite interval duration (used with approx_sync=true)
        'approx_sync_max_interval': 0.1,  # default_value='0.0'(sec)
        #TODO
        'wait_for_transform': 0.5, #TODO
        # Backward compatibility, use "args" instead.
        'sync_queue_size': 20,  # default_value='10'
        # Queue size of individual topic subscribers.
        'topic_queue_size': 20,  # default_value='10'
        # General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.
        'qos': 0,  # default_value='0'
        # QoS for camera info topics
        'qos_camera_info': 1,
        # QoS overrides for specific topics
        'qos_overrides./clock.subscription.depth': 1,
        'qos_overrides./clock.subscription.durability': 'volatile',
        'qos_overrides./clock.subscription.history': 'keep_last',
        'qos_overrides./clock.subscription.reliability': 'best_effort',
        'qos_overrides./parameter_events.publisher.depth': 1000,
        'qos_overrides./parameter_events.publisher.durability': 'volatile',
        'qos_overrides./parameter_events.publisher.history': 'keep_last',
        'qos_overrides./parameter_events.publisher.reliability': 'reliable',
        # Subscription settings
        'subscribe_depth': False,  # default_value='true'
        # Subscription settings
        'subscribe_odom': True,  # default_value='true'
        # Subscription settings
        'subscribe_odom_info': True,  # default_value='true'
        # Subscription settings
        'subscribe_rgb': False,  # default_value='false'
        # Already synchronized RGB-D related topic, e.g., with rtabmap_sync/rgbd_sync nodelet.
        'subscribe_rgbd': True,  # default_value=LaunchConfiguration('rgbd_sync')
        # Recieve Lidar data
        'subscribe_scan': False,  # default_value='false'
        # I guess recieve LIDAR msg/pointcloud
        'subscribe_scan_cloud': False,  # default_value='false'
        # Subscription settings
        'subscribe_sensor_data': False,  # default_value='false'
        # Subscription settings
        'subscribe_stereo': False,  # default_value='false'
        # User data synchronized subscription.
        'subscribe_user_data': False,  # default_value='false'
    }
    
    vslam_params = os.path.join(
        get_package_share_directory(rtabmap_package),
        'config',
        'real_rtabmap.yaml' 
    ) 
    
    rgbd_odom_params = os.path.join(
        get_package_share_directory(rtabmap_package),
        'config',
        'rgbd_odom_params.yaml'
    )

    sync_params = os.path.join(
        get_package_share_directory(rtabmap_package),
        'config',
        'rgbd_sync_params.yaml'
    )

    viz_params = os.path.join(
        get_package_share_directory(rtabmap_package),
        'config',
        'rtabmap_viz.yaml'
    ) 

    pcl_params = os.path.join(
        get_package_share_directory(rtabmap_package),
        'config',
        'pcl.yaml'
    ) 

    obstacles_params = os.path.join(
        get_package_share_directory(rtabmap_package),
        'config',
        'obstacles_champ.yaml'
    )
    
    icp_odom_params = {
    'Reg/Strategy': '1',
    'Reg/Force3DoF': 'true',
    'Mem/NotLinkedNodesKept': 'false',
    'Icp/VoxelSize': '0.1',
    'Icp/MaxCorrespondenceDistance': '1',
    'Icp/PointToPlaneGroundNormalsUp': '0.9',
    'Icp/RangeMin': '0.5',
    'Icp/MaxTranslation': '1',
    'scan_cloud_max_points': 2500,  # Changed from '2500' (string) to 2500 (integer)
    'approx_sync': False,
    'approx_sync_max_interval': 0.2,
    'queue_size': 10,
    'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # Remappings - change to your projects topics names
    vslam_remappings=[('imu', '/imu/data'),
                      ('odom', 'odom'),]
                      #('scan_cloud', '/velodyne_points'),]
  
    rgbd_remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]
    
    lidar_remappings=[('scan_cloud', '/velodyne_points'),]
    
    return [        
        # compute imu orientation
        # Node(
        #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        #     parameters=[{
        #       'use_mag':False,
        #       'world_frame':'map',
        #       'publish_tf':True,
        #       'use_sim_time': LaunchConfiguration('use_sim_time'),}],
        #     remappings=[
        #         ('imu/data_raw', 'imu/data_raw'),
        #         ('imu/data', 'imu/data')]
        #     ),
        
        # VSLAM nodes:
        Node(
            package='rtabmap_sync', executable='rgbd_sync', 
            parameters=[sync_params,common_params,
                {
                'approx_sync': True,
                'topic_queue_size': 10,
                'qos': 1,  # Reliable QoS
                'qos_image': 1,
                'qos_info': 1,
                'depth_scale': 1.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            remappings=rgbd_remappings
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=[rgbd_odom_params, common_params],
            remappings=vslam_remappings,
            arguments=["--ros-args", "--log-level", 'warn']),

        # Node(
        #     package='rtabmap_odom', executable='icp_odometry', output='screen',
        #     parameters=[common_params, icp_odom_params],
        #     remappings=[('/scan_cloud',   '/velodyne_points')],
        #     arguments=["--ros-args", "--log-level", 'warn']
        # ),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[vslam_params, common_params],
            remappings=vslam_remappings,
            arguments=['-d', "--ros-args", "--log-level", 'warn']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[vslam_params, common_params,
              {'Mem/IncrementalMemory': 'False',
               'Mem/InitWMWithAllNodes': 'True',
               'use_sim_time': use_sim_time}],
            remappings=vslam_remappings,
        ),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[common_params,viz_params],
            remappings=vslam_remappings,
            arguments=["--ros-args", "--log-level", 'warn']
        ),

        # Debug
        Node(
            package='rqt_topic', executable='rqt_topic', name='topic_monitor',
            condition=IfCondition(LaunchConfiguration("debug"))
        ),
        Node(
            package='rqt_graph', executable='rqt_graph', name='graph_monitor',
            condition=IfCondition(LaunchConfiguration("debug"))
        ),
    #     # Compute ground/obstacle clouds for nav2 voxel layers
    #     Node(
    #         package='rtabmap_util', executable='point_cloud_xyz', output='screen',
    #         parameters=[{'decimation': 2,
    #                      'max_depth': 3.0,
    #                      'voxel_size': 0.02}],
    #            remappings=[('depth/image', '/camera/depth/image_rect_raw'),
    #             ('depth/camera_info', '/camera/depth/camera_info'),
    #             ('cloud', '/camera/depth/points')]
    #     ),
        
    #     Node(
    #         package='rtabmap_util', executable='obstacles_detection', output='screen',
    #         parameters=[{
    #             'frame_id': 'base_link',
    #             'map_frame_id': 'map',
    #             'min_cluster_size': 20,
    #             'max_obstacle_height': 2.0,
    #             
    #         }],
    #         remappings=[
    #             ('cloud', '/camera/points'),
    #             ('obstacles', '/camera/obstacles'),
    #             ('ground', '/camera/ground')
    #         ]
    #     ),
     ]        
def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),
        
        DeclareLaunchArgument(
            name='rtabmap_viz', 
            default_value='true',
            description='Run rtabmap_viz'
        ),

        DeclareLaunchArgument(
            'localization', default_value='false', choices=['true', 'false'],
            description='Launch rtabmap in localization mode (a map should have been already created).'),
        
        DeclareLaunchArgument(
            name="debug",
            default_value="false",
            description="Show rqt_graph and topic monitor"
        ),
        OpaqueFunction(function=launch_setup)
    ])