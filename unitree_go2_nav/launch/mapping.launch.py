from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# from unitree_nav_launch_module import TernaryTextSubstitution


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true','false'],
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            name='deskewing',
            default_value='false',
            choices=['true','false'],
            description='Enable lidar deskewing'
        ),

        DeclareLaunchArgument(
            name='use_rtabmapviz',
            default_value='true', # suppress incessant VTK 9.0 warnings
            choices=['true','false'],
            description='Start rtabmapviz node'
        ),
        # DeclareLaunchArgument(
        #     name='restart_map',
        #     default_value='false',
        #     choices=['true','false'],
        #     description='Delete previous map and restart'
        # ),

        DeclareLaunchArgument(
            name='use_map_rviz',
            default_value='true',
            choices=['true','false'],
            description='Open RVIZ for visualization'
        ),

        SetLaunchConfiguration(name='config_file',
                                 value='mapping.rviz'),

        SetLaunchConfiguration(name='rvizconfig',
                                 value=PathJoinSubstitution([FindPackageShare('unitree_go2_nav'),
                                                             'config',
                                                             LaunchConfiguration('config_file')])),

        # Publish a static transform between base_link and base_laser for standalone use
        # of this launch file   ---> confirm you dont actually need this.
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '-0.213', '0', '0', '0', 'base_footprint', 'base_link']
        # ),

        # # Node(rtabmap generates map, takes in odom and pointcloud/ laser go2 topics) map can be visualised in rviz also.
        # Node(
        #     package='rtabmap_odom', executable='icp_odometry', output='screen',
        #     parameters=[{
        #         'frame_id':'base_link',
        #         'odom_frame_id':'odom',
        #         'wait_for_transform':-1, # 0.2
        #         # 'wait_for_transform': -1 
        #         'expected_update_rate':15.5,  #15.0 frame rate lower than what was comming in.
        #         'deskewing':LaunchConfiguration('deskewing'),
        #         'use_sim_time':LaunchConfiguration('use_sim_time'),
        #     }],
        #     remappings=[
        #         ('scan_cloud', '/utlidar/cloud'),
        #         ('odom', '/utlidar/robot_odom') # remap odom to odom_filtered_input_scan
        #         #   --------------------------------------> name of the topic to be remapped
        #     ],
        #     arguments=[
        #         'Icp/PointToPlane', 'true',
        #         'Icp/Iterations', '10',
        #         'Icp/VoxelSize', '0.1',
        #         'Icp/Epsilon', '0.001',
        #         'Icp/PointToPlaneK', '20',
        #         'Icp/PointToPlaneRadius', '0',
        #         'Icp/MaxTranslation', '2',
        #         'Icp/MaxCorrespondenceDistance', '1',
        #         'Icp/Strategy', '1',
        #         'Icp/OutlierRatio', '0.7',
        #         'Icp/CorrespondenceRatio', '0.01',
        #         'Odom/ScanKeyFrameThr', '0.6',
        #         'OdomF2M/ScanSubtractRadius', '0.1',
        #         'OdomF2M/ScanMaxSize', '15000',
        #         'OdomF2M/BundleAdjustment', 'false',
        #         '--ros-args',
        #         # '--log-level',
        #         # [
        #         #     TextSubstitution(text='icp_odometry:='),
        #         #     LaunchConfiguration('icp_odometry_log_level'),
        #         # ],
        #     ]
        #     ),

        # Node(
        #     package='rtabmap_util', executable='point_cloud_assembler', output='screen',
        #     parameters=[{
        #         'max_clouds':10,
        #         'fixed_frame_id':'',
        #         'use_sim_time':LaunchConfiguration('use_sim_time'),
        #     }],
        #     remappings=[
        #         ('cloud', 'odom_filtered_input_scan')
        #     ]),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id':'utlidar_lidar',
                'subscribe_depth':False,
                'subscribe_rgb':False,
                'subscribe_scan_cloud':True,
                'approx_sync':True, # False
                'wait_for_transform': 0.3, #0.2,
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                # Added to resolve frequency mismatch between scan cloud and odom.
                'sync_queue_size': 50,
                'topic_queue_size': 50,
                # Added to make it publish and connect map frame 
                # 'publish_tf': True,
                # 'map_frame_id': 'map',
                # 'odom_frame_id': 'odom',
                'grid/Sensor' :0, # 1 for laser, 0 for RGBD
                'grid/RayTracing' :True, # 1 for ray tracing, 0 for ray casting
                # 'grid/MaxGroundAngle' :45, # Max ground angle in degrees
                # 'grid/MaxObstacleHeight' :2, # Max obstacle height in meters
            }],

            remappings=[
                ('scan_cloud', '/utlidar/cloud_deskewed'),
                # ('scan_cloud', '/refined_scan'),
                ('odom', '/utlidar/robot_odom')
            ]
            # ,

            # arguments=[{
            #     # Added to fix the issue of no free space in occupancy grid.
            #     'grid/Sensor' :0, # 1 for laser, 0 for RGBD
            #     'grid/RayTracing' :True, # 1 for ray tracing, 0 for ray casting
            #     # 'grid/MaxGroundAngle' :45, # Max ground angle in degrees
            #     # 'grid/MaxObstacleHeight' :2, # Max obstacle height in meters
            #     'grid/MaxGroundHeight' :.3, # Max ground height in meters
            #     'grid/NormalsSegmentation': False,
            # }]

            # ,
            # arguments=[
            #     # TernaryTextSubstitution(IfCondition(LaunchConfiguration('restart_map')), '-d', ''),
            #     # 'Mem/IncrementalMemory', TernaryTextSubstitution(IfCondition(LaunchConfiguration('localize_only')), 'false', 'true'),
            #     # 'Mem/InitWMWithAllNodes', LaunchConfiguration('localize_only'),
            #     'RGBD/ProximityMaxGraphDepth', '0',
            #     'RGBD/ProximityPathMaxNeighbors', '1',
            #     'RGBD/AngularUpdate', '0.05',
            #     'RGBD/LinearUpdate', '0.05',
            #     'RGBD/CreateOccupancyGrid', 'false',
            #     'Mem/NotLinkedNodesKept', 'false',
            #     'Mem/STMSize', '30',
            #     'Mem/LaserScanNormalK', '20',
            #     'Reg/Strategy', '1',
            #     'Icp/VoxelSize', '0.1',
            #     'Icp/PointToPlaneK', '20',
            #     'Icp/PointToPlaneRadius', '0',
            #     'Icp/PointToPlane', 'true',
            #     'Icp/Iterations', '10',
            #     'Icp/Epsilon', '0.001',
            #     'Icp/MaxTranslation', '3',
            #     'Icp/MaxCorrespondenceDistance', '1',
            #     'Icp/Strategy', '1',
            #     'Icp/OutlierRatio', '0.7',
            #     'Icp/CorrespondenceRatio', '0.2',
            #     '--ros-args',
            #     # '--log-level',
            #     # [
            #     #     TextSubstitution(text='rtabmap:='),
            #     #     LaunchConfiguration('rtabmap_log_level'),
            #     # ],
            # ]
        ), 

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[{
        #         'frame_id':'utlidar_lidar',
        #         'odom_frame_id':'odom',
        #         'subscribe_odom_info':False,
        #         'subscribe_scan_cloud':True,
        #         'approx_sync':True, # False
        #         'use_sim_time':LaunchConfiguration('use_sim_time'),
        #     }],
        #     remappings=[
        #         ('scan_cloud', '/utlidar/cloud'),
        #         ('odom', '/utlidar/robot_odom')
        #     ],
        #     condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        # ),

        Node(
            package='unitree_go2_nav', executable='odomTfPublisher', output='screen',
            remappings=[
                # ('scan_cloud', '/utlidar/cloud'),
                ('odom', '/utlidar/robot_odom')
            ],
        ),

        # Node(
        #     package='unitree_go2_nav', executable='lidarToObs', output='screen',
        # ),

        Node(package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=[
                    '-d', LaunchConfiguration('rvizconfig')
            #         '-f', LaunchConfiguration('fixed_frame')
               ],
               condition=IfCondition(LaunchConfiguration('use_map_rviz'))
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go2_description'),
                    'launch',
                    'load_go2.launch.py'
                ])
            ),
            launch_arguments=[
                # These are the possible option, currently set to the default value, can be modified for particular use cases.
                # ('use_jsp', LaunchConfiguration('use_rtabmapviz')),
                ('use_rviz', 'false'),
                # ('use_nav2_links', LaunchConfiguration('localize_only')),
                # ('fixed_frame', LaunchConfiguration('restart_map')),
                # ('namespace', LaunchConfiguration('restart_map')),
                # ('config_file', LaunchConfiguration('restart_map')),
                # ('rvizconfig', LaunchConfiguration('restart_map')),
                
            ],
        ),

    ])      

        