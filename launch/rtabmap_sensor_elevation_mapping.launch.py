from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def read_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        params = data['/**']['ros__parameters']
    return params


def generate_launch_description():
    # read configuration files
    config_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'rtabmap_sensor_elevation_mapping.param.yaml')
    
    post_processing_file = os.path.join(
        get_package_share_directory('elevation_mapping_ros2'), 
        'config', 
        'post_processing.param.yaml'
    )

    visualization_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"),
        'config',
        'visualization.yaml' 
    )
    
    params = read_yaml(config_file)
    params_post_processing = read_yaml(post_processing_file)

    return LaunchDescription([
        # Launch arguments
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
            default_value='true',
            choices=['true','false'],
            description='Start rtabmapviz node'
        ),

        # RealSense D435i camera node
        Node(
            package='realsense2_camera',
            namespace='camera',
            name='camera',
            executable='realsense2_camera_node',
            parameters=[{
                'camera_name': 'camera',
                'device_type': 'd4',
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': True,
                'enable_infra2': False,
                'enable_gyro': True,
                'enable_accel': True,
                'enable_pose': False,
                'pointcloud.enable': True,
                'pointcloud.stream_filter': 2,
                'pointcloud.stream_index_filter': 0,
                'enable_sync': True,
                'align_depth.enable': True,
                'depth_module.depth_profile': '640,480,30',
                'depth_module.infra_profile': '640,480,30',
                'rgb_camera.color_profile': '640,480,30',
                'publish_tf': True,
                'tf_publish_rate': 50.0,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
            emulate_tty=True,
        ),

        # static TF publisher: map -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_link_tf',
            output='screen',
            arguments=['0.0', '0.0', '0.1', '0', '0', '0', 'map', 'camera_link']
        ),

        # ICP odometry node (from RTAB-Map)
        Node(
            package='rtabmap_odom', 
            executable='icp_odometry', 
            output='screen',
            parameters=[{
                'frame_id':'camera_link',
                'odom_frame_id':'odom',
                'wait_for_transform':0.3,
                'expected_update_rate':15.0,
                'deskewing':LaunchConfiguration('deskewing'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', '/camera/camera/depth/color/points')
            ],
            arguments=[
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/VoxelSize', '0.1',
                'Icp/Epsilon', '0.001',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/MaxTranslation', '2',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.1',
                'OdomF2M/ScanMaxSize', '15000',
                'OdomF2M/BundleAdjustment', 'false',
                '--ros-args',
                '--log-level', 'INFO',
            ]
        ),
            
        # point cloud assembler (from RTAB-Map)
        Node(
            package='rtabmap_util', 
            executable='point_cloud_assembler', 
            output='screen',
            parameters=[{
                'max_clouds':10,
                'fixed_frame_id':'',
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('cloud', 'odom_filtered_input_scan')
            ]
        ),

        # ElevationMapping node
        Node(
            package='elevation_mapping_ros2', 
            executable='elevation_mapping_ros2_composition', 
            name='elevation_mapping_ros2_composition', 
            parameters=[params, params_post_processing], 
            remappings=[
                # use ICP odometry pose output
                ("elevation_mapping/input/pose", "/odom"),
                # use assembled point cloud
                ("elevation_mapping/input/point_cloud", "assembled_cloud"),
                ("elevation_mapping/output/raw_map", "raw_elevation_map"), 
                ("post_processing/output/grid_map", "filtered_map"),
                ("post_processing/input/grid_map", "raw_elevation_map"),
            ],
            arguments=['--ros-args', '--log-level', 'INFO'], 
            output='screen'
        ),
        
        # elevation map visualization
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='grid_map_visualization',
            output='screen',
            parameters=[visualization_file]
        ),

        # RTAB-Map visualization (optional, for debugging)
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=[{
                'frame_id':'camera_link',
                'odom_frame_id':'odom',
                'subscribe_odom_info':True,
                'subscribe_scan_cloud':True,
                'subscribe_rgb': True,
                'subscribe_depth': True,
                'approx_sync':True,
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan'),
                ('rgb/image', '/camera/camera/infra1/image_rect_raw'),
                ('rgb/camera_info', '/camera/camera/infra1/camera_info'),
                ('depth/image', '/camera/camera/depth/image_rect_raw')
            ],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        ),
    ]) 