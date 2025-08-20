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
        'elevation_map.param.yaml')
    
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

        # RealSense D435i camera node (fixed configuration)
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
                'enable_infra1': False,  # disable infra1 to avoid stream matching issues
                'enable_infra2': False,
                'enable_gyro': True,
                'enable_accel': True,
                'enable_pose': False,
                'pointcloud.enable': True,
                'pointcloud.stream_filter': 2,  # use depth stream as texture
                'pointcloud.stream_index_filter': 0,
                'enable_sync': True,
                'align_depth.enable': True,
                'depth_module.depth_profile': '640,480,15',  # reduce to 15Hz
                'depth_module.infra_profile': '640,480,15',
                'rgb_camera.color_profile': '640,480,15',
                'publish_tf': True,
                'tf_publish_rate': 15.0,  # match expected frame rate
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

        # static pose publisher (alternative to ICP odometry, for testing)
        Node(
            package='elevation_mapping_ros2',
            executable='pub_zero_pose',
            name='zero_pose_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('pose', '/camera/camera/pose_covariance')
            ]
        ),

        # ElevationMapping node (directly use camera point cloud)
        Node(
            package='elevation_mapping_ros2', 
            executable='elevation_mapping_ros2_composition', 
            name='elevation_mapping_ros2_composition', 
            parameters=[params, params_post_processing], 
            remappings=[
                # use static pose
                ("elevation_mapping/input/pose", "/camera/camera/pose_covariance"),
                # directly use RealSense point cloud
                ("elevation_mapping/input/point_cloud", "/camera/camera/depth/color/points"),
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

        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory("elevation_mapping_ros2"),
                'rviz2',
                'rviz2.rviz'
            )],
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),
    ]) 