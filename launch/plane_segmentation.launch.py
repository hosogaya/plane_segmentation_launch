from launch import LaunchDescription
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
    elevation_map_param_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"), 
        'config',
        'elevation_map.param.yaml')
    
    post_processing_file = os.path.join(
        get_package_share_directory('elevation_mapping_ros2'), 
        'config', 
        'post_processing.param.yaml'
    )
    elevation_ccl_file = os.path.join(
        get_package_share_directory("elevation_ccl"), 
        'config',
        'param.yaml'
    )
    # convex_plane_file = os.path.join(
    #     get_package_share_directory('convex_plane_extractor'), 
    #     'config',
    #     ''
    # )

    visualization_file = os.path.join(
        get_package_share_directory("elevation_mapping_ros2"),
        'config',
        'visualization.yaml' 
    )

    topic_name_file = os.path.join(
        get_package_share_directory("plane_segmentation_launch"), 
        'config', 
        'remapping.param.yaml')
    
    elevation_mapping_param = read_yaml(elevation_map_param_file)
    params_post_processing = read_yaml(post_processing_file)
    elevation_ccl_param = read_yaml(elevation_ccl_file)


    topic_name = read_yaml(topic_name_file)
    elevation_mapping_topic = topic_name["elevation_mapping"]
    post_processing_topic = topic_name["post_processing"]
    elevation_ccl_topic = topic_name["elevation_ccl"]
    convex_plane_topic = topic_name["convex_plane_extractor"]
    convex_plane_visualizer_topic = topic_name["convex_plane_visualizer"]
        

    plane_segmentation_composition = Node(
        package='plane_segmentation_launch', 
        executable='plane_segmentation_node', 
        name='plane_segmentation_node', 
        parameters=[elevation_mapping_param, params_post_processing, elevation_ccl_file], 
        remappings=[
            # elevation_mapping
            ("elevation_mapping/input/point_cloud", elevation_mapping_topic["input_point_cloud"]), 
            ("elevation_mapping/input/pose", elevation_mapping_topic["input_pose_covariance"]), 
            ("elevation_mapping/output/raw_map", elevation_mapping_topic["output_map"]), 
            # post processing
            ("post_processing/output/grid_map", post_processing_topic["output_map"]),
            ("post_processing/input/grid_map", post_processing_topic["input_map"]),
            # elevation_ccl
            ("elevation_ccl/input/grid_map", elevation_ccl_topic["input_map"]),
            ("elevation_ccl/output/grid_map", elevation_ccl_topic["output_map"]),
            # convex_plane_extractor
            ("convex_plane_extractor/input/grid_map", convex_plane_topic["input_map"]),
            ("convex_plane_extractor/output/grid_map", convex_plane_topic["output_map"]),
            ("convex_plane_extractor/output/planes_with_map", convex_plane_topic["output_plane"]),
            # convex_plane_visualizer
            ("convex_plane_visualizer/input/planes", convex_plane_visualizer_topic["input_plane"]), 
            ("convex_plane_visualizer/output/marker", convex_plane_visualizer_topic["output_marker"])
        ],
        arguments=['--ros-args', '--log-level', 'INFO'], 
        output = 'screen'
    )
    
    elevation_raw_map_visualization = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_file]
    )
    
    return LaunchDescription([
        plane_segmentation_composition,
        elevation_raw_map_visualization, 
    ])