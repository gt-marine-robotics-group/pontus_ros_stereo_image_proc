import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='left_namespace', default_value='left', description='The namespace name for the left camera'
        ),
        DeclareLaunchArgument(
            name='right_namespace', default_value='right', description='The namespace name for the right camera'
        ),
        DeclareLaunchArgument(
            name='num_disparities', default_value='32', description=''
        ),
        DeclareLaunchArgument(
            name='window_size', default_value='7', description='a'
        ),
        DeclareLaunchArgument(
            name='prefilter_cap', default_value='31', description='a'
        ),
        DeclareLaunchArgument(
            name='texture_threshold', default_value='10', description='a'
        ),
        DeclareLaunchArgument(
            name='uniqueness_ratio', default_value='5', description='a'
        ),
        DeclareLaunchArgument(
            name='speckle_window_size', default_value='200', description='a'
        ),
        DeclareLaunchArgument(
            name='speckle_range', default_value='64', description='a'
        ),
        Node(
            package='pontus_ros_stereo_image_proc',
            executable='image_rectifyer',
            name='image_rectifyer_node',
            output='screen',
            remappings=[
                ('/image_raw', [LaunchConfiguration('left_namespace'), '/image_raw']),
                ('/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
                ('/image_rect_color', [LaunchConfiguration('left_namespace'), '/image_rect_color'])]
        ),
        # Node(
        #     package='pontus_ros_stereo_image_proc',
        #     executable='image_rectifyer',
        #     name='image_rectifyer_node',
        #     output='screen',
        #     remappings=[
        #         ('/image_raw', [LaunchConfiguration('right_namespace'), '/image_raw']),
        #         ('/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
        #         ('/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),
        #         ('/image_rect_color', [LaunchConfiguration('right_namespace'), '/image_rect_color'])]
        # ),
        # Node(
        #     package='pontus_ros_stereo_image_proc',
        #     executable='pointcloud_node.py',
        #     name='pointcloud_node',
        #     output='screen',
        #     remappings=[
        #         ('/left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
        #         ('/right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
        #     ]
        # ),
        # Node(
        #     package='pontus_ros_stereo_image_proc',
        #     executable='disparity_node',
        #     name='disparity_node',
        #     output='screen',
        #     remappings=[
        #         ('/left/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
        #         ('/right/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),   
        #     ],
        #     parameters=[{
        #         'num_disparities' : LaunchConfiguration('num_disparities'),
        #         'window_size' : LaunchConfiguration('window_size'),
        #         'prefilter_cap' : LaunchConfiguration('prefilter_cap'),
        #         'texture_threshold' : LaunchConfiguration('texture_threshold'),
        #         'uniqueness_ratio' : LaunchConfiguration('uniqueness_ratio'),
        #         'speckle_window_size' : LaunchConfiguration('speckle_window_size'),
        #         'speckle_range' : LaunchConfiguration('speckle_range'),
        #     }]
        # ),
    ])
"""
num_disparities (int) :
* Higher value means less FOV but better detections further away

window_size (int) :
* Higher means more finer detail

prefilter_cap (int) :
Limits the prefiltering done before the disparity calculation. 
* Higher value leads to less filtering, lower value leads to more filtering (removing noise)

texture_threshold (int) :
defines a threshold that specfiies how mcuh texture must be present for stereo block matching
to occur. Specifically for low texture regions.
* Lower values mean that the algorithm is more likely to match low-texture regions

uniqueness_ratio (int) :
Basically represents the confidence of a disparity match
* Higher values means more confidence

speckle_window_size (int) :
Controls size of the region in which small disparities are allowed
* Higher value reduces more noise

speckle_range (int):
Maximum disparite difference within the speckle window for the dispartiy values to be 
considered the same region. Dispartieis that are more than this value are treated as noise
and removed
* Lower value removes more speckle
"""