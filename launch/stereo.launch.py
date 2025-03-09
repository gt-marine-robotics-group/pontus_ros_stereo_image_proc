import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pontus_ros_stereo_image_proc',
            executable='image_rectifyer',
            name='image_rectifyer_node',
            output='screen',
            remappings=[
                ('/image_raw', '/pontus/camera_2/image_raw'),
                ('/camera_info', '/pontus/camera_2/camera_info'),
                ('/image_rect', '/pontus/camera_2/image_rect'),
                ('/image_rect_color', '/pontus/camera_2/image_rect_color')]
        ),
        Node(
            package='pontus_ros_stereo_image_proc',
            executable='image_rectifyer',
            name='image_rectifyer_node',
            output='screen',
            remappings=[
                ('/image_raw', '/pontus/camera_3/image_raw'),
                ('/camera_info', '/pontus/camera_3/camera_info'),
                ('/image_rect', '/pontus/camera_3/image_rect'),
                ('/image_rect_color', '/pontus/camera_3/image_rect_color')]
        ),
        Node(
            package='pontus_ros_stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            output='screen',
            remappings=[
                ('/left/image_rect', '/pontus/camera_2/image_rect'),
                ('/right/image_rect', '/pontus/camera_3/image_rect'),   
            ],
            parameters=[{
                'num_disparities' : 32,
                'window_size' : 7,
                'prefilter_cap' : 31,
                'texture_threshold' : 10,
                'uniqueness_ratio' : 5,
                'speckle_window_size' : 200,
                'speckle_range' : 64,
            }]
        ),
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