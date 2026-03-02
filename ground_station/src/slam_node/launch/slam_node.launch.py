"""Launch file for the SLAM node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='1'),
        DeclareLaunchArgument('map_resolution', default_value='0.1'),
        DeclareLaunchArgument('map_size_x', default_value='20.0'),
        DeclareLaunchArgument('map_size_y', default_value='20.0'),
        DeclareLaunchArgument('map_height_min', default_value='-0.5'),
        DeclareLaunchArgument('map_height_max', default_value='3.0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('map_frame', default_value='map'),

        Node(
            package='drone_swarm_slam',
            executable='slam_node',
            name='slam_node',
            parameters=[{
                'num_drones': LaunchConfiguration('num_drones'),
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_size_x': LaunchConfiguration('map_size_x'),
                'map_size_y': LaunchConfiguration('map_size_y'),
                'map_height_min': LaunchConfiguration('map_height_min'),
                'map_height_max': LaunchConfiguration('map_height_max'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'map_frame': LaunchConfiguration('map_frame'),
            }],
            output='screen',
        ),
    ])
