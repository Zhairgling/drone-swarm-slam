"""Launch file for the MAVLink pose extractor node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0'),

        Node(
            package='drone_swarm_mavlink_pose_extractor',
            executable='mavlink_pose_extractor_node',
            name='mavlink_pose_extractor',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
            output='screen',
        ),
    ])
