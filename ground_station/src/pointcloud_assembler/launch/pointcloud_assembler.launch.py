from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_swarm_pointcloud_assembler',
            executable='pointcloud_assembler_node',
            name='pointcloud_assembler',
            output='screen',
            parameters=[{
                'drone_ids': [1],
                'window_duration_sec': 1.0,
                'max_clouds': 100,
                'publish_rate_hz': 10.0,
                'output_frame': 'odom',
            }],
        ),
    ])
