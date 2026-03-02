"""Launch file for the ToF simulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('noise_stddev_mm', default_value='20.0'),
        DeclareLaunchArgument('room_size_x', default_value='5.0'),
        DeclareLaunchArgument('room_size_y', default_value='5.0'),
        DeclareLaunchArgument('room_size_z', default_value='3.0'),

        Node(
            package='drone_swarm_tof_simulator',
            executable='tof_simulator_node',
            name='tof_simulator',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'noise_stddev_mm': LaunchConfiguration('noise_stddev_mm'),
                'room_size_x': LaunchConfiguration('room_size_x'),
                'room_size_y': LaunchConfiguration('room_size_y'),
                'room_size_z': LaunchConfiguration('room_size_z'),
            }],
            output='screen',
        ),
    ])
