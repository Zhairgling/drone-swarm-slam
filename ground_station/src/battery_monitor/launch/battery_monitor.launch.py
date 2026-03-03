"""Launch file for the battery monitor node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('warn_voltage', default_value='14.2'),
        DeclareLaunchArgument('critical_voltage', default_value='13.6'),

        Node(
            package='drone_swarm_battery_monitor',
            executable='battery_monitor_node',
            name='battery_monitor',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'warn_voltage': LaunchConfiguration('warn_voltage'),
                'critical_voltage': LaunchConfiguration('critical_voltage'),
            }],
            output='screen',
        ),
    ])
