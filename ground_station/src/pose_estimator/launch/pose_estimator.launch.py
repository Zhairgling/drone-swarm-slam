"""Launch file for the pose estimator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('sysid', default_value='1'),
        DeclareLaunchArgument('compid', default_value='195'),

        Node(
            package='drone_swarm_pose_estimator',
            executable='pose_estimator_node',
            name='pose_estimator',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'sysid': LaunchConfiguration('sysid'),
                'compid': LaunchConfiguration('compid'),
            }],
            output='screen',
        ),
    ])
