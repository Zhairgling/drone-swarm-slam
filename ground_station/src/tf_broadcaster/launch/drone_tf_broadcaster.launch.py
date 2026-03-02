"""Launch file for the drone TF broadcaster node."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),

        DeclareLaunchArgument('tof_front_x', default_value='0.05'),
        DeclareLaunchArgument('tof_front_y', default_value='0.0'),
        DeclareLaunchArgument('tof_front_z', default_value='0.0'),
        DeclareLaunchArgument('tof_front_yaw', default_value='0.0'),

        DeclareLaunchArgument('tof_right_x', default_value='0.0'),
        DeclareLaunchArgument('tof_right_y', default_value='-0.05'),
        DeclareLaunchArgument('tof_right_z', default_value='0.0'),
        DeclareLaunchArgument('tof_right_yaw',
                              default_value=str(-math.pi / 2.0)),

        DeclareLaunchArgument('tof_back_x', default_value='-0.05'),
        DeclareLaunchArgument('tof_back_y', default_value='0.0'),
        DeclareLaunchArgument('tof_back_z', default_value='0.0'),
        DeclareLaunchArgument('tof_back_yaw', default_value=str(math.pi)),

        DeclareLaunchArgument('tof_left_x', default_value='0.0'),
        DeclareLaunchArgument('tof_left_y', default_value='0.05'),
        DeclareLaunchArgument('tof_left_z', default_value='0.0'),
        DeclareLaunchArgument('tof_left_yaw',
                              default_value=str(math.pi / 2.0)),

        DeclareLaunchArgument('camera_x', default_value='0.0'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.0'),
        DeclareLaunchArgument('camera_yaw', default_value='0.0'),

        Node(
            package='drone_swarm_tf_broadcaster',
            executable='drone_tf_broadcaster_node',
            name='drone_tf_broadcaster',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'tof_front_x': LaunchConfiguration('tof_front_x'),
                'tof_front_y': LaunchConfiguration('tof_front_y'),
                'tof_front_z': LaunchConfiguration('tof_front_z'),
                'tof_front_yaw': LaunchConfiguration('tof_front_yaw'),
                'tof_right_x': LaunchConfiguration('tof_right_x'),
                'tof_right_y': LaunchConfiguration('tof_right_y'),
                'tof_right_z': LaunchConfiguration('tof_right_z'),
                'tof_right_yaw': LaunchConfiguration('tof_right_yaw'),
                'tof_back_x': LaunchConfiguration('tof_back_x'),
                'tof_back_y': LaunchConfiguration('tof_back_y'),
                'tof_back_z': LaunchConfiguration('tof_back_z'),
                'tof_back_yaw': LaunchConfiguration('tof_back_yaw'),
                'tof_left_x': LaunchConfiguration('tof_left_x'),
                'tof_left_y': LaunchConfiguration('tof_left_y'),
                'tof_left_z': LaunchConfiguration('tof_left_z'),
                'tof_left_yaw': LaunchConfiguration('tof_left_yaw'),
                'camera_x': LaunchConfiguration('camera_x'),
                'camera_y': LaunchConfiguration('camera_y'),
                'camera_z': LaunchConfiguration('camera_z'),
                'camera_yaw': LaunchConfiguration('camera_yaw'),
            }],
            output='screen',
        ),
    ])
