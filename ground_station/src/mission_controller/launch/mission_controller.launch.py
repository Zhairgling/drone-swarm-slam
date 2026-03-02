"""Launch file for the drone-swarm-slam mission controller."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1',
                              description='Drone identifier (N in /drone_N/ topics)'),
        DeclareLaunchArgument('waypoint_radius_m', default_value='0.5',
                              description='Acceptance radius in metres'),
        DeclareLaunchArgument('republish_rate_hz', default_value='1.0',
                              description='Waypoint republish rate (Hz)'),

        Node(
            package='drone_swarm_mission_controller',
            executable='mission_controller_node',
            name='mission_controller',
            parameters=[{
                'drone_id': LaunchConfiguration('drone_id'),
                'waypoint_radius_m': LaunchConfiguration('waypoint_radius_m'),
                'republish_rate_hz': LaunchConfiguration('republish_rate_hz'),
                # DESIGN: waypoints is a flat list [x0,y0,z0, x1,y1,z1, ...].
                # Override via a YAML param file for real missions, e.g.:
                #   ros2 run ... --ros-args --params-file mission.yaml
                'waypoints': [0.0, 0.0, 1.0,
                              2.0, 0.0, 1.0,
                              2.0, 2.0, 1.0,
                              0.0, 2.0, 1.0],
            }],
            output='screen',
        ),
    ])
