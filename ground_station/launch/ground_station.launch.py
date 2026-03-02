"""Launch file for the drone-swarm-slam ground station."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones in the swarm'
    )

    num_drones = LaunchConfiguration('num_drones')

    # DESIGN: Nodes will be added here as they are implemented:
    # - SLAM node (pointcloud -> map)
    # - Pose estimator (corrections -> drone)
    # - Mission controller (planning, multi-drone)

    return LaunchDescription([
        num_drones_arg,
    ])
