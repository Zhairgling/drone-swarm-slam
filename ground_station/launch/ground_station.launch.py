"""Launch file for the drone-swarm-slam ground station.

Starts all ground station nodes:
  - PointcloudAssembler  (ToF scans → merged PointCloud2)
  - SlamNode             (PointCloud2 → OccupancyGrid / 3D map)
  - PoseEstimator        (map + drone pose → MAVLink corrections)
  - MissionController    (waypoint sequencing)
  - FoxgloveBridge       (web visualization on ws://localhost:8765)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones in the swarm'
    )
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='Drone identifier (used for single-drone per-node params)'
    )

    num_drones = LaunchConfiguration('num_drones')
    drone_id = LaunchConfiguration('drone_id')

    # DESIGN: For now, one pose_estimator and mission_controller instance covers
    # drone 1. Multi-drone support will add instances here once orchestration
    # (issue #11) stabilises.
    return LaunchDescription([
        num_drones_arg,
        drone_id_arg,

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

        Node(
            package='drone_swarm_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[{
                'num_drones': num_drones,
                'map_resolution': 0.1,
                'map_size_x': 20.0,
                'map_size_y': 20.0,
                'map_height_min': -0.5,
                'map_height_max': 3.0,
                'publish_rate_hz': 1.0,
                'map_frame': 'map',
            }],
        ),

        Node(
            package='drone_swarm_pose_estimator',
            executable='pose_estimator_node',
            name='pose_estimator',
            output='screen',
            parameters=[{
                'drone_id': drone_id,
                'sysid': 1,
                'compid': 195,
            }],
        ),

        Node(
            package='drone_swarm_mission_controller',
            executable='mission_controller_node',
            name='mission_controller',
            output='screen',
            parameters=[{
                'drone_id': drone_id,
                'waypoint_radius_m': 0.5,
                'republish_rate_hz': 1.0,
                # DESIGN: Override waypoints at runtime via a YAML params file:
                #   ros2 launch ground_station ground_station.launch.py \
                #     --ros-args --params-file mission.yaml
                'waypoints': [0.0, 0.0, 1.0],
            }],
        ),

        # Foxglove bridge: web-based visualization on ws://localhost:8765.
        # Connect with Foxglove Studio (https://foxglove.dev/studio).
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
            }],
        ),
    ])
