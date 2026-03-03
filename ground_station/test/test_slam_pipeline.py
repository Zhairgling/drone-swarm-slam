"""End-to-end SLAM pipeline integration test.

Launches the full ground-station pipeline and validates that simulated ToF
data flows through to a non-empty occupancy map.

Nodes under test (subprocesses, real compiled binaries):
  - drone_swarm_tf_broadcaster : static sensor-frame TF (base_link → tof_*)
  - drone_swarm_tof_simulator  : synthetic PointCloud2 publisher
  - drone_swarm_pointcloud_assembler : merges clouds in map frame
  - drone_swarm_slam           : builds OccupancyGrid + 3-D map

Pose mock: a static TF (map → drone_1/base_link) is published in-process via
rclpy, replacing the pose_extractor which requires real MAVLink data.

Validation:
  - /slam/map   is received and has at least one occupied cell
  - /slam/map_3d is received and contains points
"""

import os
import signal
import subprocess
import time

import pytest
import rclpy
import rclpy.qos
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros

# Isolated domain ID — avoids cross-talk with other ROS 2 nodes on the host.
_DOMAIN_ID = "47"
_TIMEOUT_SEC = 30.0


# ---------------------------------------------------------------------------
# Helper: locate installed executables
# ---------------------------------------------------------------------------

def _find_executable(package: str, name: str) -> str:
    """Find an installed ROS 2 executable.

    Searches AMENT_PREFIX_PATH directly AND sibling install directories at
    the same level. The latter covers CI builds where colcon installs each
    package into its own sub-directory (e.g. install/drone_swarm_slam/) but
    the test environment's AMENT_PREFIX_PATH only lists the packages that are
    direct dependencies of the package under test — not every sibling.
    """
    ament_prefix_path = os.environ.get("AMENT_PREFIX_PATH", "")
    search_roots: set[str] = set()
    for prefix in ament_prefix_path.split(":"):
        prefix = prefix.strip()
        if not prefix:
            continue
        # Direct lookup in this prefix (works when the package is listed).
        search_roots.add(prefix)
        # Sibling lookup: go one level up and try the target package name.
        # This handles CI per-package install trees where AMENT_PREFIX_PATH
        # only contains a subset of the workspace packages.
        parent = os.path.dirname(prefix)
        if parent:
            search_roots.add(os.path.join(parent, package))

    for root in search_roots:
        candidate = os.path.join(root, "lib", package, name)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate

    pytest.fail(
        f"Could not find executable '{name}' for package '{package}'. "
        f"AMENT_PREFIX_PATH={ament_prefix_path!r}"
    )


# ---------------------------------------------------------------------------
# In-process validator node
# ---------------------------------------------------------------------------

class _PipelineValidator(Node):
    """Publishes a mock pose TF and captures SLAM output topics."""

    def __init__(self) -> None:
        super().__init__("slam_pipeline_validator")

        # DESIGN: Publish map → drone_1/base_link as a static TF so slam_node
        # and pointcloud_assembler can transform incoming clouds into map frame.
        # Zero timestamp is the convention for static transforms — TF2 serves
        # them at any requested time regardless of the header stamp.
        self._tf_bc = tf2_ros.StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = Time(sec=0, nanosec=0)
        t.header.frame_id = "map"
        t.child_frame_id = "drone_1/base_link"
        # Drone at room centre — matches tof_simulator default drone_position.
        t.transform.translation.x = 2.5
        t.transform.translation.y = 2.5
        t.transform.translation.z = 1.5
        t.transform.rotation.w = 1.0
        self._tf_bc.sendTransform(t)

        self.map_msg: OccupancyGrid | None = None
        self.map_3d_msg: PointCloud2 | None = None

        # DESIGN: TransientLocal subscriber for /slam/map so that we receive
        # the latest map even if it was published before this node started.
        # slam_node publishes both topics with reliable + transient_local QoS.
        map_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, "/slam/map", self._on_map, map_qos
        )

        # DESIGN: volatile subscriber for /slam/map_3d — this is compatible
        # with both the slam_node publisher (transient_local, also serves
        # volatile subscribers) and the assembler publisher (volatile).
        map_3d_qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.create_subscription(
            PointCloud2, "/slam/map_3d", self._on_map_3d, map_3d_qos
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_map_3d(self, msg: PointCloud2) -> None:
        self.map_3d_msg = msg


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def rclpy_context():
    """Initialise rclpy once per test module with the isolated domain ID."""
    os.environ["ROS_DOMAIN_ID"] = _DOMAIN_ID
    rclpy.init()
    yield
    rclpy.shutdown()
    os.environ.pop("ROS_DOMAIN_ID", None)


@pytest.fixture()
def pipeline_processes():
    """Start all pipeline nodes as subprocesses; terminate them after the test."""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = _DOMAIN_ID

    procs: list[subprocess.Popen] = []

    def _start(cmd: list[str]) -> subprocess.Popen:
        p = subprocess.Popen(
            cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        procs.append(p)
        return p

    # 1. TF broadcaster — publishes drone_1/base_link → drone_1/tof_* frames.
    _start([
        _find_executable(
            "drone_swarm_tf_broadcaster", "drone_tf_broadcaster_node"
        ),
        "--ros-args", "-p", "drone_id:=1",
    ])

    # 2. ToF simulator — publishes /drone_1/tof/pointcloud at 15 Hz.
    #    noise_stddev_mm=0 makes measurements deterministic (faster convergence).
    _start([
        _find_executable("drone_swarm_tof_simulator", "tof_simulator_node"),
        "--ros-args", "-p", "drone_id:=1", "-p", "noise_stddev_mm:=0.0",
    ])

    # 3. Pointcloud assembler — merges clouds in map frame → /slam/map_3d.
    _start([
        _find_executable(
            "drone_swarm_pointcloud_assembler", "pointcloud_assembler_node"
        ),
        "--ros-args",
        "-p", "drone_ids:=[1]",
        "-p", "output_frame:=map",
        "-p", "publish_rate_hz:=10.0",
    ])

    # 4. SLAM node — accumulates pointclouds into OccupancyGrid + 3-D map.
    #    publish_rate_hz=2 ensures we receive a map quickly in the test.
    _start([
        _find_executable("drone_swarm_slam", "slam_node"),
        "--ros-args",
        "-p", "num_drones:=1",
        "-p", "map_frame:=map",
        "-p", "publish_rate_hz:=2.0",
    ])

    yield

    for p in procs:
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------

def test_slam_pipeline_produces_valid_map(
    rclpy_context,  # noqa: ARG001
    pipeline_processes,  # noqa: ARG001
) -> None:
    """Full pipeline produces a non-empty occupancy map from simulated ToF data."""
    validator = _PipelineValidator()
    deadline = time.monotonic() + _TIMEOUT_SEC
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(validator, timeout_sec=0.5)
            if validator.map_msg is not None and validator.map_3d_msg is not None:
                break
    finally:
        validator.destroy_node()

    assert validator.map_msg is not None, (
        f"/slam/map not received within {_TIMEOUT_SEC}s — "
        "slam_node may have crashed or TF was unavailable"
    )
    assert validator.map_3d_msg is not None, (
        f"/slam/map_3d not received within {_TIMEOUT_SEC}s — "
        "check assembler or slam_node logs"
    )

    # OccupancyGrid must have cells with at least one occupied entry.
    data = list(validator.map_msg.data)
    assert len(data) > 0, "OccupancyGrid.data is empty"
    assert any(v > 0 for v in data), (
        "OccupancyGrid has no occupied cells — SLAM produced an all-unknown map"
    )

    # 3-D map must contain at least one point.
    assert validator.map_3d_msg.width > 0, (
        "/slam/map_3d PointCloud2 contains no points"
    )
