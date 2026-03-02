# Getting Started — macOS Ground Station

This guide covers running the drone-swarm-slam ground station on macOS (Apple
Silicon) using Docker.

## Prerequisites

- Docker Desktop for Mac (arm64) — <https://docs.docker.com/desktop/install/mac-install/>
- Foxglove Studio (optional, for visualization) — <https://foxglove.dev/studio>

## Quick Start

### 1. Build the runtime image

```bash
docker compose -f docker/docker-compose.runtime.yml build
```

This downloads the ROS 2 Humble base image and installs dependencies
(~2 GB on first run, cached after that).

### 2. Launch the full stack

```bash
docker compose -f docker/docker-compose.runtime.yml up
```

This starts:

| Service | What it does |
|---------|-------------|
| `micro-ros-agent` | UDP bridge — receives topics from drones over WiFi |
| `ground-station` | All ROS 2 nodes + Foxglove bridge |

The first start builds the ROS workspace from source (~1–3 min). Subsequent
starts reuse cached build artifacts and are much faster.

### 3. Visualize

Open [Foxglove Studio](https://foxglove.dev/studio) and connect to:

```
ws://localhost:8765
```

Useful topics to add panels for:

| Topic | Type | Panel |
|-------|------|-------|
| `/slam/map` | `nav_msgs/OccupancyGrid` | Map |
| `/slam/map_3d` | `sensor_msgs/PointCloud2` | 3D |
| `/drone_1/pose/estimated` | `geometry_msgs/PoseStamped` | Pose |

### 4. Launch ground station only (no micro-ROS agent)

Useful when testing with the ToF simulator instead of real hardware:

```bash
docker compose -f docker/docker-compose.runtime.yml up ground-station
```

## Network Setup

### WiFi mesh and drone IP scheme

```
MacBook Pro (ground station)
  ├── Docker: micro-ros-agent listening on UDP 8888
  └── WiFi 5 network (same SSID as drones)

Drone ESP32S3 (micro-ROS client)
  └── WiFi → UDP → <mac-ip>:8888
```

To find your Mac's WiFi IP:

```bash
ipconfig getifaddr en0
```

Configure each drone with that IP as the micro-ROS agent address
(see `firmware/main/Kconfig` → `MICRO_ROS_AGENT_IP`).

### UDP ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 8888 | UDP | micro-ROS agent (drones → ground) |
| 8765 | TCP | Foxglove WebSocket (browser → ground) |

## Development Workflow (Edit on Mac, Run in Container)

Source files are volume-mounted read-only into the container. After editing C++
source on the Mac, rebuild inside the container:

```bash
docker compose -f docker/docker-compose.runtime.yml exec ground-station bash -c \
    "source /opt/ros/humble/setup.bash && \
     colcon build --symlink-install --packages-select <package_name> && \
     source install/setup.bash"
```

Then restart the nodes:

```bash
docker compose -f docker/docker-compose.runtime.yml restart ground-station
```

> **Tip:** Python launch files and parameter files take effect immediately
> without a colcon rebuild (symlink-install links directly to source).

## Testing with the ToF Simulator

Run a simulated drone without hardware to exercise the full pipeline:

```bash
# Terminal 1 — ground station (builds and launches all nodes)
docker compose -f docker/docker-compose.runtime.yml up ground-station

# Terminal 2 — ToF simulator (inside the same container)
docker compose -f docker/docker-compose.runtime.yml exec ground-station bash -c \
    "source /opt/ros/humble/setup.bash && \
     source install/setup.bash && \
     ros2 launch drone_swarm_tof_simulator tof_simulator.launch.py drone_id:=1"
```

The simulator publishes synthetic point clouds to `/drone_1/tof/pointcloud`,
which the PointcloudAssembler and SLAM node consume.

## CI vs Runtime Images

| | `docker-compose.yml` | `docker-compose.runtime.yml` |
|-|----------------------|------------------------------|
| Purpose | CI build + test | Runtime on macOS |
| Entrypoint | `colcon build && colcon test` | `colcon build && ros2 launch` |
| Source mount | Read-only (verifies checkout) | Read-only (live dev) |
| Persisted artifacts | No | Yes (named volumes) |
| Foxglove bridge | No | Yes (port 8765) |
| micro-ROS agent | No | Yes (port 8888/udp) |

## Stopping

```bash
docker compose -f docker/docker-compose.runtime.yml down
```

To also remove build caches (forces a full rebuild next time):

```bash
docker compose -f docker/docker-compose.runtime.yml down -v
```
