# CLAUDE.md — Drone Swarm SLAM

## Project Overview
Multi-drone 3D SLAM system. Minimal onboard processing (XIAO ESP32S3 Sense),
heavy processing on ground station (macOS, ROS 2 Humble, C++).

## Architecture
```
XIAO ESP32S3 Sense (micro-ROS / FreeRTOS)
  ├── VL53L8CX ToF sensors (×4, I2C) → sensor_msgs/PointCloud2
  ├── OV2640 camera → sensor_msgs/CompressedImage
  └── UART → H743-mini v3 (ArduCopter) via MAVLink
        ↕ WiFi 5 mesh (UDP)
macOS Ground Station (ROS 2 Humble, C++)
  ├── micro-ROS agent (UDP bridge)
  ├── SLAM node (pointcloud → map)
  ├── Pose estimator → corrections → drone
  └── Mission controller (planning, multi-drone)
```

## Hardware
- Frame: Flywoo Explorer LR 4 (4")
- FC: H743-mini v3 (ArduCopter)
- GPS: M100QMC-5883L
- Companion: Seeed XIAO ESP32S3 Sense
- ToF: 4× VL53L8CX (8×8, I2C, 15Hz)
- Camera: OV2640 (rolling shutter, onboard XIAO — NOT primary SLAM camera)
- Battery: 4S 14.8V 3000mAh Sony VTC6
- RC: ELRS
- Motors: Nin V2 1404 2750kv
- Ground: MacBook Pro M4 Max 64GB

## Code Conventions

### C++ (ground_station/ — ROS 2)
- C++17, Google Style + ROS 2 naming
- `snake_case` files/functions/variables, `PascalCase` classes
- `#pragma once`, includes: system → ROS 2 → project (alphabetical)
- No exceptions in callbacks, use RCLCPP_* logging
- `shared_ptr`/`unique_ptr` only, no raw new/delete
- Tests: GTest via `ament_cmake_gtest`, every public method tested
- Lint: `clang-tidy` + `cppcheck`

### C (firmware/ — ESP-IDF)
- C11, ESP-IDF conventions
- `module_action()` functions, `module_t` types, `CONFIG_*` Kconfig
- Always check `esp_err_t`. `ESP_ERROR_CHECK()` in init, graceful in loops
- Static allocation preferred. No malloc in ISRs.
- Tests: Unity (ESP-IDF built-in)

### ROS 2 Topics
```
/drone_N/tof/pointcloud      sensor_msgs/PointCloud2
/drone_N/camera/compressed   sensor_msgs/CompressedImage
/drone_N/pose/estimated      geometry_msgs/PoseStamped
/drone_N/mavlink/from_fc     mavros_msgs/Mavlink
/drone_N/mavlink/to_fc       mavros_msgs/Mavlink
/slam/map                    nav_msgs/OccupancyGrid
/slam/map_3d                 sensor_msgs/PointCloud2
```

### micro-ROS
- Transport: UDP over WiFi
- Node: `drone_N_onboard` (N from Kconfig)
- QoS: best_effort sensors, reliable commands
- Timers: 66ms (15Hz) ToF, 100ms (10Hz) camera

## Git
- Branches: `feat/component-desc`, `fix/component-desc`
- Commits: Conventional (`feat(firmware):`, `fix(ground):`)
- One issue = one PR. CI must pass.
- Rebase onto main before merge.

## Build & Test

Development runs on macOS (no native ROS 2 / ESP-IDF). **Always use Docker to
build and test locally before pushing.**

```bash
# Test ground station (ROS 2 Humble — build + colcon test)
docker compose -f docker/docker-compose.yml run ground-build

# Test firmware (ESP-IDF — build + unit tests)
docker compose -f docker/docker-compose.yml run firmware-build

# Run both
docker compose -f docker/docker-compose.yml run ground-build && \
docker compose -f docker/docker-compose.yml run firmware-build
```

If Docker is not available, push and let GitHub Actions CI run — but prefer
local Docker testing to catch issues early.

### Native (only if ROS 2 / ESP-IDF installed locally)
```bash
# Firmware
cd firmware && idf.py set-target esp32s3 && idf.py build
# Ground
source /opt/ros/humble/setup.bash && colcon build && colcon test
```

## Agent Rules
1. Read this file first. Follow all conventions.
2. **Run tests via Docker before pushing.** Use `docker compose -f docker/docker-compose.yml run ground-build` for ground station and `docker compose -f docker/docker-compose.yml run firmware-build` for firmware. If Docker is unavailable, push and monitor CI.
3. Keep firmware MINIMAL. 512KB SRAM + 512KB PSRAM.
4. Simplest working implementation first.
5. Non-obvious decisions → `// DESIGN:` comment.
6. Fix CI before marking PR ready.
7. One component per PR. Never mix firmware + ground.
8. Never commit secrets or WiFi passwords.