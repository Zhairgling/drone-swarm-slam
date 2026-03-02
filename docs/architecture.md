# Architecture

Multi-drone 3D SLAM system. Each drone carries minimal onboard sensors
(XIAO ESP32S3 Sense) and streams data over WiFi to a macOS ground station
running ROS 2 Humble, where all heavy processing (SLAM, pose estimation,
mission planning) takes place.

## System Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     Drone N (×multiple)                     │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐    │
│  │         XIAO ESP32S3 Sense (micro-ROS / FreeRTOS)  │    │
│  │                                                     │    │
│  │  ┌──────────────┐   ┌──────────────┐               │    │
│  │  │ VL53L8CX ×4  │   │   OV2640     │               │    │
│  │  │ ToF 8×8 I2C  │   │   Camera     │               │    │
│  │  │   15 Hz      │   │   10 Hz      │               │    │
│  │  └──────┬───────┘   └──────┬───────┘               │    │
│  │         │                  │                        │    │
│  │         ▼                  ▼                        │    │
│  │  PointCloud2        CompressedImage                 │    │
│  │         │                  │                        │    │
│  │         └────────┬─────────┘                        │    │
│  │                  │ micro-ROS (UDP)                  │    │
│  └──────────────────┼──────────────────────────────────┘    │
│                     │                                       │
│  ┌──────────────────┼──────────────────────────────────┐    │
│  │  H743-mini v3 (ArduCopter)  ◄── UART/MAVLink ──────┤    │
│  │  Flight controller          │                       │    │
│  │  GPS: M100QMC-5883L         │                       │    │
│  │  RC: ELRS                   │                       │    │
│  └─────────────────────────────┘                       │    │
│                                                             │
│  Frame: Flywoo Explorer LR 4" │ Motors: Nin V2 1404 2750kv │
│  Battery: 4S 14.8V 3000mAh Sony VTC6                       │
└────────────────────┬────────────────────────────────────────┘
                     │
                     │ WiFi 5 mesh (UDP)
                     │
┌────────────────────┴────────────────────────────────────────┐
│              macOS Ground Station (ROS 2 Humble, C++)       │
│              MacBook Pro M4 Max 64GB                        │
│                                                             │
│  ┌──────────────────┐                                       │
│  │  micro-ROS Agent │  UDP bridge ← per-drone connections   │
│  └────────┬─────────┘                                       │
│           │                                                 │
│           ▼                                                 │
│  ┌──────────────────┐   ┌──────────────────┐                │
│  │    SLAM Node     │   │  Pose Estimator  │                │
│  │  pointcloud→map  │──▶│  corrections→FC  │                │
│  └────────┬─────────┘   └──────────────────┘                │
│           │                                                 │
│           ▼                                                 │
│  ┌──────────────────┐                                       │
│  │Mission Controller│  planning, multi-drone coordination   │
│  └──────────────────┘                                       │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

```
Drone N onboard                              Ground Station
─────────────────                            ──────────────

VL53L8CX ×4 (I2C)                           micro-ROS Agent
     │  8×8 depth frames @ 15 Hz                 │
     ▼                                           │
micro-ROS publisher ──── UDP/WiFi ────▶ /drone_N/tof/pointcloud
                                                 │
OV2640 camera                                    │
     │  JPEG frames @ 10 Hz                      │
     ▼                                           │
micro-ROS publisher ──── UDP/WiFi ────▶ /drone_N/camera/compressed
                                                 │
                                                 ▼
                                           SLAM Node
                                                 │
                                        ┌────────┴────────┐
                                        ▼                 ▼
                                  /slam/map          /slam/map_3d
                               OccupancyGrid       PointCloud2
                                        │
                                        ▼
                                  Pose Estimator
                                        │
                           /drone_N/pose/estimated
                                 PoseStamped
                                        │
                                        ▼
                                Mission Controller
                                        │
                         ┌──────────────┴──────────────┐
                         ▼                              ▼
              /drone_N/mavlink/to_fc     (multi-drone coordination)
                  Mavlink commands
                         │
              UDP/WiFi + UART/MAVLink
                         │
                         ▼
              H743-mini v3 (ArduCopter)
                         │
                /drone_N/mavlink/from_fc
                  Mavlink telemetry
```

## ROS 2 Topic Map

| Topic | Message Type | Direction | QoS | Rate |
|-------|-------------|-----------|-----|------|
| `/drone_N/tof/pointcloud` | `sensor_msgs/PointCloud2` | Drone → Ground | Best effort | 15 Hz |
| `/drone_N/camera/compressed` | `sensor_msgs/CompressedImage` | Drone → Ground | Best effort | 10 Hz |
| `/drone_N/pose/estimated` | `geometry_msgs/PoseStamped` | Ground → Drone | Reliable | — |
| `/drone_N/mavlink/from_fc` | `mavros_msgs/Mavlink` | FC → Ground | Reliable | — |
| `/drone_N/mavlink/to_fc` | `mavros_msgs/Mavlink` | Ground → FC | Reliable | — |
| `/slam/map` | `nav_msgs/OccupancyGrid` | Ground internal | Reliable | — |
| `/slam/map_3d` | `sensor_msgs/PointCloud2` | Ground internal | Reliable | — |

`N` is the drone ID, configured per-drone via Kconfig.

## Hardware Components

### Per Drone

| Component | Part | Role |
|-----------|------|------|
| Frame | Flywoo Explorer LR 4" | Airframe |
| Flight controller | H743-mini v3 | ArduCopter autopilot |
| Companion computer | Seeed XIAO ESP32S3 Sense | Sensor hub, micro-ROS node |
| ToF sensors | 4× VL53L8CX | 8×8 depth grids, I2C, 15 Hz |
| Camera | OV2640 | Compressed image stream (not primary SLAM input) |
| GPS | M100QMC-5883L | Position + compass |
| RC receiver | ELRS | Manual override / failsafe |
| Motors | Nin V2 1404 2750kv | Propulsion |
| Battery | 4S 14.8V 3000mAh Sony VTC6 | Power |

### Ground Station

| Component | Part | Role |
|-----------|------|------|
| Computer | MacBook Pro M4 Max 64GB | SLAM, pose estimation, mission planning |

## Software Stack

### Firmware (XIAO ESP32S3 Sense)

- **Framework:** ESP-IDF + FreeRTOS
- **Middleware:** micro-ROS (UDP transport over WiFi 5)
- **Language:** C11
- **Constraints:** 512 KB SRAM + 512 KB PSRAM — minimal processing onboard
- **ROS node name:** `drone_N_onboard` (N from Kconfig)

The firmware reads ToF and camera sensors, packages them as ROS 2 messages,
and publishes over micro-ROS. It also bridges MAVLink between the flight
controller (UART) and the ground station (UDP).

### Ground Station

- **OS:** macOS
- **Middleware:** ROS 2 Humble
- **Language:** C++17
- **Components:**
  - **micro-ROS Agent** — UDP bridge between each drone's micro-ROS node and the ROS 2 graph
  - **SLAM Node** — fuses multi-drone point clouds into a unified 3D map
  - **Pose Estimator** — computes corrected poses and sends them back to drones
  - **Mission Controller** — high-level planning and multi-drone coordination

## Communication

### Drone ↔ Ground Station

- **Transport:** WiFi 5 mesh, UDP
- **Protocol:** micro-ROS (DDS-XRCE over UDP)
- **QoS:** Best effort for sensor streams (tolerate drops), reliable for commands

### Companion ↔ Flight Controller

- **Transport:** UART
- **Protocol:** MAVLink
- **Bridge:** The XIAO ESP32S3 forwards MAVLink frames between the H743 FC and
  the ground station via micro-ROS topics (`/drone_N/mavlink/from_fc`,
  `/drone_N/mavlink/to_fc`)

## Multi-Drone Scaling

Each drone runs an independent micro-ROS node (`drone_N_onboard`) with its own
namespace (`/drone_N/`). The ground station subscribes to all drone namespaces,
fuses their sensor data in the SLAM node, and publishes a single unified map.
Adding a drone requires only assigning a new ID via Kconfig and connecting it to
the WiFi mesh.
