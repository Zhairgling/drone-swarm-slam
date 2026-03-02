# Post-Assembly Verification Procedure

Step-by-step checks to confirm every subsystem works before first flight.
Complete phases in order — do not skip ahead. **Remove props before any
motor or arm test.**

**References:**
- [`docs/hardware_wiring.md`](hardware_wiring.md) — pin assignments and wiring
- [`docs/architecture.md`](architecture.md) — system data flow
- [`docs/getting_started.md`](getting_started.md) — Docker runtime setup

---

## Phase 1: Power-on Smoke Test

**Goal:** Confirm power rails are healthy and both computers boot.

### Steps

1. **Remove all propellers.**

2. **Connect the battery** (4S 14.8 V Sony VTC6) with the USB cable disconnected.

3. **Observe H743-mini v3 LEDs:**

   | Expected | Notes |
   |----------|-------|
   | Power LED (red/green) solid | Board is receiving power |
   | Status LED blinks | ArduCopter boot in progress |
   | No smoke or unusual heat | If any, disconnect immediately |

   > **Fail:** LED does not light → check battery connector polarity and XT60 solder joints.

4. **Open a serial monitor** (115200 baud) on the XIAO ESP32S3 Sense USB port
   and observe boot output:

   ```bash
   # macOS — find the port first
   ls /dev/cu.usbmodem*
   # then open monitor (replace with actual port)
   screen /dev/cu.usbmodem1101 115200
   ```

   Expected lines during boot:
   ```
   I (xxx) tof_driver: initialising 4 VL53L8CX sensors
   I (xxx) tof_driver: sensor 0 address 0x29 OK
   I (xxx) tof_driver: sensor 1 address 0x2A OK
   I (xxx) tof_driver: sensor 2 address 0x2B OK
   I (xxx) tof_driver: sensor 3 address 0x2C OK
   I (xxx) micro_ros: agent connected
   ```

   > **Fail:** `sensor N address 0x?? FAIL` → check I2C wiring for that sensor.
   See [`docs/hardware_wiring.md` — I2C Address Assignment](hardware_wiring.md#i2c-address-assignment).

5. **Measure the 5 V BEC output** on the H743-mini v3 5 V pad with a multimeter:

   | Expected | Fail threshold |
   |----------|---------------|
   | 4.9 – 5.2 V | < 4.8 V or > 5.5 V |

   > **Fail:** BEC out of range → replace ESC or check power distribution board.

---

## Phase 2: Flight Controller

**Goal:** Confirm ArduCopter is configured correctly, GPS locks, and compass/
accelerometer are calibrated.

### Steps

1. **Connect a laptop to the FC** via USB and open Mission Planner (or MAVProxy):

   ```bash
   # MAVProxy (if preferred)
   mavproxy.py --master=/dev/cu.usbmodem1101 --baudrate 115200
   ```

2. **Spot-check key parameters** (`CONFIG` → `Full Parameter List`):

   | Parameter | Expected value | Purpose |
   |-----------|---------------|---------|
   | `SERIAL6_PROTOCOL` | 2 | MAVLink 2 on UART4 |
   | `SERIAL6_BAUD` | 921 | 921600 baud to XIAO |
   | `FRAME_TYPE` | 1 | Quad X |
   | `ARMING_CHECK` | 1 | All checks enabled |

   > **Fail:** Params not loaded → connect to FC and load the param file from
   `ardupilot/params/` via Mission Planner (`CONFIG` → `Full Parameter List` →
   `Load from file`).

3. **Verify GPS lock** (outdoors or near a window):
   - HUD compass rose should show heading
   - GPS status ≥ 3D Fix, HDOP < 1.5, satellites ≥ 8

   > **Fail:** No GPS lock after 5 minutes → check antenna orientation (patch
   face up), verify `GPS_TYPE = 1` (auto-detect).

4. **Verify compass heading:**
   - Point the drone nose North.
   - Mission Planner HUD should show ~0° (± 10°).
   - Rotate 90° clockwise → heading should increase to ~90°.

   > **Fail:** Heading is inverted or 180° off → re-run compass calibration
   (`INITIAL SETUP` → `Mandatory Hardware` → `Compass`).

5. **Verify accelerometer level:**
   - Place drone on a flat surface.
   - `INITIAL SETUP` → `Mandatory Hardware` → `Accel Calibration` → check
     current reading is near 0 pitch / 0 roll.

   > **Fail:** Excessive offset → re-run accelerometer calibration.

6. **Arm/disarm test (no props):**
   - Switch transmitter to Stabilize mode.
   - Arm (throttle down-right for 5 s or via Mission Planner).
   - Motors should beep once; no spinning at idle is normal with ESC arming.
   - Disarm immediately.

   > **Fail:** Arming refused → check `ARMING_CHECK` output in MAVProxy
   (`status arming`) for the blocking condition.

---

## Phase 3: Companion Computer (XIAO ESP32S3)

**Goal:** Confirm WiFi connectivity and micro-ROS node appears on the ground
station ROS graph.

### Steps

1. **Start the ground station Docker stack** on the MacBook:

   ```bash
   docker compose -f docker/docker-compose.runtime.yml up
   ```

   See [`docs/getting_started.md` — Quick Start](getting_started.md#quick-start)
   for first-run setup.

2. **Confirm the XIAO connects to WiFi.** In the serial monitor from Phase 1:

   ```
   I (xxx) wifi: connected to SSID <your-network>
   I (xxx) wifi: IP 192.168.x.x
   ```

   > **Fail:** WiFi connection refused → verify SSID/password in
   `firmware/main/Kconfig` (`CONFIG_WIFI_SSID`, `CONFIG_WIFI_PASSWORD`) match
   the ground station network. Never commit credentials — use local sdkconfig
   override.

3. **Confirm micro-ROS agent connects.** Ground station container log should show:

   ```
   [micro_ros_agent] [info] Client connected. client_key: 0x...
   ```

   Serial monitor should show:
   ```
   I (xxx) micro_ros: agent connected
   ```

   > **Fail:** No connection after 30 s → verify the agent IP in firmware
   (`CONFIG_MICRO_ROS_AGENT_IP`) matches your Mac's WiFi IP:
   ```bash
   ipconfig getifaddr en0
   ```

4. **Verify the ROS node appears:**

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
   ```

   Expected output includes:
   ```
   /drone_1_onboard
   ```

   > **Fail:** Node not listed → check micro-ROS agent logs for errors; restart
   the agent service and power-cycle the XIAO.

---

## Phase 4: Sensors

**Goal:** Confirm all 4 ToF sensors and the camera are publishing at the
expected rates.

### Steps

1. **Check ToF publish rate:**

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic hz /drone_1/tof/pointcloud"
   ```

   Expected: `average rate: 15.00` (±1 Hz is acceptable).

   > **Fail:** Rate is 0 → verify micro-ROS connection (Phase 3); check XIAO
   serial log for `tof_driver` errors.

2. **Verify all 4 ToF sensors contribute points.** Echo one message and check
   the point count:

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic echo /drone_1/tof/pointcloud --once | grep width"
   ```

   Expected: `width: 256` (4 sensors × 8×8 = 256 points per frame).

   > **Fail:** `width: 192` or similar → one sensor is missing. Check the XIAO
   serial log for `sensor N` errors; re-check wiring for the silent sensor
   (see [`docs/hardware_wiring.md` — Pin Assignments](hardware_wiring.md#pin-assignments)).

3. **Check camera publish rate:**

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic hz /drone_1/camera/compressed"
   ```

   Expected: `average rate: 10.00` (±1 Hz).

   > **Fail:** Rate is 0 → check XIAO serial log for camera init errors.

4. **Visualize in Foxglove Studio:**
   - Open Foxglove Studio and connect to `ws://localhost:8765`.
   - Add a **3D panel** and subscribe to `/drone_1/tof/pointcloud`.
   - Add an **Image panel** and subscribe to `/drone_1/camera/compressed`.
   - Wave a hand in front of each ToF sensor — the point cloud should react.

   > **Fail:** Foxglove cannot connect → confirm port 8765 is exposed
   (`docker ps` should show `0.0.0.0:8765->8765/tcp`).

---

## Phase 5: MAVLink Bridge

**Goal:** Confirm the MAVLink bridge is passing telemetry from the FC through
the XIAO to the ground station and vice versa.

### Steps

1. **Verify telemetry is flowing from the FC:**

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic hz /drone_1/mavlink/from_fc"
   ```

   Expected: non-zero rate (heartbeat at 1 Hz minimum).

   > **Fail:** Rate is 0 → check XIAO serial log for UART/MAVLink errors;
   verify `SERIAL6_PROTOCOL = 2` and `SERIAL6_BAUD = 921` on the FC
   (see [`docs/hardware_wiring.md` — UART Configuration](hardware_wiring.md#uart-configuration-mavlink)).

2. **Send a command to the FC** (heartbeat echo test via MAVProxy on ground):

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic pub --once /drone_1/mavlink/to_fc \
              mavros_msgs/msg/Mavlink '{header: {stamp: {sec: 0}}, \
              magic: 253, len: 9, seq: 0, sysid: 255, compid: 190, \
              msgid: 0, payload64: [0, 0, 0, 0, 0, 0]}'"
   ```

   Observe the FC responds (heartbeat visible in `from_fc` topic echo).

   > **Fail:** No response → check UART wiring continuity TX↔RX cross-connect
   (H743 TX4 → XIAO RX, H743 RX4 → XIAO TX).

---

## Phase 6: Motors (Props Off!)

**Goal:** Confirm all four motors spin in the correct direction and ESCs respond
to throttle.

> **Warning:** Keep propellers off for this entire phase. Tether the drone
> to a table if possible.

### Steps

1. **Identify motor positions** per Flywoo Explorer LR 4 frame layout:

   ```
       Front
   M1 (CCW)  M2 (CW)
   M4 (CW)   M3 (CCW)
       Back
   ```

   Motor numbering follows ArduCopter Quad-X convention (motor 1 = front-right).

2. **Run motor test via Mission Planner:**
   - `INITIAL SETUP` → `Optional Hardware` → `Motor Test`
   - Test motors **one at a time**, starting from Motor A (front-right, M1).
   - Increase throttle to ~10% for 2 seconds.

   > **Fail:** Motor does not spin → check ESC signal wire connection and ESC
   calibration; verify the motor output assignments in ArduCopter params.

3. **Verify spin direction** for each motor:

   | Motor | Expected direction |
   |-------|--------------------|
   | M1 (front-right) | Counter-clockwise (CCW) |
   | M2 (front-left) | Clockwise (CW) |
   | M3 (rear-right) | Clockwise (CW) |
   | M4 (rear-left) | Counter-clockwise (CCW) |

   > **Fail:** Wrong direction → swap any two of the three motor phase wires
   on that ESC. Do **not** change firmware settings.

4. **Verify ESC throttle response:**
   - Arm the FC (transmitter in Stabilize, throttle down-right for 5 s).
   - Slowly raise throttle from 0 to ~15% — motors should spin smoothly
     with no stuttering.
   - Lower throttle to 0 and disarm.

   > **Fail:** Stuttering or desync → run ESC calibration (Mission Planner
   `INITIAL SETUP` → `Optional Hardware` → `ESC Calibration`).

---

## Phase 7: Ground Station Pipeline

**Goal:** Confirm the full data pipeline from drone sensors through to the SLAM
map and Foxglove visualization.

### Steps

1. **Start the full runtime Docker stack** (if not already running):

   ```bash
   docker compose -f docker/docker-compose.runtime.yml up
   ```

   See [`docs/getting_started.md`](getting_started.md) for details.

2. **Verify the SLAM node receives pointcloud data:**

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic hz /slam/map_3d"
   ```

   Expected: non-zero publish rate (map updates as new scans arrive).

   > **Fail:** Rate is 0 → confirm `/drone_1/tof/pointcloud` is publishing
   (Phase 4 step 1); check ground-station container logs for SLAM node errors:
   ```bash
   docker compose -f docker/docker-compose.runtime.yml logs ground-station
   ```

3. **Verify pose estimator publishes corrections:**

   ```bash
   docker compose -f docker/docker-compose.runtime.yml exec ground-station \
     bash -c "source /opt/ros/humble/setup.bash && \
              ros2 topic hz /drone_1/pose/estimated"
   ```

   Expected: non-zero rate once the SLAM node has enough data to estimate pose.

   > **Fail:** No pose output after 30 s of movement → check SLAM node has
   sufficient point cloud density; wave hand in front of ToF sensors to provide
   varied scan data.

4. **Verify Foxglove shows the live map:**
   - Connect Foxglove Studio to `ws://localhost:8765`.
   - Add a **3D panel** and subscribe to `/slam/map_3d`.
   - Move the drone (or wave in front of ToF sensors) — the 3D map should
     update in near-real-time.
   - Add a **Map panel** and subscribe to `/slam/map` (2D occupancy grid).

   > **Fail:** Topics not visible in Foxglove topic list → confirm the
   Foxglove bridge service is running:
   ```bash
   docker compose -f docker/docker-compose.runtime.yml ps
   ```
   The `ground-station` service should show `Up`.

---

## Checklist Summary

| Phase | Description | Pass |
|-------|-------------|------|
| 1 | Power-on smoke test (LEDs, serial boot, 5 V BEC) | ☐ |
| 2 | Flight controller (params, GPS, compass, accel, arm) | ☐ |
| 3 | Companion computer (WiFi, micro-ROS, node list) | ☐ |
| 4 | Sensors (ToF rate, 4-sensor count, camera, Foxglove) | ☐ |
| 5 | MAVLink bridge (from_fc rate, to_fc echo) | ☐ |
| 6 | Motors — props off (spin direction, throttle response) | ☐ |
| 7 | Ground station pipeline (SLAM map, pose, Foxglove) | ☐ |

All phases must pass before installing propellers and conducting a hover test.
