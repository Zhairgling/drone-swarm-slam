# ardupilot/ — ArduCopter Configuration as Code

ArduCopter parameter files and tooling for the drone swarm.

## Hardware
- **Flight controller:** Matek H743-MINI V3
- **Firmware:** ArduCopter (latest stable 4.x)
- **Frame:** Flywoo Explorer LR 4 (4" quad, X-frame)
- **Battery:** 4S 14.8V 3000mAh Sony VTC6

## Directory Structure

```
ardupilot/
├── params/
│   ├── common.param           # Shared base config for all swarm drones
│   ├── companion_serial.param # SERIAL6 (UART4) for XIAO companion
│   └── README.md              # Import/export workflow
├── scripts/
│   └── diff_params.sh         # Diff two param files for review
└── README.md                  # This file
```

> **Individual drone files** (`drone_1.param`, `drone_2.param`, …) are **not**
> stored here. Export them from Mission Planner after tuning and keep them
> locally or in a private store. They contain per-unit compass offsets and
> PID values.

## Quick Start

### 1. Flash ArduCopter firmware

Use Mission Planner (Windows/macOS) or [ArduPilot firmware site](https://firmware.ardupilot.org/):

1. Connect H743-MINI V3 via USB.
2. **Setup → Install Firmware → Quad (X)** → select latest stable.
3. Wait for flash and reboot confirmation.

### 2. Load shared parameters

```bash
# Option A: Mission Planner
# Config → Full Parameter List → Load from file
# Load common.param, then companion_serial.param

# Option B: MAVProxy
mavproxy.py --master /dev/ttyACM0 --baud 115200
param load ardupilot/params/common.param
param load ardupilot/params/companion_serial.param
param fetch
```

### 3. Calibrate (mandatory before first flight)

In Mission Planner **Setup → Mandatory Hardware**:

1. **Accelerometer calibration** — follow on-screen 6-position guide.
2. **Compass calibration** — rotate drone in all orientations until complete.
   - The M100QMC-5883L external compass will appear as Compass #2.
   - Prioritise external compass (further from ESC noise).
3. **Radio calibration** — move all sticks and switches to full travel.
4. **ESC calibration** — required once if using non-DSHOT ESCs (skip for DSHOT).

### 4. Export per-drone param file

After calibration and initial tuning:

```bash
# Mission Planner: Config → Full Parameter List → Save to file → drone_1.param
# MAVProxy:
param save drone_1.param
```

Keep this file safe. It contains your compass offsets, PID tunes, and battery
voltage divider calibration.

## Reviewing Parameter Changes

Use `diff_params.sh` to compare a drone's exported file against the shared base:

```bash
cd ardupilot
./scripts/diff_params.sh params/common.param /path/to/drone_1.param
```

This shows:
- Parameters added in the drone file (tuning overrides)
- Parameters removed (potential regressions)
- Value changes (e.g. updated PIDs or compass offsets)

## MAVLink / Companion Interface

SERIAL6 (physical UART4 pads TX4/RX4) connects to the XIAO ESP32S3 companion.
See `params/companion_serial.param` and `docs/hardware_wiring.md` for details.

Key parameters:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL6_PROTOCOL` | 2 | MAVLink2 |
| `SERIAL6_BAUD` | 921 | 921600 baud |

## Parameter File Format

ArduPilot uses a plain-text `NAME VALUE` format:

```
# Comment lines start with #
PARAM_NAME value
```

- Names are case-sensitive and must match ArduPilot exactly.
- Values are floating-point (integers work too).
- One parameter per line.

## References

- [ArduCopter Parameters](https://ardupilot.org/copter/docs/parameters.html)
- [Matek H743-MINI V3 docs](https://www.mateksys.com/?portfolio=h743-mini-v3)
- [SERIAL port setup](https://ardupilot.org/copter/docs/common-serial-options.html)
- [EKF3 tuning](https://ardupilot.org/copter/docs/common-ek3-affinity-lane-switching.html)
