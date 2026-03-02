# ArduCopter Parameter Files

Parameter files for the Matek H743-MINI V3 running ArduCopter.

## Files

| File | Purpose |
|------|---------|
| `common.param` | Shared base configuration for all swarm drones |
| `companion_serial.param` | SERIAL6 (UART4) config for XIAO ESP32S3 companion |

> **Note:** Individual drone param files (e.g. `drone_1.param`) are **not** checked in.
> Export them from Mission Planner after your first tuning session and keep them locally.
> They contain frame-specific PID tunes and compass offsets that vary per-unit.

## Import (load params onto FC)

### Mission Planner
1. Connect FC via USB or telemetry.
2. Go to **Config → Full Parameter List**.
3. Click **Load from file** (bottom right).
4. Select the `.param` file.
5. Click **Write Params** to apply.

> Apply `common.param` first, then `companion_serial.param`.

### MAVProxy
```bash
mavproxy.py --master /dev/ttyUSB0 --baud 115200
# Inside MAVProxy shell:
param load common.param
param load companion_serial.param
param fetch   # verify all loaded correctly
```

### ArduPilot SITL (testing)
```bash
sim_vehicle.py -v ArduCopter --console
# In MAVProxy:
param load ardupilot/params/common.param
```

## Export (save params from FC)

### Mission Planner
1. Connect FC.
2. **Config → Full Parameter List → Save to file**.
3. Name it `drone_N.param` (N = drone ID).

### MAVProxy
```bash
param save drone_1.param
```

## Workflow for updating shared params

1. Edit `common.param` or `companion_serial.param` in this repo.
2. Open a PR with the change and a rationale comment.
3. After merge, load the updated file onto each drone in the swarm.
4. Re-export individual `drone_N.param` files if PID tunes were affected.
