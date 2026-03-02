# Single VL53L8CX Time-of-Flight sensor scan (8x8 zone grid)
#
# Each VL53L8CX produces a 8x8 grid of range measurements.
# Four sensors are mounted on each drone (sensor_id 0-3).

std_msgs/Header header

# Sensor identifier (0-3, one per VL53L8CX on the drone)
uint8 sensor_id

# 8x8 grid of range measurements in millimeters (row-major order)
# Invalid zones report 0
uint16[64] ranges_mm

# Per-zone measurement status from the VL53L8CX
# 0 = valid, non-zero = error/low-confidence (vendor-defined)
uint8[64] status

# Ambient light per zone (kcps/SPAD) — useful for filtering
uint16[64] ambient

# Grid dimensions (always 8x8 for VL53L8CX, included for generality)
uint8 WIDTH=8
uint8 HEIGHT=8
