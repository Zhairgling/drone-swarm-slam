# Hardware Wiring Guide

Pinout and wiring for the XIAO ESP32S3 Sense companion board, four VL53L8CX
ToF sensors, and the Matek H743-MINI V3 flight controller.

## Component Overview

| Component | Role | Interface |
|-----------|------|-----------|
| Seeed XIAO ESP32S3 Sense | Companion computer (micro-ROS) | I2C, UART, WiFi |
| VL53L8CX (×4) | 8×8 ToF ranging | I2C (shared bus) |
| Matek H743-MINI V3 | Flight controller (ArduCopter) | UART (MAVLink) |

## Pin Assignments

### XIAO ESP32S3 Sense

| Pin Label | GPIO | Function | Connected To |
|-----------|------|----------|-------------|
| SDA | GPIO6 | I2C data | VL53L8CX SDA (all 4) |
| SCL | GPIO7 | I2C clock | VL53L8CX SCL (all 4) |
| A0 | GPIO2 | LPn control | VL53L8CX #0 (front) LPn |
| A1 | GPIO3 | LPn control | VL53L8CX #1 (right) LPn |
| A2 | GPIO4 | LPn control | VL53L8CX #2 (back) LPn |
| A3 | GPIO5 | LPn control | VL53L8CX #3 (left) LPn |
| TX | GPIO21 | UART TX | H743 RX4 (PB8) |
| RX | GPIO20 | UART RX | H743 TX4 (PB9) |
| 3V3 | — | 3.3V out | VL53L8CX VIN (all 4) |
| GND | — | Ground | Common ground |

**Remaining GPIOs** (GPIO8, GPIO9, GPIO10) are available for future use
(e.g., shared INT line from ToF sensors via OR-wired open-drain).

### VL53L8CX Breakout (Pololu #3419 or equivalent)

Each sensor uses the same wiring pattern. Only the LPn pin differs per sensor.

| Breakout Pin | Connected To | Notes |
|-------------|-------------|-------|
| VIN | XIAO 3V3 | 3.2–5.5 V input |
| GND | Common GND | |
| SDA | XIAO GPIO6 | Shared I2C bus |
| SCL | XIAO GPIO7 | Shared I2C bus |
| LPn | XIAO GPIO 2/3/4/5 | One per sensor (see table above) |
| SPI/I2C | GND | Selects I2C mode |
| INT | (optional) | See "Interrupt Wiring" below |

Default 7-bit I2C address: **0x29** (8-bit write: 0x52). Each sensor is
assigned a unique address at boot via the LPn sequencing procedure.

### Matek H743-MINI V3

| Pad Label | MCU Pin | Function | Connected To |
|-----------|---------|----------|-------------|
| TX4 | PB9 | UART4 TX | XIAO RX (GPIO20) |
| RX4 | PB8 | UART4 RX | XIAO TX (GPIO21) |
| 5V | — | BEC 5V | XIAO 5V pin (power) |
| GND | — | Ground | Common ground |

UART4 = ArduPilot **SERIAL6**. Set `SERIAL6_PROTOCOL = 2` (MAVLink2) and
`SERIAL6_BAUD = 921` (921600 baud) in ArduCopter parameters.

## Wiring Diagram

```
                        Shared I2C Bus
                    ┌───────────────────────────────────┐
                    │  SDA                              │
    ┌───────────┐   │  SCL                              │
    │  H743     │   │         ┌─────────┐ ┌─────────┐  │
    │  MINI V3  │   │    ┌────┤ VL53L8CX│ │ VL53L8CX│  │
    │           │   │    │    │  #0 FWD │ │  #1 RGT │  │
    │  TX4 ─────────────────────────────────┐ LPn─GPIO2 │ LPn─GPIO3
    │  RX4 ──────────────────────────────┐  │   │     │ │   │     │
    │  5V  ──── 5V  │    │    └─────────┘ └─────────┘  │
    │  GND ──── GND │    │                              │
    └───────────┘   │    │    ┌─────────┐ ┌─────────┐  │
                    │    │    │ VL53L8CX│ │ VL53L8CX│  │
                    │    │    │  #2 AFT │ │  #3 LFT │  │
         ┌──────────────┐│    │ LPn─GPIO4 │ LPn─GPIO5  │
         │  XIAO        ││    └─────────┘ └─────────┘  │
         │  ESP32S3     ││                              │
         │  Sense       ││                              │
         │              ││                              │
         │  GPIO6 (SDA)─┘├──────────────────────────────┘
         │  GPIO7 (SCL)──┘
         │  GPIO21 (TX)──── → H743 RX4
         │  GPIO20 (RX)──── ← H743 TX4
         │  GPIO2 ──────── LPn sensor #0
         │  GPIO3 ──────── LPn sensor #1
         │  GPIO4 ──────── LPn sensor #2
         │  GPIO5 ──────── LPn sensor #3
         │  3V3 ────────── VIN (all sensors)
         │  5V  ←───────── H743 5V BEC
         │  GND ────────── common
         └──────────────┘
```

## Sensor Placement

Mount four VL53L8CX sensors on the frame facing outward, 90 degrees apart:

```
        Front (#0, 0x29)
            ▲
            │
Left (#3) ◄─┼─► Right (#1)
            │
            ▼
        Back (#2)
```

| Sensor | Position | I2C Address | LPn Pin |
|--------|----------|-------------|---------|
| #0 | Front | 0x29 | GPIO2 |
| #1 | Right | 0x2A | GPIO3 |
| #2 | Back | 0x2B | GPIO4 |
| #3 | Left | 0x2C | GPIO5 |

Addresses are assigned at boot. See "I2C Address Assignment" below.

## I2C Address Assignment

All VL53L8CX sensors share the same default address (0x29). At boot, the
firmware assigns unique addresses by sequencing the LPn (enable) pins:

```
1. Set all LPn LOW          → all sensors disabled
2. Set GPIO2 HIGH            → sensor #0 wakes at 0x29
3. Call vl53l8cx_set_i2c_address(0x29, 0x29)  → keep 0x29 (or reassign)
4. Set GPIO3 HIGH            → sensor #1 wakes at 0x29
5. Call vl53l8cx_set_i2c_address(0x29, 0x2A)  → reassign to 0x2A
6. Set GPIO4 HIGH            → sensor #2 wakes at 0x29
7. Call vl53l8cx_set_i2c_address(0x29, 0x2B)  → reassign to 0x2B
8. Set GPIO5 HIGH            → sensor #3 wakes at 0x29
9. Call vl53l8cx_set_i2c_address(0x29, 0x2C)  → reassign to 0x2C
```

Addresses are volatile — they reset to 0x29 on power cycle, so this sequence
runs on every boot.

## I2C Bus Configuration

- **Speed:** 1 MHz (Fast Mode Plus). The VL53L8CX supports up to 1 Mbit/s.
- **Pull-ups:** The Pololu breakout includes on-board pull-ups via its level
  shifter. No external pull-ups needed if using breakout boards.
- **Wire length:** Keep I2C wires under 8 cm. Longer runs may need 30–50 pF
  capacitors on SDA/SCL to suppress ringing.
- **Bus capacitance:** Four sensors on one bus is within I2C spec if wires
  are short. If instability occurs, reduce speed to 400 kHz (Fast Mode).

## UART Configuration (MAVLink)

| Parameter | XIAO Side | H743 Side |
|-----------|-----------|-----------|
| Baud rate | 921600 | 921600 |
| Voltage | 3.3V logic | 5V tolerant input |
| Protocol | MAVLink 2 | MAVLink 2 |
| ArduPilot serial | — | SERIAL6 (UART4) |

The H743 UART4 pins are 5V tolerant, and the XIAO outputs 3.3V logic.
Direct connection works — no level shifter needed.

### ArduCopter Parameters

```
SERIAL6_PROTOCOL = 2      # MAVLink2
SERIAL6_BAUD     = 921    # 921600
```

## Interrupt Wiring (Optional)

If data-ready interrupts are needed instead of polling:

| Sensor | INT Pin | XIAO GPIO |
|--------|---------|-----------|
| All 4 | INT | GPIO8 (shared) |

Wire all four INT outputs to GPIO8 via OR-wired open-drain (no additional
components needed — the VL53L8CX INT pin is active-low open-drain). Add a
single 10 kΩ pull-up to 3.3V on GPIO8. The firmware detects any sensor
data-ready, then polls individual sensor status registers to identify which.

## Power Budget

| Component | Voltage | Typical Current |
|-----------|---------|-----------------|
| XIAO ESP32S3 (WiFi active) | 5V (from H743 BEC) | ~300 mA |
| VL53L8CX (×4, ranging) | 3.3V (from XIAO LDO) | 4 × ~20 mA = 80 mA |
| **Total from H743 5V BEC** | | **~380 mA** |

The H743-MINI V3 5V BEC supplies 1.5A continuous — sufficient headroom.
The XIAO 3.3V LDO supplies 700 mA — sufficient for four sensors (80 mA).

## Assembly Notes

1. **Solder TX4/RX4 pads** on the H743-MINI V3 if not already broken out.
   These are surface-mount pads on the board edge.
2. **Use JST-SH connectors** or direct solder for I2C and LPn connections.
   Keep wires short and tidy to avoid interference with props.
3. **Secure the XIAO** with double-sided foam tape or a 3D-printed mount.
   Orient it so the camera (if used) has a clear field of view.
4. **Route wires** away from ESCs and motor wires to reduce electromagnetic
   interference on the I2C bus.
5. **Conformal coat** exposed solder joints if flying outdoors.
6. **Test I2C** with a scanner before final assembly — verify all four
   sensors respond at their assigned addresses.
