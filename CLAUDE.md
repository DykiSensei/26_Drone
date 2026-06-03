# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 quadcopter flight controller based on ESP-IDF 5.5.4. WiFi AP + WebSocket for remote control and telemetry. References Crazyflie/esp-drone architecture.

Full architecture, pinout, and design decisions are in `DESIGN.md`.

## Build & Flash

- ESP-IDF environment: `C:/Espressif/frameworks/esp-idf-v5.5.4/` (or `D:/Espressif/...` on the reference machine)
- Target: `esp32s3`
- Flash port: `COM14`

```bash
# First time on a new machine (or after clean clone):
idf.py set-target esp32s3

# Build
idf.py build

# Flash + monitor
idf.py flash monitor
```

## Pinout

| Signal | GPIO | Notes |
|--------|------|-------|
| I2C0 SDA | 9 | Shared: MPU6050 + TOF400F |
| I2C0 SCL | 8 | Shared: MPU6050 + TOF400F |
| UART1 RX | 44 | PV3901L1 optical flow (TX-only module) |
| YAW_MODE | 15 | PV3901L1 yaw mode select |
| M0 (FR, CCW) | 14 | LEDC PWM |
| M1 (FL, CW) | 11 | LEDC PWM |
| M2 (RL, CCW) | 13 | LEDC PWM |
| M3 (RR, CW) | 12 | LEDC PWM |
| RGB LED | 48 | WS2812 status indicator |

## Directory Structure

```
26_Drone/
├── main/main.c                 # Entry point — init then 100Hz main loop
├── components/
│   ├── drivers/                # Hardware drivers (I2C, MPU6050, TOF400F, PV3901L1, Motor)
│   ├── control/                # Flight control (commander, PID, mixer, altitude, flow_hold, position)
│   ├── communication/          # WiFi AP + HTTP/WebSocket server + embedded web frontend
│   ├── estimation/             # Mahony AHRS attitude filter
│   └── system/                 # Parameter storage (NVS) — planned
└── build/                      # Build artifacts (git-ignored)
```

Each component has its own `CMakeLists.txt`. The main component `REQUIRES drivers communication control estimation`.

## Architecture

**100Hz main loop** (Core 1): sensor read → Mahony AHRS → Commander setpoint → PID control → Mixer → Motor PWM → telemetry broadcast

**Control chain** (STABILIZE mode):
```
MPU6050 → Mahony → Euler angles (roll/pitch/yaw)
  Commander setpoint → target angle (±30°) → Angle P → target rate
  → Rate PID → torque → Mixer (X-quad) → 4×LEDC PWM → motors
```

**Task layout** (dual-core FreeRTOS):
- Core 0: WiFi protocol stack (ESP-IDF managed)
- Core 1: `main` (100Hz), `flow_rx` (UART interrupt-driven), HTTP server (event-driven)

## Component Dependencies

```
drivers/       → (none, leaf)
estimation/    → drivers/ (needs MPU6050 data)
control/       → drivers/ (needs motor), estimation/ (needs attitude)
communication/ → (standalone)
main/          → all components
```

## Key Design Decisions

- **Shared I2C0 bus**: `i2c_bus.c` initializes I2C0 once (`g_i2c0_bus` handle). Both MPU6050 (0x68) and TOF400F (0x29) attach via `i2c_master_bus_add_device()` — never re-init the bus.
- **Motor safety**: `MOTOR_MIN = 0.05` floor-clip (not shift-up). Throttle < 5% stops motors + resets all PIDs + resets altitude/flow controllers to prevent ground spooling.
- **TOF skip-counter**: VL53L1X data-ready checked every 10th call (@100Hz ≈ every 100ms) to match sensor timing budget; cached values returned otherwise.
- **Deferred command execution**: Blocking operations (ESC calibration, gyro recalibration) are queued via `pending_cmd` and executed in the main loop context — never from the HTTP server task. This prevents WiFi/WebSocket freeze during calibration.
- **Control modes**: DISARMED (default) / STABILIZE / ALT_HOLD / POS_HOLD. ALT_HOLD adds TOF height PID on top of STABILIZE. POS_HOLD adds optical flow velocity hold on top of ALT_HOLD. Mode transitions auto-capture target height and reset flow integrals.
- **WebSocket command flow**: browser sends JSON → `http_server` → `commander_parse` → updates global `setpoint_t`. Special commands: `{"cmd": "calibrate"}`, `{"cmd": "gyro_calib"}`, `{"cmd": "level_trim"}`, `{"cmd": "reset_trim"}`, `{"cmd": "calibrate_motor", "motor_index": 0}`, `{"cmd": "move_to", "x": 0.5, "y": -0.3}`, `{"cmd": "move_stop"}`. Velocity commands (`vel_x`/`vel_y`) are sent via regular 50Hz stick data, not special commands.

## C Headers

All component headers are public (no `private_*.h`). Include patterns:

**Drivers:**
- `i2c_bus.h` — `i2c_bus_init()`, `g_i2c0_bus` handle
- `mpu6050.h` — `mpu6050_init()`, `mpu6050_read()`, `mpu6050_calibrate()`, `mpu6050_recalibrate_gyro()`, `mpu6050_data_t`
- `tof400f.h` — `tof400f_init()`, `tof400f_get_distance()`
- `pv3901l1.h` — `pv3901l1_init()`, `pv3901l1_get_data()`, `pv3901l1_reset_integral()`, `pv3901l1_set_yaw_mode()`, `pv3901l1_data_t`
- `motor.h` — `motor_init()`, `motor_set()`, `motor_stop()`, `motor_calibrate()`, `motor_calibrate_single()`

**Estimation:**
- `attitude.h` — `attitude_init()`, `attitude_update()`, `attitude_get_euler()`

**Control:**
- `pid.h` — `pid_init()`, `pid_update()`, `pid_reset()`, `pid_t`
- `mixer.h` — `mixer_apply()`
- `commander.h` — `commander_parse()`, `commander_set_cmd_callback()`, `commander_get_setpoint()`, `commander_clear_pending_cmd()`, `commander_mode_name()`, `setpoint_t` (includes `vel_x`, `vel_y`, `move_to_x`, `move_to_y`), `flight_mode_t`, `CMD_*` enums
- `altitude.h` — `altitude_init()`, `altitude_update()`, `altitude_capture_target()`, `altitude_reset()`, `altitude_ctrl_t`
- `flow_hold.h` — `flow_hold_init()`, `flow_hold_set_velocity()`, `flow_hold_update()`, `flow_hold_reset()`, `flow_hold_is_active()`, `flow_hold_t`
- `position.h` — `position_init()`, `position_set_target()`, `position_update()`, `position_reset()`, `position_reached()`, `position_ctrl_t`

**Communication:**
- `wifi_ap.h` — `wifi_ap_init()`
- `http_server.h` — `http_server_init()`, `http_server_set_command_cb()`, `http_server_broadcast()`
- `web_page.h` — embedded HTML/CSS/JS frontend (single `const char *`)
