# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 quadcopter flight controller based on ESP-IDF 5.5.4. WiFi AP + WebSocket for remote control and telemetry. References Crazyflie/esp-drone architecture.

Full architecture, pinout, and design decisions are in `DESIGN.md`.

## Build & Flash

- ESP-IDF environment: `D:/Espressif/frameworks/esp-idf-v5.5.4/`
- Target: `esp32s3`
- Flash port: `COM14`

```bash
# Build
idf.py build

# Flash (build + flash + monitor)
idf.py flash monitor
```

## Directory Structure

```
26_Drone/
├── main/main.c                 # Entry point — init then 100Hz main loop
├── components/
│   ├── drivers/                # Hardware drivers (I2C, MPU6050, TOF400F, PV3901L1, Motor)
│   ├── control/                # Flight control (commander, PID, mixer)
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

## Key Design Decisions

- **Shared I2C0 bus**: `i2c_bus.c` initializes I2C0 once (`g_i2c0_bus` handle). Both MPU6050 (0x68) and TOF400F (0x29) attach via `i2c_master_bus_add_device()` — never re-init the bus.
- **Motor safety**: `MOTOR_MIN = 0.05` floor-clip (not shift-up). Throttle < 5% stops motors + resets PID to prevent ground spooling.
- **TOF skip-counter**: VL53L1X data-ready checked every 10th call (@100Hz ≈ every 100ms) to match sensor timing budget; cached values returned otherwise.
- **Control modes**: DISARMED (default) / STABILIZE / ALT_HOLD / POS_HOLD. Latter two currently degrade to STABILIZE (outer loops not yet implemented).
- **WebSocket command flow**: browser sends JSON → `http_server` → `commander_parse` → updates global `setpoint_t`. Special commands: `{"cmd": "calibrate"}`, `{"cmd": "gyro_calib"}`, `{"cmd": "level_trim"}`.

## Component Dependencies

```
drivers/  → (none, leaf)
estimation/ → drivers/ (needs MPU6050 data)
control/  → drivers/ (needs motor), estimation/ (needs attitude)
communication/ → (standalone)
main/     → all components
```

## C Headers

All component headers are public (no `private_*.h`). Include patterns:
- `i2c_bus.h` — `g_i2c0_bus` handle
- `mpu6050.h` — `mpu6050_read()`, `mpu6050_calibrate()`, `mpu6050_recalibrate_gyro()`
- `tof400f.h` — `tof400f_get_distance()`
- `pv3901l1.h` — `pv3901l1_init()`, `pv3901l1_poll()`
- `attitude.h` — `mahony_update()`, `get_roll()/get_pitch()/get_yaw()`
- `pid.h` — `pid_init()`, `pid_update()`, `pid_reset()`
- `mixer.h` — `mixer_apply()`
- `commander.h` — `commander_parse()`, `commander_set_cmd_callback()`, `setpoint_t`
- `motor.h` — `motor_set()`, `motor_stop_all()`, `motor_calibrate()`
- `wifi_ap.h` — `wifi_ap_init()`
- `http_server.h` — `http_server_start()`, `http_server_set_command_cb()`
- `web_page.h` — embedded HTML/CSS/JS frontend (single `const char *`)
