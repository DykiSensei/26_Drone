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

## Key Design Decisions

- **Shared I2C0 bus**: `i2c_bus.c` initializes I2C0 once (`g_i2c0_bus` handle). Both MPU6050 (0x68) and TOF400F (0x29) attach via `i2c_master_bus_add_device()` — never re-init the bus.
- **Motor safety**: `MOTOR_MIN = 0.05` floor-clip (not shift-up). Throttle < 5% stops motors + resets all PIDs + resets altitude/flow/position controllers to prevent ground spooling.
- **TOF skip-counter**: VL53L1X data-ready checked every 10th call (@100Hz ≈ every 100ms) to match sensor timing budget; cached values returned otherwise.
- **Deferred command execution**: Blocking operations (ESC calibration, gyro recalibration) are queued via `pending_cmd` and executed in the main loop context — never from the HTTP server task. This prevents WiFi/WebSocket freeze during calibration.
- **Control modes**: DISARMED (default) / STABILIZE / ALT_HOLD / POS_HOLD. ALT_HOLD adds TOF height PID on top of STABILIZE. POS_HOLD adds optical flow velocity hold on top of ALT_HOLD. Mode transitions auto-capture target height and reset flow integrals.
- **WebSocket command flow**: browser sends JSON → `http_server` → `commander_parse` → updates global `setpoint_t`. Special commands: `{"cmd": "calibrate"}`, `{"cmd": "gyro_calib"}`, `{"cmd": "level_trim"}`, `{"cmd": "reset_trim"}`, `{"cmd": "calibrate_motor", "motor_index": 0}`, `{"cmd": "move_to", "x": 0.5, "y": -0.3}`, `{"cmd": "move_stop"}`. Velocity commands (`vel_x`/`vel_y`) are sent via regular 50Hz stick data, not special commands.
- **Safety mechanisms**:
  - 500ms command timeout: if no WebSocket message received for >500ms → auto-DISARMED
  - All-clients-disconnected → `commander_reset_setpoint()` forces DISARMED + throttle=0
  - `motor_active` path now respects DISARMED + throttle < 5% safety
  - Frontend resets all control variables on WebSocket reconnect, sends explicit DISARMED
  - `commander_parse` parses to local temp then struct-assigns to `g_sp` (narrowed race window)

## Component Dependencies

```
drivers/       → (none, leaf)
estimation/    → drivers/ (needs MPU6050 data)
control/       → drivers/ (needs motor), estimation/ (needs attitude for angle loop)
communication/ → (standalone, only depends on ESP-IDF)
main/          → all components
```

Control modules: `pid` (leaf), `mixer` (leaf), `commander` (leaf), `altitude` (depends on pid), `flow_hold` (depends on pid), `position` (depends on pid — uses PID internally for position→velocity conversion).

## C Headers

All component headers are public (no `private_*.h`). Include patterns:

**Drivers:**
- `i2c_bus.h` — `i2c_bus_init()`, `g_i2c0_bus` handle
- `mpu6050.h` — `mpu6050_init()`, `mpu6050_read()`, `mpu6050_recalibrate_gyro()`, `mpu6050_data_t`
- `tof400f.h` — `tof400f_init()`, `tof400f_get_distance()`
- `pv3901l1.h` — `pv3901l1_init()`, `pv3901l1_get_data()`, `pv3901l1_reset_integral()`, `pv3901l1_set_yaw_mode()`, `pv3901l1_data_t`
- `motor.h` — `motor_init()`, `motor_set()`, `motor_stop()`, `motor_calibrate()`, `motor_calibrate_single()`

**Estimation:**
- `attitude.h` — `attitude_init()`, `attitude_update()`, `attitude_get_euler()`

**Control:**
- `pid.h` — `pid_init()`, `pid_update()`, `pid_reset()`, `pid_t`
- `mixer.h` — `mixer_apply()`
- `commander.h` — `commander_parse()`, `commander_get_setpoint()`, `commander_reset_setpoint()`, `commander_is_command_timeout()`, `commander_clear_pending_cmd()`, `commander_mode_name()`, `setpoint_t` (includes `vel_x`, `vel_y`, `move_to_x`, `move_to_y`), `flight_mode_t`, `CMD_*` enums
- `altitude.h` — `altitude_init()`, `altitude_update()`, `altitude_capture_target()`, `altitude_reset()`, `altitude_ctrl_t`
- `flow_hold.h` — `flow_hold_init()`, `flow_hold_set_velocity()`, `flow_hold_update()`, `flow_hold_reset()`, `flow_hold_is_active()`, `flow_hold_t`
- `position.h` — `position_init()`, `position_set_target()`, `position_update()`, `position_reset()`, `position_reached()`, `position_ctrl_t`

**Communication:**
- `wifi_ap.h` — `wifi_ap_init()`
- `http_server.h` — `http_server_init()`, `http_server_set_command_cb()`, `http_server_set_disconnect_cb()`, `http_server_broadcast()`
- `web_page.h` — embedded HTML/CSS/JS frontend (single `const char *`)

## Coding Conventions

- **Headers**: always `#pragma once` + `extern "C"` block (all headers are C++-safe)
- **Return values**: 0 = success, -1 = failure (ESP-IDF convention)
- **Globals**: `g_` prefix for file-static/module-level variables (e.g. `g_i2c0_bus`, `g_trim_roll`)
- **Logging**: `static const char *TAG = "module"` then `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`(TAG, ...)
- **Timing**: `vTaskDelay(pdMS_TO_TICKS(10))` for 100Hz loop; `esp_timer_get_time()` for variable-rate updates (flow_hold) and command timeout
- **JSON**: telemetry uses raw `snprintf` (not cJSON) — buffer is 640 bytes; commands use cJSON for parsing

## Init Ordering

Init order in `app_main()` is critical (I2C must come first, HTTP must come last):

```
i2c_bus_init → mpu6050_init → tof400f_init → pv3901l1_init
  → motor_init → attitude_init → PID init → altitude_init → flow_hold_init → position_init
  → wifi_ap_init → http_server_init
```

WiFi starts after motors so calibration doesn't conflict with the WiFi stack. HTTP server needs WiFi up + commander callback + disconnect callback registered before starting.

## Concurrency

- **setpoint_t** is shared between HTTP server task (writer) and main loop (reader) — no mutex
- Commander parses JSON into a local temp, then struct-assigns to `g_sp` (narrowed race window vs. per-field writes)
- Blocking commands (calibrate, gyro_calib, move_to, move_stop) are deferred via `pending_cmd`: HTTP task sets the flag, main loop executes it next iteration
- **500ms timeout**: `g_last_command_us` timestamp updated on each valid WebSocket parse; main loop checks `commander_is_command_timeout()` → auto-DISARMED
- **Disconnect safety**: when all WS clients disconnect, `ws_disconnect_cb` fires `commander_reset_setpoint()` — cleans both `g_sp` and the timestamp
- `pv3901l1_data_t` is filled by the `flow_rx` FreeRTOS task (Core 1, priority 10, 100ms UART poll), consumed by main loop
