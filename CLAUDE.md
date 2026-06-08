# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 quadcopter flight controller based on ESP-IDF 5.5.4. WiFi AP + WebSocket for remote control and telemetry. References Crazyflie/esp-drone architecture.

Full architecture, pinout, and design decisions are in `DESIGN.md`.

## Build & Flash

- ESP-IDF environment: `D:/Espressif/frameworks/esp-idf-v5.5.4/`
- Target: `esp32s3`
- Flash port: `COM14`

On this machine, `idf.py` is not on PATH — invoke via the full Python + idf.py path:

```bash
# Set up ESP-IDF env (needed once per shell):
export PATH="D:/Espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin:D:/Espressif/tools/cmake/3.30.2/bin:D:/Espressif/tools/ninja/1.12.1:$PATH"
export IDF_PATH="D:/Espressif/frameworks/esp-idf-v5.5.4"

# Build
python D:/Espressif/frameworks/esp-idf-v5.5.4/tools/idf.py -C C:/Users/15381/26_Drone build

# Flash + monitor
python D:/Espressif/frameworks/esp-idf-v5.5.4/tools/idf.py -C C:/Users/15381/26_Drone flash monitor

# Clean build (after messing with config)
python D:/Espressif/frameworks/esp-idf-v5.5.4/tools/idf.py -C C:/Users/15381/26_Drone fullclean

# On new machines: set target first
python D:/Espressif/frameworks/esp-idf-v5.5.4/tools/idf.py -C C:/Users/15381/26_Drone set-target esp32s3
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
│   └── system/                 # NVS-backed params (hover-throttle learning)
└── build/                      # Build artifacts (git-ignored)
```

Each component has its own `CMakeLists.txt`. The main component `REQUIRES drivers communication control estimation system`.

## Architecture

**100Hz main loop** (Core 1): sensor read → accel EMA → Mahony AHRS → setpoint copy + failsafe FSM → PID control → tilt-compensation → Mixer → Motor PWM → telemetry broadcast. Loop period locked by `vTaskDelayUntil`; `dt` is measured per-iteration via `esp_timer_get_time()` (clamped 5–50 ms) — never use the literal `0.01f` for PID/Mahony integration.

**Control chain** (STABILIZE mode):
```
MPU6050 → DLPF=21Hz + accel EMA → Mahony → Euler angles (roll/pitch/yaw)
  Commander setpoint → target angle (±30°) → Angle P → target rate
  → Rate PID (D-term LPF) → torque → Mixer (X-quad)
  → tilt-comp ÷ cos(roll)·cos(pitch) → 4×LEDC PWM → motors
```

**Horizontal movement** (vel_x/vel_y → flow_hold, move_to → position → flow_hold):
```
Web buttons → vel_x/vel_y  ──→ flow_hold velocity PID ──→ ±5° correction to target angle
P4 move_to → position PID ──→ velocity setpoint ────────→
```
Position controller uses optical flow integral as position feedback (dead-reckoning in flow units). It has two roles: **`move_to`** (one-shot move) and **position hold** (lock the current point to resist drift). In ALT_HOLD/POS_HOLD, position hold is the *default* — once airborne with good flow quality, the current x/y is captured and locked. A `move_to` runs to its target then **transitions into hold at that point** (no longer falls back to velocity=0). Web direction buttons temporarily override with manual velocity, then re-lock the new position on release. STABILIZE does not lock position (full manual).

**Task layout** (dual-core FreeRTOS):
- Core 0: WiFi protocol stack (ESP-IDF managed)
- Core 1: `main` (100Hz), `flow_rx` (UART interrupt-driven), HTTP server (event-driven)

## Key Design Decisions

- **`FLOW_ENABLED` compile flag** (`main/main.c:27`): a `#define FLOW_ENABLED 1` gates *all* optical-flow code — PV3901L1 init, the `flow_hold`/`position` updates, the `move_to`/`move_stop`/`vel_x`/`vel_y` paths. Set to `0` to build without the flow module (e.g. when the hardware is broken); STABILIZE and ALT_HOLD still work, but POS_HOLD and all horizontal movement become no-ops.
- **Shared I2C0 bus**: `i2c_bus.c` initializes I2C0 once (`g_i2c0_bus` handle). Both MPU6050 (0x68) and TOF400F (0x29) attach via `i2c_master_bus_add_device()` — never re-init the bus.
- **IMU filtering**: MPU6050 DLPF=4 (21Hz BW) replaces the default 256Hz, killing motor-vibration aliasing into the 100Hz control loop. A second software EMA (α=0.3, ≈5.7Hz) on accel feeds Mahony and `az_up` — telemetry still shows the raw accel for debugging. Gyro is not double-filtered because the rate-PID's D-term has its own LPF (`pid.c:PID_D_FILT_ALPHA`).
- **Motor safety**: `MOTOR_MIN = 0.05` floor-clip (not shift-up). Throttle < 5% stops motors + resets all PIDs + resets altitude/flow/position controllers + clears spool-up window. **Tilt-compensated throttle** (`effective_throttle /= cos(roll)·cos(pitch)`, clamped at 1.5×) maintains vertical thrust during banking — without this, a 15° lean drops 3.4% lift and the alt PID overshoots on level-out.
- **ESC spool-up**: `takeoff` command sets `g_spoolup_end_us = now + 200ms` (`SPOOLUP_DURATION_US`). During that window mixer output is overridden with `MOTOR_IDLE_THROTTLE` (5%) — controllers keep running (no cold-start lag) but motors are forced to idle until all 4 ESCs sync. Without this, 30-80ms ESC sync jitter would tip the drone before angle control engages.
- **Altitude loop**: height PID (P+I, no PID-D) plus a **vertical-speed (`vz`) damping term** that is the real fix for up/down oscillation. `vz` is fused via complementary filter — IMU world-up accel (high-rate prediction) corrected by fresh TOF samples (low-rate absolute, `VZ_FUSE_K=0.2`). **Ground-effect handling**: when `current_m < ALT_GROUND_EFFECT_M` (0.30 m), the alt PID's `freeze_integral` flag is set — prevents the bounced downwash from spinning up a negative I term that bites you on exit. **Takeoff ramps the target altitude** (`ALT_RAMP_RATE=0.30 m/s`) instead of stepping. `vz` is exposed in telemetry as `alt.vz`.
- **NVS-backed parameters** (`components/system/params.c`): two fields persist across boots, both written together by `params_save()` on DISARM transition (each tracked with its own dirty flag for flash-wear protection):
  - **`hover_throttle`** — slow EMA learner (α=0.001) updated when steady hover detected (alt_mode + height>30cm + |target-actual|<5cm + |vz|<0.1m/s for 1s). Range `[0.20, 0.65]`, default 0.40, save threshold 1%. On takeoff `commander_set_throttle(params_get_hover_throttle())` injects it into `g_sp.throttle` as feed-forward so the alt PID only fills the delta.
  - **`flow_imu_scale`** — direct overwrite (no learning), set by user via `{"cmd":"flow_scale","s":<val>}`. Range `[0, 100]`, default 1.0, save threshold 0.05. main loads it via `commander_set_flow_imu_scale(params_get_flow_imu_scale())` at startup, and `params_set_flow_imu_scale(sp->flow_imu_scale)` each frame so changes propagate. Telemetry broadcasts as `flow.s` so the web input reflects the loaded value.
- **TOF skip-counter**: VL53L1X data-ready checked every 10th call (@100Hz ≈ every 100ms) to match sensor timing budget; cached values returned otherwise.
- **Optical flow soft-start + complementary filter**: `flow_hold` no longer hard-cuts at `qual > 50`. Continuous quality weight `q_gain` ramps linearly between `FLOW_QUALITY_LOW=30` and `FLOW_QUALITY_HIGH=80`; PID output is multiplied by `q_gain` so low-quality frames produce small but non-zero corrections (closes the "qual not ready yet → drift" window on takeoff). When `q_gain < 0.5` the flow-PID's `freeze_integral` is set to avoid noise wind-up. **vx/vy use IMU+flow complementary filter**: `flow_hold_predict()` runs every loop (100Hz) integrating world-frame accel into `vx_est`/`vy_est` with slow leak (`IMU_LEAK=0.999`, τ≈10s), and runs the PID at 100Hz; `flow_hold_update()` only fires on new flow frames (~50Hz) and corrects `vx_est` toward the flow measurement with strength `FLOW_CORRECT_K × q_gain`. The `imu_scale` (m/s² → flow_unit/s²) is runtime-tunable via `{"cmd": "flow_scale", "s": <val>}` — observe `flow.vx` vs `flow.cx` in telemetry to calibrate.
- **Deferred command execution**: Blocking operations (ESC calibration, gyro recalibration) are queued via `pending_cmd` and executed in the main loop context — never from the HTTP server task. This prevents WiFi/WebSocket freeze during calibration. **TWDT is temporarily removed** for blocking commands (`CMD_CALIBRATE`, `CMD_CALIBRATE_MOTOR`, `CMD_GYRO_CALIB`) and re-added afterward; `prev_us` and `next_wake` are also reset so the dt clamp doesn't see the multi-second gap.
- **Control modes**: DISARMED (default) / STABILIZE / ALT_HOLD / POS_HOLD. ALT_HOLD adds TOF height PID on top of STABILIZE; POS_HOLD adds optical-flow velocity hold on top. **Both ALT_HOLD and POS_HOLD now run the position-hold loop** (lock current x/y to resist drift, flow-quality gated, threshold lowered to `qual > 30` to match the flow_hold soft-start). Entering an altitude mode auto-captures the target height. The flow integral is **no longer reset** on POS_HOLD entry — position hold uses the absolute integral as a relative lock reference.
- **WebSocket command flow**: browser sends JSON → `http_server` → `commander_parse` → updates global `setpoint_t`. Special commands: `{"cmd": "calibrate"}`, `{"cmd": "gyro_calib"}`, `{"cmd": "level_trim"}`, `{"cmd": "reset_trim"}`, `{"cmd": "calibrate_motor", "motor_index": 0}`, `{"cmd": "move_to", "x": 0.5, "y": -0.3}`, `{"cmd": "move_stop"}`, `{"cmd": "takeoff", "height": 0.5, "base_throttle": 0.4}`, `{"cmd": "flow_comp", "kx": -2.5, "ky": -2.5}` (gyro-compensation), `{"cmd": "flow_scale", "s": 1.0}` (IMU complementary-filter scale). Velocity commands (`vel_x`/`vel_y`) are sent via regular 50Hz stick data, not special commands.
- **Safety mechanisms**:
  - **TWDT** (`TWDT_TIMEOUT_MS = 1000`): the main loop is registered to the Task Watchdog; an I2C/UART hang that stops the heartbeat triggers panic + reboot rather than freezing the last PWM frame indefinitely.
  - **Arming prerequisite**: `commander_parse` refuses to leave DISARMED if `throttle >= ARMING_THROTTLE_THRESHOLD` (5%) — prevents residual-throttle-after-mode-switch from launching the drone on web reconnect / power-up.
  - **Graduated failsafe** (replaces the old "500ms → DISARMED = freefall"): `commander_us_since_last_command()` drives a state machine — `HOLD` (0.5s, zero stick) → `DESCEND` (1.0s, force STABILIZE + throttle ramps down at `FS_DESCEND_RATE=0.05/s`) → `LAND` (5.0s, full DISARMED + `commander_reset_setpoint()` so reconnect must re-arm). Telemetry exposes the state as `failsafe`.
  - All-clients-disconnected → `commander_reset_setpoint()` forces DISARMED + throttle=0 (preserves `flow_kx/ky` and `flow_imu_scale` calibration values).
  - `motor_active` path respects DISARMED + throttle < 5% safety.
  - Frontend resets all control variables on WebSocket reconnect, sends explicit DISARMED.
  - `commander_parse` parses to local temp then struct-assigns to `g_sp` (narrowed race window).

## Component Dependencies

```
drivers/       → (none, leaf)
estimation/    → drivers/ (needs MPU6050 data)
control/       → drivers/ (needs motor), estimation/ (needs attitude for angle loop)
communication/ → (standalone, only depends on ESP-IDF)
system/        → nvs_flash (ESP-IDF)
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
- `pid.h` — `pid_init()`, `pid_update()`, `pid_reset()`, `pid_t` (fields: `prev_meas` for derivative-on-measurement, `d_filt` for D-term EMA LPF, `freeze_integral` set externally to halt I-term accumulation in ground-effect/low-quality situations)
- `mixer.h` — `mixer_apply()`
- `commander.h` — `commander_parse()`, `commander_get_setpoint()`, `commander_reset_setpoint()`, `commander_set_throttle()` (feed-forward write into `g_sp.throttle` for takeoff hover-throttle injection), `commander_is_command_timeout()`, `commander_us_since_last_command()` (drives graduated failsafe FSM), `commander_clear_pending_cmd()`, `commander_mode_name()`, `setpoint_t` (includes `vel_x`, `vel_y`, `move_to_x`, `move_to_y`, `takeoff_height`, `takeoff_throttle`, `flow_kx/ky`, `flow_imu_scale`), `flight_mode_t`, `CMD_*` enums
- `altitude.h` — `altitude_init()`, `altitude_update()` (takes `az_up` for vz complementary filter; freezes integral below `ALT_GROUND_EFFECT_M=0.30m`), `altitude_capture_target()` (hold in place), `altitude_set_target()` (ramp from current height to a final target — used by takeoff), `altitude_reset()`, `altitude_ctrl_t`
- `flow_hold.h` — `flow_hold_init()`, `flow_hold_set_velocity()`, `flow_hold_set_gyro_comp()` (gyro-comp gains, default −2.5/−2.5), `flow_hold_set_imu_scale()` (IMU complementary-filter scale, default 1.0), `flow_hold_predict()` (run every loop with `ax_world/ay_world/dt` — does the IMU integration + 100Hz PID), `flow_hold_update()` (only on new flow frames — gyro-comp + flow correction of `vx_est`), `flow_hold_reset()`, `flow_hold_is_active()`, `flow_hold_t` (extra fields: `vx_est/vy_est` for the fused estimate, `imu_scale`, `flow_x_corr/y_corr` debug)
- `position.h` — `position_init()`, `position_set_target()` (move_to, one-shot), `position_hold_start()` (lock current point, persistent — does not exit on reach), `position_update()`, `position_reset()`, `position_reached()`, `position_ctrl_t` (has `hold` flag distinguishing the two)

**Communication:**
- `wifi_ap.h` — `wifi_ap_init()` (also calls `nvs_flash_init()`, which `params_init()` depends on)
- `http_server.h` — `http_server_init()`, `http_server_set_command_cb()`, `http_server_set_disconnect_cb()`, `http_server_broadcast()`
- `web_page.h` — embedded HTML/CSS/JS frontend (single `const char *`)

**System:**
- `params.h` — `params_init()` (must run *after* `wifi_ap_init` so NVS is ready), `params_get_hover_throttle()`, `params_update_hover_throttle()` (slow EMA learner), `params_get_flow_imu_scale()`, `params_set_flow_imu_scale()` (direct overwrite — main syncs `g_sp.flow_imu_scale` here every frame so user `flow_scale` web commands flow through), `params_save()` (call on DISARM — checks both `hover_throttle` and `flow_imu_scale` dirty flags independently)

## Coding Conventions

- **Headers**: always `#pragma once` + `extern "C"` block (all headers are C++-safe)
- **Return values**: 0 = success, -1 = failure (ESP-IDF convention)
- **Globals**: `g_` prefix for file-static/module-level variables (e.g. `g_i2c0_bus`, `g_trim_roll`)
- **Logging**: `static const char *TAG = "module"` then `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`(TAG, ...)
- **Timing**: main loop uses `vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(10))` for jitter-free 100Hz; `dt` is **measured** with `esp_timer_get_time()` and clamped `[DT_MIN, DT_MAX] = [5ms, 50ms]` (never use the literal `0.01f` — WiFi/HTTP preemption stretches the period and breaks PID integration). Same `esp_timer_get_time()` drives variable-rate updates (flow_hold), failsafe FSM, command timeout, hover-learning steady-state timer.
- **JSON**: telemetry uses raw `snprintf` (not cJSON) — buffer is 768 bytes; commands use cJSON for parsing

## Init Ordering

Init order in `app_main()` is critical (I2C first, HTTP last, `params_init` after `wifi_ap_init`):

```
i2c_bus_init → mpu6050_init → tof400f_init → pv3901l1_init
  → motor_init → attitude_init → PID init → altitude_init → flow_hold_init → position_init
  → wifi_ap_init → params_init → http_server_init
  → esp_task_wdt_init/add(NULL)  (just before main loop)
```

- WiFi starts after motors so calibration doesn't conflict with the WiFi stack.
- `wifi_ap_init` calls `nvs_flash_init()`; `params_init` depends on this, so it must come after.
- HTTP server needs WiFi up + commander callback + disconnect callback registered before starting.
- TWDT registration is the last step before entering the 100Hz loop — the main task is the one being watched.

## Concurrency

- **setpoint_t** is shared between HTTP server task (writer) and main loop (reader) — no mutex
- Commander parses JSON into a local temp, then struct-assigns to `g_sp` (narrowed race window vs. per-field writes)
- `commander_set_throttle()` is a single-float reverse write (main → `g_sp.throttle`) used only by takeoff hover feed-forward; it has a small race with `commander_parse` (HTTP task may overwrite within ~1ms of takeoff), but the worst case is one frame of "no feed-forward" — degrades gracefully to the pre-Task-5 behavior.
- Blocking commands (calibrate, gyro_calib, takeoff, move_to, move_stop) are deferred via `pending_cmd`: HTTP task sets the flag, main loop executes it next iteration
- **Main loop creates a local copy** of the setpoint every iteration (`local_sp = *commander_get_setpoint()`) and the failsafe FSM overrides fields on the copy — never on `g_sp` (which would race with HTTP task writes)
- **Graduated failsafe FSM** (`fs_state`) lives entirely in main-loop locals; driven by `commander_us_since_last_command()`. Transitions log; current state is broadcast in telemetry as `failsafe`.
- **Disconnect safety**: when all WS clients disconnect, `ws_disconnect_cb` fires `commander_reset_setpoint()` — cleans both `g_sp` and the timestamp (but preserves `flow_kx/ky` and `flow_imu_scale` calibration)
- `pv3901l1_data_t` is filled by the `flow_rx` FreeRTOS task (Core 1, priority 10, 100ms UART poll), consumed by main loop
- **TWDT**: only the main task is registered. Blocking calibration commands `esp_task_wdt_delete(NULL)` before executing and `add(NULL)` after — without this the 1s timeout would panic during the ~12s ESC calibration. After re-add, `prev_us` and `next_wake` are also reset so the next iteration's `dt` doesn't clamp to `DT_MAX` from the calibration gap.
