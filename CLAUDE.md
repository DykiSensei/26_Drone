# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 quadcopter flight controller based on ESP-IDF 5.5.4. WiFi AP + WebSocket for remote control and telemetry. References Crazyflie/esp-drone architecture.

Full architecture, pinout, and design decisions are in `DESIGN.md`. Flight/calibration procedures for humans are in `README.md`. The S3↔P4 serial protocol spec for the P4-side team is `P4LINK_PROTOCOL.md` (authoritative byte-level definition: `components/communication/p4link_protocol.h`).

## Current Status & TODO (last updated 2026-07-05)

**Flight-verified**: STABILIZE, ALT_HOLD (vz damping + takeoff target ramp), auto-takeoff, per-motor trim. **Metric flow pipeline flight-validated 2026-07-05**: attitude very stable, hover position drift ~30cm — and **yaw is nearly constant in flight**, so the drift is NOT lock-frame rotation (this finding deprioritized the magnetometer, see roadmap).

**Current priority — shrink the ~30cm hover drift** (all-flow levers, no new hardware):
1. Hover lower (~0.5m) **as a diagnostic only**: flow quantization noise scales with height — 1 count = 0.12 m/s at 1m, half that at 0.5m. ⚠️ NOT the final answer: the operating cruise altitude is **~1.7m** (user-confirmed 2026-07-05, P4 vision needs it), where 1 count ≈ 0.2 m/s — high-altitude drift is ultimately absorbed by the P4 visual servo loop, not by flying lower.
2. Raise position-loop gain: `POS_KP` 1.5 → 2.0–2.5 in `position.c` (watch for lock-point oscillation), optionally velocity `FLOW_KP` 8 → 10–12 in `flow_hold.c`.
3. Better floor texture / lighting raises qual → stronger flow correction.

**Done & bench-calibrated 2026-07-04 — NOT yet flight-tested** (builds clean for esp32s3):
1. **Metric flow pipeline** — all horizontal control in m/s / meters (details in Architecture below); fixed the 5-10x altitude-dependent loop-gain drift that caused hover instability and takeoff lateral drift. Bench-confirmed constants, all now code defaults (still lost on reboot — no NVS yet, but defaults match bench values):
   - `flow_scale = 0.00244` rad/count — verified by 1m hand-carry push (<10cm repeatability, accepted 2026-07-04)
   - gyro-comp `kx = ky = +1.0` (clean tilt test after DC fixes)
   - flow mount remap in `pv3901l1.c parse_byte()` (module mounted 90° rotated: body-forward = raw_y, body-right = -raw_x)
   - accel axis mapping verified by fast-push test (IMU X points backward -> negated; IMU Y points right -> direct)
2. **Estimator design rules** (each learned from a real bench-reproduced bug on 2026-07-04 — do not regress):
   - **Flow provides DC velocity; the IMU accel channel is demoted to `ACCEL_GAIN = 0.0`** (flow-only velocity). Body-frame accel is structurally unusable for DC: tilt leaks `g*sin(theta)` (3° hand tilt = 0.5 m/s²; a hover's trim tilt does the same), and any HPF pays back absorbed spike-impulse after a fast stop (zero DC gain). Machinery kept — restore via one constant once world-frame accel rotation exists (EKF/mag milestone).
   - **Gyro comp in the metric domain** (`comp = k * omega * h`, frame-rate independent — counts-domain constant k cannot null with measured, jittering dt_frame) **and only with high-passed gyro** (`omega - omega_lp`, tau≈1-2s — DC bias injects `k*bias*h`, past the deadband at 1m from just 1°/s).
   - **Consume flow via the driver's accumulated counts** (`acc_x/acc_y`, taken-and-zeroed by `get_data`), never latest-frame sampling: UART batching (100ms timeout on a 256-byte read at 19200 baud) once dropped ~95% of displacement counts -> "same move, random magnitude". UART read timeout is now 10ms.
   - **Input-conditioning filters run continuously** (accel EMA, DC trackers) — per-loop resets anchored the trackers to a scaled-down baseline, and every arm/calib transition produced a ~2s fake-velocity transient (~0.5 m/s, tens of cm of phantom drift — also hit every real takeoff).
   - **Freeze when untrusted**: `quality_gain < 0.05` or >0.3s without a flow frame -> freeze `pos_x/y_m`, fast-decay `vx/vy_est` (IMU-only integration is amplified ~10x by the velocity-leak steady state).
   - Debugging entry point: telemetry `flow.qg` (fusion trust, color-coded in the web UI next to Qual) — check it first whenever position behaves oddly. Noise floor: one flow count at 1m height = 0.12 m/s metric, so judge position stability, not instantaneous Vel; precision tests are better at ~0.5m.
2b. **测试模式 / flow_calib** (`{"cmd":"flow_calib","on":1}`, 前端"测试模式"按钮): in DISARMED **motors stay stopped (props never spin — structural safety, the DISARMED branch always calls motor_stop)** but **all estimators keep running** — hand-carry horizontally to calibrate flow_scale (watch `flow.x`), lift vertically to watch `alt.vz` respond (`altitude_update_vz` runs each tick, not reset). Auto-cleared on disconnect/timeout. **This is the safe bench-test mode for validating estimators without arming** (ALT_HOLD would spin props). Note: `alt.azb` (accel-Z bias, always in telemetry) needs no mode — read it anytime after `gyro_calib`.
3. **Safety hardening** — calibration/trim commands DISARMED-gated; manual motor test now only works in DISARMED (bench workflow inverted vs before!); telemetry 20Hz async via `httpd_queue_work`; IMU failsafe (200ms -> DISARM); TOF staleness (500ms -> invalid, ALT_HOLD degrades to stick throttle); WS client-limit rejection.
4. Loop timing: `vTaskDelayUntil` fixed cadence + measured dt.

5. **BN-880 magnetometer driver + frontend cleanup (2026-07-05)** — `drivers/bn880_mag.c` probes three chip variants (QMC5883L@0x0D, HMC5883L@0x1E, IST8310@0x0E; HMC data order is X,Z,Y big-endian vs QMC/IST X,Y,Z little-endian; IST is single-measurement mode, re-triggered each read). On total probe failure it scans the whole bus and logs every ACKing address (only 0x29/0x68 = compass unpowered/not on bus). Gauss output in **module axes — no body-frame alignment or hard/soft-iron calibration yet** (that's the Mahony 9-axis step). ⚠️ **Hardware unresolved & PARKED (user decision 2026-07-05)**: compass never ACKed (vendor says "QMC5883"); flight test showed yaw nearly constant → drift is not yaw-induced → mag priority dropped. When resumed: flash latest firmware via the native-USB port, read the boot-log bus scan — unknown address = new variant to support (QMC5883P is 0x2C); only 0x29/0x68 = module-side hardware fault (check power LED, feed VCC 5V, verify silk-vs-cable pin order, continuity SDA→GPIO9/SCL→GPIO8). Init is non-fatal (`mag.ok=0` in telemetry when absent); read at 20Hz in the telemetry block. Web UI simplified the same day: removed IMU-raw/PID panels, flow debug rows (fps/raw/comp/corr/lock-target), GyroComp+FlowScale inputs (values are firmware defaults now; recalibrate via WS `flow_comp` command), dpad nudge buttons, battery placeholder; telemetry JSON trimmed to match (accel/gyro/pid/alt.out/flow debug fields dropped).

6. **MG995 机械爪驱动 (2026-07-05, 台架标定完成)** — `drivers/servo_grip.c`: GPIO10, LEDC TIMER_1/CH4 @50Hz (电机占 TIMER_0/CH0-3), 500–2500µs↔0–180° (参数沿用用户已验证的 esp32s3-mg995-servo-web 库), 主循环 100Hz 限速逼近 120°/s (防电流尖峰+飞行反扭, 无独立任务)。**台架实测 2026-07-05: 0°=完全张开, 90°=完全闭合; ⚠️ >90° 齿条行程用尽、舵机空转打滑 — `SERVO_GRIP_MAX_DEG=90` 是驱动层硬限位 (clamp 在驱动内部, 任何上层都越不过), commander 侧同步钳位, 前端滑条 0–90**。机械爪兼作**起落架**: 地面停放/上电默认完全张开 (OPEN=0°, init 即输出)。WS `grip` 命令经 setpoint 非阻塞下发 (任意模式可用), 安全复位**保留**爪角度 (防断连掉落已抓目标), 遥测 `grip` 字段 + 前端调试面板。**待办**: ⚠️ 硬件接线必须独立 5V BEC (堵转 1A+, 共轨会在闭爪瞬间拉复位主控)。**传感器布局 (2026-07-06 用户确认)**: TOF/光流已改装位置, 抓着目标**不被遮挡** (携带飞行全功能); P4 相机在爪末端, **抓取后必盲 → 携带阶段无视觉**, 投放导航只能靠航位推算 (pos 坐标系同一解锁周期内连续)+人工; 相机被挡=天然抓取成功信号 (协议 v1.1 拟加 TRACK_BLOCKED)。降落策略方向: 永远先投放再降落 (垃圾负载, 投放永远安全); 拟定"标记投放点"方案见 DESIGN Phase 6。最终目标 (S3+P4 视觉伺服抓取) 见 DESIGN.md Phase 6。

7. **S3↔P4 串口协议 v1 定稿 (2026-07-05, ⚠️ 纯理论设计 — P4 硬件未就绪, 不可实测)** — 唯一权威定义 `communication/p4link_protocol.h` (P4 工程持同一份拷贝), 设计文档 DESIGN.md §3.10, **P4 侧开发独立规格 `P4LINK_PROTOCOL.md`** (字节表/解析器状态机/P4 实现清单/联调步骤, 自包含无需读飞控代码)。要点: P4=纯传感器永不指挥飞行 / UART2 TX=GPIO17 RX=GPIO18 @115200 (引脚已预留) / 帧 `AA 55 type len payload crc16` (CCITT-FALSE, 两侧必须用头文件里的参考实现) / 仅两条消息: MSG_STATE S3→P4 50Hz (姿态/高度/模式/爪角), MSG_TARGET P4→S3 (机体系偏差 dx/dy 同 move_to 约定 + conf + track, 未锁定时 ≥10Hz 心跳) / ts_echo 回传免时钟同步测陈旧度 / S3 四道安全门: 限幅 ±1m, conf≥50, 陈旧度 ≤200ms, 链路超时 500ms→中止抓取回悬停 (绝不 DISARM) + 准静态门 |tilt|>5° 不采信。**下一步**: P4 就绪后按 §3.10 联调清单实测。

8. **p4link 驱动 + 抓取任务状态机 (2026-07-05, 编译通过, ⚠️ 台架/实飞未测)** — `communication/p4link.c`: UART2 (TX=17/RX=18) + `p4link_rx` 任务 (Core1 prio9) 喂解析状态机, take 语义交付原始测量; **安全门控在 main.c** (协议四道门 + 准静态门), 主循环 50Hz 广播 MSG_STATE (非阻塞 uart_tx_chars)。`control/grab_mission.c`: IDLE→ALIGN(P4 边降边修: 视觉修正限速 1s/次, **对准容差随高度放宽** max(8cm, 0.15×h) — 高空光流噪声大只做粗对准, ×3 次连续确认, 台阶 0.25m, 视觉地板 0.35m, 对准计时每级台阶重置 + 120s 任务总超时)→DESCEND(末段开环)→GRASP(**TOF ≤ sp->grab_tof_m 触发闭爪**, 默认 0.20m=TOF 落地安装读数, 前端滑条 0.10–0.40 可调)→ASCEND(回起始高度)→IDLE。**参数按 ~1.7m 工作巡航高度设计**(用户 2026-07-05 确认: 下降行程 ~1.5m, 斜坡本身 5s)。闭爪经 `commander_set_grip()` 写 setpoint 且 GRASP/ASCEND **每拍重写** (与 WS 解析整结构体赋值的窄竞态丢写 10ms 自愈); 全局中止 = 模式退出/油门<5%/TOF 失效, **中止绝不张爪**。前端"抓取任务"面板: 触发高度滑条 + **测试抓取按钮** (`grab_start test=1`: 无 P4, 跳过 ALIGN, 假设触发时位置精确, 下降不做视觉修正 — 用户 2026-07-05 指定的测试假设) + 中止按钮; 遥测 `grab:{st,p4}`。**待办**: 实飞验证触发高度/时序, 调参回填 grab_mission.c 顶部常量。

9. **投放任务 + 标记投放点 (2026-07-06, 编译通过, ⚠️ 实飞未测)** — 主线方案落地 (用户认可: 永远先投放再降落, 垃圾负载投放永远安全)。`grab_mission` 扩展 GRAB_MISSION_GRAB/DROP 两种任务类型 + GOTO(5)/RELEASE(6) 状态: **就地投放** = 测试抓取镜像 (下降→TOF≤drop_tof_m 触发张爪→回升, 默认 0.30m 前端滑条 0.15–0.60); **返航投放** = 悬停筐上方按"标记投放点"存 flow 航位坐标 (需 qg≥0.05) → 抓取后 `drop_start ret=1` → GOTO 状态 move_to 返航 (到达判据: 距标记 <0.25m 且位置环 hold; 用户方向键干预后限速 1s 重发 move_to 自愈) → 下降张爪 → 回升。**标记仅同一解锁周期有效** — main.c 在两处 flow_hold_reset 旁联动 clear_mark (DISARMED 分支 + 低油门分支), 坐标系复位标记即失效。投放后爪=张开=起落架就绪可直接降落; ASCEND 仅抓取任务每拍重写闭合角, 投放任务不干预。前端: 投放点状态行 + 投放高度滑条 + 标记投放点/就地投放/返航投放三按钮; 遥测 `grab.mk`。**待办**: 实飞验证返航精度 (配大开口筐吸收 ~0.5m 航位误差); P4 上线后协议 v1.1 加 TRACK_BLOCKED (相机被挡=抓取成功信号)。

10. **Vz 常值偏移修复 (2026-07-06, 编译通过, ⚠️ 实飞待验证)** — 实飞观察: 上升中 Vz 读负、起飞漂移大难以安全进悬停, 用户怀疑位置环从未介入 — 代码证实: accel-Z 零偏从未校准 (审计老项, MPU6050 规格 ±0.1g) → `az_up` 常值假加速度 → vz 互补滤波稳态偏移 ≈ **0.5×零偏** (K=0.2, TOF 10Hz) → 起飞位置锁的 |vz|<0.15 门永远不过, `g_position_lock_pending` 挂死, ps 停在 wait。修复两层: ① `mpu6050.c` 初始+重校准均计算 Z 零偏 (avg − 9.80665) 并在 read 扣除; `gyro_calib` 以 WARN 打印 **"accel Z bias"** (地面确认零偏大小的唯一入口 — **DISARMED 下遥测 Vz 恒 0 是代码行为**, altitude_reset 每拍清零, 手持竖动测不出任何东西); ② `altitude.c` 新增 az_up 直流跟踪器 (`altitude_track_az`, tau=2s, 主循环每拍**全模式**调用; `altitude_reset` 不清零 — flow_hold 同款"输入调理滤波器连续运行"规则), vz 只积分高通分量, 吸收静态校准治不了的**飞行中电机振动整流直流下移**。验证清单: 烧录→gyro_calib 看 Z bias 数值; 实飞悬停 Vz 围绕 0 波动、ps 进入 lock(2)。若爬升段漂移仍大 → 下一杆是 POS_KP/FLOW_KP (roadmap 既有项)。2026-07-06 追加: Z 零偏经 `mpu6050_get_accel_z_bias()` 进遥测 `alt.azb` (免串口, 前端 TOF 面板显示, |zb|>0.3 变黄); vz 互补滤波拆出 `altitude_update_vz()` 供测试模式在 DISARMED 下推进 (电机停转看 Vz 响应手持提放)。**地面滑行测 flow/位置保持无效** (爪腿撑起+倾斜+电机蹭地, 净位移≠肉眼峰值); 干净测 flow_scale 必须用测试模式手持匀速移动已知距离。

**Sensor roadmap** (BN-880 GPS+magnetometer module purchased, plan agreed 2026-07):
1. ✅ Metric flow + dt fix — bench-calibrated 2026-07-04, **flight-validated 2026-07-05** (stable attitude, ~30cm drift)
2. ⏸️ **Magnetometer — PARKED 2026-07-05**: driver complete (3-variant probe + bus-scan diagnostic) but the compass never ACKs on the bus (hardware fault suspected, see item 5 above for the resume checklist). Priority dropped because flight showed yaw nearly constant — the yaw-drift benefit doesn't apply; still needed later for heading hold / yaw-proof world-frame position / P4 multi-step maneuvers / GPS.
3. ⏸️ GPS — parked with the mag; only for outdoor navigation (meter-level accuracy, useless indoors). UART1 RX (GPIO44) is taken by optical flow; use another UART.

**Open issues from the 2026-07-02 code audit** (roughly priority-ordered):
- `setpoint_t` torn reads: main loop dereferences the live `g_sp` pointer many times per iteration — copy the struct once per loop under a small lock; `pending_cmd` is a single slot that can drop a command arriving between execute and clear — replace with a FreeRTOS queue
- ~~accel-Z bias never calibrated~~ — **fixed 2026-07-06** (see status item 10)
- `altitude.c` detects fresh TOF samples via float equality (`current_m != prev_m`) — a perfectly steady reading stalls the vz correction; the TOF driver should export a fresh flag instead
- No battery voltage ADC (frontend already parses a `battery` telemetry field that firmware never sends); no low-voltage failsafe — most common crash cause for hobby quads
- No pre-arm checks on takeoff (attitude sane / TOF valid / flow alive)
- RGB status LED (GPIO48, WS2812) is in the pinout but has no driver
- NVS parameter persistence (`components/system/` planned): trim, mtrim, flow_kx/ky, flow_scale all lost on reboot
- Dead code: `commander_set_cmd_callback`/`g_cmd_cb` never called; unreachable rate-mode `else` branch in main.c angle control (outer branch already excludes DISARMED)
- `pid_t` shadows the POSIX type from `<sys/types.h>` — rename to e.g. `pid_ctrl_t` eventually
- No unit tests / CI; `web_page.h` is a 260-line C string macro (consider `EMBED_TXTFILES` with a real .html file)

## Build & Flash

- ESP-IDF environment: `C:/Espressif/frameworks/esp-idf-v5.5.4/`
- Target: `esp32s3`
- Flash port: **use the board's native-USB port** (enumerates as "USB 串行设备", COM12 as of 2026-07-05; number shifts across replugs — check `Get-CimInstance Win32_PnPEntity`). ⚠️ The CH343 port ("USB-Enhanced-SERIAL CH343", COM13) **cannot flash**: it bridges UART0 (GPIO43/44) and GPIO44 is permanently driven by the optical-flow module's TX, jamming PC→chip data ("Download mode detected, but no sync reply"). Console logs appear on both ports (UART0 primary + USB-Serial-JTAG secondary).

On this machine, `idf.py` is not on PATH. **Must run from PowerShell/cmd — idf.py rejects MSYS/bash environments.** Invoke via the IDF Python env + full idf.py path:

```powershell
# Set up ESP-IDF env (needed once per shell):
$env:PATH = "C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;" + $env:PATH
$env:IDF_PATH = "C:\Espressif\frameworks\esp-idf-v5.5.4"

# Build
& C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe C:\Espressif\frameworks\esp-idf-v5.5.4\tools\idf.py -C C:\Users\15381\OneDrive\Desktop\drone\26_Drone build

# Flash + monitor
& C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe C:\Espressif\frameworks\esp-idf-v5.5.4\tools\idf.py -C C:\Users\15381\OneDrive\Desktop\drone\26_Drone flash monitor

# Clean build (after messing with config)
& C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe C:\Espressif\frameworks\esp-idf-v5.5.4\tools\idf.py -C C:\Users\15381\OneDrive\Desktop\drone\26_Drone fullclean

# On new machines: set target first
& C:\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe C:\Espressif\frameworks\esp-idf-v5.5.4\tools\idf.py -C C:\Users\15381\OneDrive\Desktop\drone\26_Drone set-target esp32s3
```

(A non-fatal `ESP_ROM_ELF_DIR` gdbinit warning appears during builds — safe to ignore; the build still completes.)

## Pinout

| Signal | GPIO | Notes |
|--------|------|-------|
| I2C0 SDA | 9 | Shared: MPU6050 + TOF400F + BN-880 mag |
| I2C0 SCL | 8 | Shared: MPU6050 + TOF400F + BN-880 mag |
| UART1 RX | 44 | PV3901L1 optical flow (TX-only module) |
| YAW_MODE | 15 | PV3901L1 yaw mode select |
| M0 (FR, CCW) | 14 | LEDC PWM |
| M1 (FL, CW) | 11 | LEDC PWM |
| M2 (RL, CCW) | 13 | LEDC PWM |
| M3 (RR, CW) | 12 | LEDC PWM |
| Grip servo | 10 | MG995 机械爪, LEDC 50Hz (TIMER_1/CH4; motors own TIMER_0/CH0-3 @400Hz) |
| P4 link TX | 17 | UART2 → P4 RX (p4link driver active @115200; P4 hardware not connected yet) |
| P4 link RX | 18 | UART2 ← P4 TX |
| RGB LED | 48 | WS2812 status indicator |

## Directory Structure

```
26_Drone/
├── main/main.c                 # Entry point — init then 100Hz main loop
├── components/
│   ├── drivers/                # Hardware drivers (I2C, MPU6050, TOF400F, PV3901L1, BN880 mag, Motor, Servo grip)
│   ├── control/                # Flight control (commander, PID, mixer, altitude, flow_hold, position, grab_mission)
│   ├── communication/          # WiFi AP + HTTP/WebSocket server + embedded web frontend + p4link (S3↔P4 UART)
│   ├── estimation/             # Mahony AHRS attitude filter
│   └── system/                 # Parameter storage (NVS) — planned
└── build/                      # Build artifacts (git-ignored)
```

Each component has its own `CMakeLists.txt`. The main component `REQUIRES drivers communication control estimation`.

## Architecture

**100Hz main loop** (Core 1): sensor read → Mahony AHRS → Commander setpoint → PID control → Mixer → Motor PWM → telemetry broadcast (20Hz, async — never blocks the loop)

**Control chain** (STABILIZE mode):
```
MPU6050 → Mahony → Euler angles (roll/pitch/yaw)
  Commander setpoint → target angle (±30°) → Angle P → target rate
  → Rate PID → torque → Mixer (X-quad) → 4×LEDC PWM → motors
```

**Horizontal movement** (vel_x/vel_y → flow_hold, move_to → position → flow_hold):
```
Web buttons → vel_x/vel_y  ──→ flow_hold velocity PID ──→ ±8° correction to target angle
P4 move_to → position PID ──→ velocity setpoint ────────→
```
**Flow sign convention**: the raw module axes are remapped to the body frame at a single point — `parse_byte()` in `pv3901l1.c` (`flow_x = raw_y`, `flow_y = -raw_x`; bench-verified 2026-07-04: this module is mounted rotated 90°, hand-moving forward gave raw_y > 0, left gave raw_x > 0). Everything downstream (driver integral, telemetry `fx/fy`, gyro comp, control) is body-frame: forward gives `fx > 0`, right gives `fy > 0`. **If the module is ever remounted, fix only those two driver lines, re-verify by hand-pushing (forward → fx > 0, right → fy > 0), and re-check the gyro-comp signs (kx/ky) with the tilt test.** The API's `vel>0=forward/right` maps *directly* (no negation) to the flow_hold setpoint. The angle-direction inversion lives in one place: `flow_hold_predict()` negates its PID outputs (positive pitch angle = nose-up = backward acceleration, so intercepting forward drift needs +pitch from a negative PID error). In the IMU predict path `accel_x` is negated while `accel_y` feeds directly — **bench-verified 2026-07-04 by fast-push test** (push forward → accel_x reads negative ⇒ IMU X points backward; push left → accel_y reads negative ⇒ IMU Y points right). Slow-push translation tracking verified correct end-to-end the same day.
**Everything horizontal is metric now**: `flow_hold_update()` converts raw flow counts to m/s (`v = counts × flow_scale × TOF_height / dt_frame`, `flow_scale` default 0.00244 rad/count for the PMW3901 optics family, runtime-tunable via `{"cmd":"flow_comp","scale":..}`). This removed the old raw-flow-unit plumbing whose effective loop gain varied 5–10× with altitude — the root cause of height-dependent hover instability and lateral drift during the takeoff climb. Velocity setpoints are m/s (web buttons = ±0.5 × `MANUAL_VEL_MS`), `move_to` offsets are meters.
Position controller uses `flow_hold`'s dead-reckoned metric position (`pos_x_m`/`pos_y_m`, integrated from the fused velocity estimate) as feedback — not the driver's raw flow integral (which remains telemetry-only). It has two roles: **`move_to`** (one-shot move) and **position hold** (lock the current point to resist drift). In ALT_HOLD/POS_HOLD, position hold is the *default* — once airborne with good flow quality, the current x/y is captured and locked. A `move_to` runs to its target then **transitions into hold at that point** (no longer falls back to velocity=0). Web direction buttons temporarily override with manual velocity, then re-lock the new position on release. STABILIZE does not lock position (full manual).

**Task layout** (dual-core FreeRTOS):
- Core 0: WiFi protocol stack (ESP-IDF managed)
- Core 1: `main` (100Hz), `flow_rx` (blocking UART read, 100ms timeout), `p4link_rx` (UART2 read, 20ms poll — silent when P4 absent), HTTP server (event-driven)

## Key Design Decisions

- **`FLOW_ENABLED` compile flag** (top of `main/main.c`): a `#define FLOW_ENABLED 1` gates *all* optical-flow code — PV3901L1 init, the `flow_hold`/`position` updates, and the `move_to`/`move_stop`/`vel_x`/`vel_y` paths. Set to `0` to build without the flow module (e.g. when the hardware is broken); STABILIZE and ALT_HOLD still work, but POS_HOLD and all horizontal movement become no-ops.
- **Shared I2C0 bus**: `i2c_bus.c` initializes I2C0 once (`g_i2c0_bus` handle). Both MPU6050 (0x68) and TOF400F (0x29) attach via `i2c_master_bus_add_device()` — never re-init the bus. ⚠️ TOF400F 新模块出厂是**串口模式**（板上 MCU 占用 VL53L1X 总线, 0x29 无应答, 症状与模块损坏完全相同）— 需 USB-TTL 发 `01 06 00 09 00 01 98 08` 切 I2C 模式后才能用, 掉电保存一次即可; 完整流程见 README §6.1。上电严重发热 = 内部短路已损坏 (2026-07-06 实例), 直接更换。
- **Motor safety**: `MOTOR_MIN = 0.05` floor-clip (not shift-up). Throttle < 5% stops motors + resets all PIDs + resets altitude/flow/position controllers to prevent ground spooling.
- **Altitude loop**: height PID (P+I, no PID-D) plus a **vertical-speed (`vz`) damping term** that is the real fix for up/down oscillation. `vz` is computed only on *fresh* TOF samples (TOF is stair-step ~10Hz; differentiating cached frames would spike) and EMA-smoothed. **Takeoff ramps the target altitude** from the current ground height (slew-rate limited, `ALT_RAMP_RATE`) instead of stepping straight to the goal — a stepped target caused overshoot then a throttle cut to zero (crash). `vz` is exposed in telemetry as `alt.vz`.
- **TOF skip-counter**: VL53L1X data-ready checked every 10th call (@100Hz ≈ every 100ms) to match sensor timing budget; cached values returned otherwise.
- **Deferred command execution**: Blocking operations (ESC calibration, gyro recalibration) are queued via `pending_cmd` and executed in the main loop context — never from the HTTP server task. This prevents WiFi/WebSocket freeze during calibration. **Calibration/trim commands (`calibrate`, `gyro_calib`, `calibrate_motor`, `level_trim`, `reset_trim`) are rejected unless mode == DISARMED** — ESC calibration drives all motors to MAX for 6s and blocks the loop ~11s; triggering it in flight would be catastrophic.
- **Control modes**: DISARMED (default) / STABILIZE / ALT_HOLD / POS_HOLD. ALT_HOLD adds TOF height PID on top of STABILIZE; POS_HOLD adds optical-flow velocity hold on top. **Both ALT_HOLD and POS_HOLD now run the position-hold loop** (lock current x/y to resist drift, flow-quality gated). Entering an altitude mode auto-captures the target height. Position-hold feedback is `flow_hold.pos_x_m/pos_y_m` (never reset while armed) used as a relative lock reference — resetting it mid-flight would dislocate the lock point and cause a fly-away; this keeps the lock continuous across ALT_HOLD↔POS_HOLD switches. It resets only on DISARM / throttle-cut.
- **IMU+flow complementary velocity filter (metric)**: `flow_hold` is split into `flow_hold_predict()` (every 100Hz tick — EMA-filtered IMU accel, `ACCEL_EMA_ALPHA=0.3` ≈5.7Hz, integrates the velocity estimate in m/s, integrates that into `pos_x_m`, and the velocity PID runs on it) and `flow_hold_update()` (fresh flow frames only — metric conversion using TOF height and the measured frame interval, then gyro compensation in the *metric* domain (rotation-induced apparent velocity = ω × height exactly, frame-rate independent; kx/ky are dimensionless nominal ±1.0 — counts-domain constant-k comp cannot null with a measured, jittering dt_frame), then complementary-filter correction of the estimate). Both channels are m/s, so no arbitrary `imu_scale` factor exists anymore. This catches small drifts the PV3901L1 reports as zero displacement.
- **Takeoff delayed position lock**: during the takeoff climb, position hold is suppressed (`g_position_lock_pending`) because body tilt corrupts the position dead-reckoning; the velocity loop (setpoint 0, now metric so its gain no longer detunes at low altitude) keeps resisting lateral drift throughout the climb, and the position lock engages once height error < 10cm vs `target_final_m`, |vz| < 0.15 m/s, and flow qual > 30. Telemetry `flow.ps` encodes the state: 0=idle, 1=pending, 2=hold, 3=move_to.
- **WebSocket command flow**: browser sends JSON → `http_server` → `commander_parse` → updates global `setpoint_t`. Special commands: `{"cmd": "calibrate"}`, `{"cmd": "gyro_calib"}`, `{"cmd": "level_trim"}`, `{"cmd": "reset_trim"}`, `{"cmd": "calibrate_motor", "motor_index": 0}`, `{"cmd": "move_to", "x": 0.5, "y": -0.3}` (offsets in meters, clamped ±3), `{"cmd": "move_stop"}`, `{"cmd": "takeoff", "height": 0.5, "base_throttle": 0.4}`, `{"cmd": "flow_comp", "kx": -1.0, "ky": -1.0, "scale": 0.00244}` (runtime optical-flow tuning: gyro-comp direction/trim factors [dimensionless, nominal ±1] + metric scale rad/count), `{"cmd": "flow_calib", "on": 1}` (calibration mode: while DISARMED the flow estimator keeps running with motors stopped so `flow_scale` can be calibrated by hand-carrying the drone — props on, no arming; auto-cleared on disconnect/timeout/safety-reset), `{"cmd": "grip", "angle": 90}` or `{"cmd": "grip", "action": "open"|"close"}` (机械爪 MG995: angle 0–180 直接给角度 [前端调试滑条], action 用 `SERVO_GRIP_OPEN/CLOSE_DEG` 预设角; **任意模式可用** — 飞行抓取需要在空中闭爪; 非阻塞经 setpoint 下发, 主循环限速逼近; **断连/超时安全复位保留爪角度** — 强制张开会把已抓取目标掉落), `{"cmd":"grab_start","test":1}` (启动抓取任务: test=1 无 P4 测试流程 — 跳过视觉对准, 假设当前位置精确, 直接下降→TOF 触发闭爪→回升; test=0/缺省 走 P4 ALIGN 流程, 需 p4link 存活; 前置校验见 grab_mission_start), `{"cmd":"grab_abort"}` (中止任务, 回起始高度, 爪保持当前角), `{"cmd":"grab_cfg","tof":0.20,"drop":0.30}` (抓取/投放触发高度 = TOF 读数, tof 0.10–0.50 / drop 0.15–0.60 钳位, 失联复位保留 — TOF 离地安装高度 ≈0.20m, 落地读数即 0.20), `{"cmd":"mark_drop"}` (标记当前位置为投放点: 需 ALT/POS_HOLD + 光流可信 qg≥0.05; **仅同一解锁周期有效** — 航位坐标系复位时 main.c 联动 `grab_mission_clear_mark`), `{"cmd":"drop_start","ret":1}` (投放任务: ret=1 先航位推算 move_to 返航标记点再下降张爪; ret=0/缺省就地投放; 投放后爪保持张开=起落架就绪). Velocity commands (`vel_x`/`vel_y`) are sent via regular 50Hz stick data, not special commands.
- **Safety mechanisms**:
  - 500ms command timeout: if no WebSocket message received for >500ms → auto-DISARMED
  - All-clients-disconnected → `commander_reset_setpoint()` forces DISARMED + throttle=0
  - Manual motor test (`motor_active`) works **only in DISARMED** (bench test); in flight modes the motor array is ignored entirely — the old inverted logic made the web "All MAX" button a full-throttle command in flight
  - IMU failsafe: 20 consecutive `mpu6050_read` failures (~200ms) → forced DISARMED (attitude feedback untrustworthy = open-loop); on transient failures the last-good IMU sample is held instead of feeding zeros to the rate PIDs
  - TOF staleness: no fresh sample for >500ms → driver reports invalid (`tof_mm=0`) instead of serving the stale cache forever; altitude PID is skipped so ALT_HOLD degrades to flying on base stick throttle
  - Telemetry is sent from the httpd task via `httpd_queue_work` (dropped, never queued, if the previous frame is still in flight) at 20Hz — a congested WiFi link can no longer block the 100Hz control loop
  - WS handshake rejected when 4 clients already connected (an untracked 5th client could send commands without counting toward disconnect safety)
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

Control modules: `pid` (leaf), `mixer` (leaf), `commander` (includes `servo_grip.h` for grip presets), `altitude` (depends on pid), `flow_hold` (depends on pid), `position` (depends on pid — uses PID internally for position→velocity conversion), `grab_mission` (orchestrates altitude/position/servo_grip/commander — the mission layer on top of all of them; P4 measurements arrive pre-gated from main.c so control never includes communication headers).

## C Headers

All component headers are public (no `private_*.h`). Include patterns:

**Drivers:**
- `i2c_bus.h` — `i2c_bus_init()`, `g_i2c0_bus` handle
- `mpu6050.h` — `mpu6050_init()`, `mpu6050_read()`, `mpu6050_recalibrate_gyro()`, `mpu6050_data_t`
- `tof400f.h` — `tof400f_init()`, `tof400f_get_distance()`
- `pv3901l1.h` — `pv3901l1_init()`, `pv3901l1_get_data()`, `pv3901l1_reset_integral()`, `pv3901l1_set_yaw_mode()`, `pv3901l1_data_t`
- `bn880_mag.h` — `bn880_mag_init()` (probes QMC5883L@0x0D → HMC5883L@0x1E → IST8310@0x0E, logs a full I2C bus scan if all three fail; **non-fatal** on failure), `bn880_mag_read()` (gauss, module axes — unaligned/uncalibrated until the Mahony 9-axis milestone), `bn880_mag_chip_name()`, `bn880_mag_data_t`
- `motor.h` — `motor_init()`, `motor_set()`, `motor_stop()`, `motor_calibrate()`, `motor_calibrate_single()`
- `servo_grip.h` — `servo_grip_init()` (GPIO10, LEDC TIMER_1/CH4 @50Hz, 500–2500µs↔0–180°; **non-fatal** on failure), `servo_grip_set_angle()` (**驱动层硬限位 `SERVO_GRIP_MAX_DEG=90°` — 齿条行程极限, 超过空转, 台架实测 2026-07-05**), `servo_grip_open()`/`servo_grip_close()` (presets `SERVO_GRIP_OPEN_DEG=0`/`SERVO_GRIP_CLOSE_DEG=90`, 实测值; 爪兼作起落架, 上电默认张开), `servo_grip_update(dt)` (100Hz slew, 120°/s — 防电流尖峰/飞行反扭), `servo_grip_get_angle()`, `servo_grip_is_moving()`

**Estimation:**
- `attitude.h` — `attitude_init()`, `attitude_update()`, `attitude_get_euler()`

**Control:**
- `pid.h` — `pid_init()`, `pid_update()`, `pid_reset()`, `pid_t`
- `mixer.h` — `mixer_apply()`
- `commander.h` — `commander_parse()`, `commander_get_setpoint()`, `commander_reset_setpoint()`, `commander_is_command_timeout()`, `commander_clear_pending_cmd()`, `commander_mode_name()`, `setpoint_t` (includes `vel_x`, `vel_y`, `move_to_x`, `move_to_y`, `takeoff_height`, `takeoff_throttle`), `flight_mode_t`, `CMD_*` enums
- `altitude.h` — `altitude_init()`, `altitude_update()`, `altitude_capture_target()` (hold in place), `altitude_set_target()` (ramp from current height to a final target — used by takeoff), `altitude_reset()`, `altitude_ctrl_t`
- `flow_hold.h` — `flow_hold_init()`, `flow_hold_set_velocity()` (m/s), `flow_hold_set_gyro_comp()` (metric-domain direction factors, nominal ±1; effective default **+1/+1** — the source of truth is the `commander.c` setpoint defaults (`flow_kx/ky/scale`), which the main loop pushes into flow_hold every tick, so `flow_hold_init()`'s own zeros never take effect), `flow_hold_set_flow_scale()` (metric scale rad/count, default 0.00244), `flow_hold_predict()` (every 100Hz tick: IMU accel integrates `vx_est`/`vy_est` [m/s] and `pos_x_m`/`pos_y_m` [m] + runs the velocity PID), `flow_hold_update()` (fresh flow frames only: gyro compensation → metric conversion via TOF height → complementary-filter correction of the estimate — no PID here), `flow_hold_reset()` (keeps calibration values `gyro_kx/ky`, `flow_scale`), `flow_hold_is_active()`, `flow_hold_t`
- `position.h` — `position_init()`, `position_set_target()` (move_to, one-shot, offsets in meters), `position_hold_start()` (lock current point, persistent — does not exit on reach), `position_update()` (also debounces move_to arrival: 10 consecutive in-tolerance ticks), `position_reset()`, `position_reached()`, `position_ctrl_t` (has `hold` flag distinguishing the two)
- `grab_mission.h` — 抓取/投放任务状态机: `grab_mission_init()`, `grab_mission_start()` (抓取; 前置校验: ALT/POS_HOLD + TOF 有效且高于触发高度+10cm + 爪已张开 + 非测试需 P4 存活), `grab_mission_start_drop()` (投放; use_goto=返航到标记点, 需标记有效+光流可信), `grab_mission_mark_drop()` (标记投放点, 存 flow 航位坐标), `grab_mission_clear_mark()` (**必须与 flow_hold_reset 同点调用** — 坐标系复位标记即失效, main.c 两处已联动), `grab_mission_abort()` (回起始高度, **爪保持当前角**), `grab_mission_update()` (100Hz 推进; 内置全局中止: 模式退出/油门切断/TOF 失效/120s 总超时), `grab_mission_active()`, `grab_state_t` (IDLE/ALIGN/DESCEND/GRASP/ASCEND/GOTO/RELEASE — 遥测 `grab.st` 0–6), `grab_mission_kind_t` (GRAB/DROP — DESCEND 触发高度与爪动作按类型分叉), `grab_meas_t` (已过安全门的视觉测量, main.c 门控后传入)

**Communication:**
- `wifi_ap.h` — `wifi_ap_init()`
- `http_server.h` — `http_server_init()`, `http_server_set_command_cb()`, `http_server_set_disconnect_cb()`, `http_server_broadcast()`
- `web_page.h` — embedded HTML/CSS/JS frontend (single `const char *`)
- `p4link_protocol.h` — S3↔P4 串口协议 v1 唯一权威定义 (frame format, `p4link_state_t`/`p4link_target_t` packed structs, safety-gate defaults, CRC-16/CCITT-FALSE reference impl; P4 工程持同一份拷贝; 设计详见 DESIGN.md §3.10 / `P4LINK_PROTOCOL.md`)
- `p4link.h` — S3 侧链路驱动: `p4link_init()` (UART2 TX=GPIO17/RX=GPIO18 @115200 + `p4link_rx` 任务; **non-fatal**), `p4link_send_state()` (非阻塞 `uart_tx_chars`, FIFO 满丢帧), `p4link_take_target()` (take 语义: 每帧只交付一次; **原始值, 安全门控在 main.c**), `p4link_alive()` (500ms 内有 CRC 正确帧), `p4link_get_stats()`

## Coding Conventions

- **Headers**: always `#pragma once` + `extern "C"` block (all headers are C++-safe)
- **Return values**: 0 = success, -1 = failure (ESP-IDF convention)
- **Globals**: `g_` prefix for file-static/module-level variables (e.g. `g_i2c0_bus`, `g_trim_roll`)
- **Logging**: `static const char *TAG = "module"` then `ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`(TAG, ...)
- **Timing**: `vTaskDelayUntil()` fixed 10ms cadence for the 100Hz loop, with `dt` measured each iteration via `esp_timer_get_time()` (clamped 5–30ms) and fed to all integrators — never hard-code dt; `esp_timer_get_time()` also for variable-rate updates (flow frame interval) and command timeout
- **JSON**: telemetry uses raw `snprintf` (not cJSON) — buffer is 768 bytes; commands use cJSON for parsing

## Init Ordering

Init order in `app_main()` is critical (I2C must come first, HTTP must come last):

```
i2c_bus_init → mpu6050_init → tof400f_init → bn880_mag_init (non-fatal) → pv3901l1_init
  → motor_init → attitude_init → PID init → altitude_init → flow_hold_init → position_init
  → wifi_ap_init → http_server_init
```

WiFi starts after motors so calibration doesn't conflict with the WiFi stack. HTTP server needs WiFi up + commander callback + disconnect callback registered before starting.

## Concurrency

- **setpoint_t** is shared between HTTP server task (writer) and main loop (reader) — no mutex
- Commander parses JSON into a local temp, then struct-assigns to `g_sp` (narrowed race window vs. per-field writes)
- Blocking commands (calibrate, gyro_calib, move_to, move_stop, takeoff) are deferred via `pending_cmd`: HTTP task sets the flag, main loop executes it next iteration
- **500ms timeout**: `g_last_command_us` timestamp updated on each valid WebSocket parse; main loop checks `commander_is_command_timeout()` → auto-DISARMED
- **Disconnect safety**: when all WS clients disconnect, `ws_disconnect_cb` fires `commander_reset_setpoint()` — cleans both `g_sp` and the timestamp
- `pv3901l1_data_t` is filled by the `flow_rx` FreeRTOS task (Core 1, priority 10, 100ms UART poll), consumed by main loop
- **WS broadcast**: main loop copies JSON into `g_bcast_buf` and queues `bcast_work` via `httpd_queue_work`; all `g_ws_fds` mutation (connect, close, send-failure removal) happens in the httpd task — no cross-task race on the fd list. `g_bcast_inflight` makes the main loop drop frames instead of ever waiting
