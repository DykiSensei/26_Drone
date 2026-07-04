# ESP32-S3 Drone — 项目大纲与设计文档

## 1. 项目概述

基于微雪 ESP32-S3-DEV-KIT-N16R8 开发板的四轴无人机飞控项目。采用 ESP-IDF 5.5.4 框架，参考 Crazyflie/esp-drone 飞控架构，利用板载 WiFi 实现遥测与遥控。

### 1.1 硬件清单

| 模块 | 型号 | 接口 | 地址/参数 | 用途 |
|------|------|------|-----------|------|
| 主控 | ESP32-S3-DEV-KIT-N16R8 | — | 16MB Flash / 8MB PSRAM, 240MHz 双核 | 飞控核心 |
| IMU | MPU6050 | I2C | 0x68 | 姿态估计（6轴：3陀螺仪+3加速度计） |
| 激光测距 | TOF400F (VL53L1X) | I2C | 0x29 (VL53L1X 直通) | 定高（4cm–4m） |
| 光流 | PV3901L1 | UART + GPIO | RX=GPIO44, YAW_MODE=GPIO15, 波特率 19200 | 水平位置悬停 |
| 电机×4 | 无刷电机 + 电调 | PWM | 4路 LEDC | 动力输出 |

### 1.2 总线分配

```
I2C0 (共用总线):
  ├── MPU6050   SDA=GPIO9  SCL=GPIO8  (I2C addr: 0x68, 400kHz)
  └── TOF400F   SDA=GPIO9  SCL=GPIO8  (I2C addr: 0x29, 400kHz, VL53L1X 直连)

UART1:
  └── PV3901L1  RX=GPIO44               (光流数据接收，模块仅TX端)

GPIO:
  └── YAW_MODE  GPIO15                 (光流偏航模式选择)

LEDC PWM (4通道):
  ├── M0 (FR): GPIO14
  ├── M1 (FL): GPIO11
  ├── M2 (RL): GPIO13
  └── M3 (RR): GPIO12

System:
  └── RGB LED: GPIO48  (状态指示，WS2812)
```

---

> I2C0 总线由 `i2c_bus.c` 统一初始化（`g_i2c0_bus` 全局句柄），各驱动通过 `i2c_master_bus_add_device()` 挂载设备。

## 2. 软件架构

### 2.1 分层结构

```
┌──────────────────────────────────┐
│        应用层 (Application)       │  ← 飞行模式、遥控指令
├──────────────────────────────────┤
│       飞控核心 (Flight Core)      │  ← 姿态估计、PID控制、高度/位置控制
├──────────────────────────────────┤
│      硬件抽象层 (HAL / Drivers)   │  ← 传感器驱动、电机驱动、通信
├──────────────────────────────────┤
│     ESP-IDF 5.5 + FreeRTOS       │  ← 实时操作系统、外设驱动
├──────────────────────────────────┤
│     ESP32-S3 硬件                 │
└──────────────────────────────────┘
```

### 2.2 FreeRTOS 任务规划

ESP32-S3 双核分配：

| 核心 | 任务 | 频率 | 优先级 | 说明 |
|------|------|------|--------|------|
| Core 1 | **flow_rx** | 事件驱动 | 10 | 光流 UART 数据解析（FreeRTOS 任务，100ms 轮询） |
| Core 1 | **http_server** | 事件驱动 | 5 | HTTP + WebSocket（ESP-IDF 自动创建） |
| Core 1 | **main** | 100 Hz | 1 | 传感器读取 → 姿态计算 → 控制（遥测每 5 拍降至 20Hz，经 httpd 任务异步发出） |
| Core 0 | **WiFi** | — | — | WiFi 协议栈（ESP-IDF 自动创建） |

> 注：当前主循环为 100Hz 简单轮询。控制链路（Mahony → PID → Mixer → Motor）已完整集成，未来 Phase 2 将重构为 1000Hz 稳定器中断驱动架构。

### 2.3 数据流

```
                    ┌─────────────────┐
MPU6050 (I2C) ─────→│                 │
TOF400F (I2C) ─────→│   Main Loop     │────→ 4×PWM → 电机
PV3901L1 (UART) ──→│   100Hz         │
                    │                 │
Commander ─────────→│                 │────→ Telemetry (WiFi)
                    └─────────────────┘
```

控制链路（100Hz）：
```
MPU6050 → Mahony AHRS → 欧拉角 (roll/pitch/yaw)
                          ↓
Commander setpoint → 模式判断:
  所有模式:  摇杆 → 目标倾角 (±30°) → + 光流修正 (±5°) → 角度环 P → 目标角速率
  偏航:      摇杆 → 目标角速率 (±3 rad/s)
                          ↓
                    PID (rate) → 力矩输出
                          ↓
                    Mixer (X-quad, MOTOR_MIN floor) → motor[0..3] → LEDC PWM
```

---

## 3. 模块设计

### 3.1 传感器驱动层

#### MPU6050 驱动 `drivers/mpu6050.h`
- I2C0 设备地址 0x68, 400kHz，先 probe 再注册，WHO_AM_I 5 次重试
- 配置：陀螺仪 ±2000°/s, 加速度计 ±16g, **DLPF=4（~21Hz BW）**, 采样率 8kHz (gyro) / 1kHz (accel)
  - DLPF=4 是为 IMU+光流互补滤波准备的：原 DLPF=0 (256Hz BW) 让电机振动直接进入 accel → 积分到 vx_est 形成假漂移，DLPF=4 把振动频带（>50Hz）滤掉
- 上电自动校准：采样 500 次求零偏（陀螺仪 xyz + 加速度 xy）
- 输出单位：加速度 m/s², 角速度 rad/s, 已扣除零偏
- **起飞前再校准**：`mpu6050_recalibrate_gyro()`，采样 100 次（1 秒），**同步重校准陀螺仪零偏和加速度计 x/y 零偏**。加速度计以当前放置面为"水平"基准，消除上电面与起飞面不一致导致的姿态偏差。校准后自动调用 `attitude_init()` 复位 Mahony 滤波器（防止旧积分项使用过期零偏导致漂移），约 2 秒内重新收敛到真实姿态

#### TOF400F 驱动 `drivers/tof400f.h`
- I2C0 (SDA=GPIO9, SCL=GPIO8, 400kHz)，设备地址 0x29
- TOF400F 模块在 I2C 模式下 MCU 释放总线，VL53L1X 传感器直接暴露在 I2C 总线上
- 使用 **ST VL53L1X Ultra Lite Driver (ULD)** 原生 I2C 寄存器协议（16-bit 寄存器地址）
- 初始化：写入 91 字节默认配置 → VHV 校准 → 设置距离模式 LONG (4m) + 100ms Timing Budget → 启动连续测距
- 主循环优化：skip-counter 每 10 次调用（@100Hz ≈ 100ms）才检查 VL53L1X 数据就绪，其余返回缓存值
- 读取 RESULT__FINAL_CROSSTALK_CORRECTED_RANGE (0x0096) 获取串扰校正后距离
- 有效范围 40–4000mm，保留上次有效值作为回退

#### PV3901L1 光流驱动 `drivers/pv3901l1.h`
- UART1 RX=GPIO44, 波特率 19200（模块仅TX端，ESP仅需接收）
- YAW_MODE GPIO15：输出高低电平切换光流偏航模式
- 数据包解析：帧头 0xFE 0x04，9字节包，和校验 + 结束符双重校验
- **安装方向映射（parse_byte 内，机体系唯一转换点）**：本机模块安装旋转了 90°（2026-07-04 实测：前移→raw_y 正、左移→raw_x 正），驱动内映射为机体系 `flow_x = raw_y`（前正）、`flow_y = −raw_x`（右正）。下游全部消费机体系值；换装模块后只改这两行并重验陀螺补偿符号
- 提取 flow_x, flow_y, qual（质量）——均为机体系
- 积分位移：flow_x_i += flow_x, flow_y_i += flow_y（双轴积分，遥测/调试用，控制反馈已改用 flow_hold 航位推算）

### 3.2 姿态估计 `estimation/attitude.h`

采用 **Mahony 互补滤波器**：

- 输入：陀螺仪角速度 (rad/s) + 加速度计重力向量 (m/s²)
- 输出：四元数 → 欧拉角 (roll, pitch, yaw)
- 更新频率：100Hz（主循环频率）
- 参数：`MAHONY_KP = 2.0f`, `MAHONY_KI = 0.005f`

```
q' = q + Δt * f(q, gyro, accel)    // Mahony更新
roll  = atan2(2*(q0*q1+q2*q3), 1-2*(q1²+q2²)) * 180/PI
pitch = asin(2*(q0*q2-q3*q1)) * 180/PI
yaw   = atan2(2*(q0*q3+q1*q2), 1-2*(q2²+q3²)) * 180/PI
```

### 3.3 PID 控制器 `control/pid.h`

当前实现为 **角度环 + 角速率环串级（Angle + Rate）**：

**自稳模式 (STABILIZE)**：
```
摇杆输入 (±1.0)
    ↓
目标姿态角 (±30°)
    ↓
角度环 P 控制 ──→ 目标角速率 (rad/s)
    ↓
角速率环 PID ──→ 力矩输出 ──→ 混控矩阵 ──→ Motor[1..4]
    ↑
实际角速率 (gyro)
```

- `pid_init(&pid, kp, ki, kd, output_limit, integral_limit)`
- `pid_update(&pid, setpoint, measurement, dt)` — 采用 **derivative on measurement** 避免 setpoint kick
- 当前调优参数：
  - roll/pitch rate PID：`Kp=0.25, Ki=0.02, Kd=0.01`，输出限幅 ±0.8，积分限幅 0.15
  - yaw rate PID：`Kp=0.8, Ki=0.05, Kd=0.0`，输出限幅 ±0.5，积分限幅 0.15
- 角度环参数（STABILIZE 模式）：
  - `MAX_ANGLE_DEG = 30°` — 最大倾斜角
  - `ANGLE_KP = 6.0 * 0.017453293f` (≈ 0.105 rad/s 每度误差) — 纯 P 控制，无积分
- **水平修正 (Level Trim)**：补偿 MPU6050 安装偏移
  - `CMD_LEVEL_TRIM` 指令 → `g_trim_roll/g_trim_pitch` 捕获当前 roll/pitch 作为零位
  - 角度环计算时扣除：`roll_err = roll - g_trim_roll`
- 低油门 (< 5%) 时停转 + PID 复位，防止地面翘机
- 混控 `MIXER_SCALE = 0.2f`，电机最低怠速 5%（`MOTOR_MIN`）

**定高/悬停模式**：ALT_HOLD / POS_HOLD 均使用角度外环（同 STABILIZE）+ 定高 PID（TOF 反馈）。POS_HOLD 额外启用光流速度保持 + 位置控制。

### 3.3.1 定高控制器 `control/altitude.h`

- `altitude_ctrl_t`：封装 altitude PID + 当前(斜坡)目标 `target_m` + 最终目标 `target_final_m` + 垂直速度估计 `vz`
- `altitude_init()`：初始化 PID（**Kp=0.25, Ki=0.02, Kd=0.0**，输出限幅 ±0.3 油门，积分限幅 0.15）
- `altitude_capture_target(current_m)`：原地保持，最终目标=当前高度（斜坡不动），定高模式切入时用
- `altitude_set_target(final_m, current_m)`：从当前高度**斜坡爬升**到 final_m（起飞用），目标按 `ALT_RAMP_RATE=0.3 m/s` 缓升，避免阶跃过冲
- `altitude_update(current_m, dt)`：推进目标斜坡 → 运行 P+I → 叠加 vz 阻尼，返回油门修正量
- `altitude_reset()`：清零积分和 vz（DISARMED 或低油门时调用）
- **垂直速度阻尼**：`output = (P+I) − ALT_KD_VZ·vz`（`ALT_KD_VZ=0.5`）。`vz` 仅在 TOF 有新值时按真实间隔差分计算（TOF 是 ~10Hz 阶梯数据，对缓存帧求导会爆尖峰），并经 EMA 平滑（`VZ_SMOOTH=0.5`）。这是抑制上下摇摆的关键项
- 有效油门 = 摇杆基准 + 高度修正（钳位 0.0–1.0）
- 调参背景：原 Kp=0.5/Ki=0.05/无阻尼时大幅上下摇摆（欠阻尼振荡）；降增益 + vz 阻尼后收敛。`vz` 通过遥测 `alt.vz` 暴露用于调试

### 3.3.2 光流速度保持控制器 `control/flow_hold.h`（IMU + 光流互补滤波）

在 STABILIZE / ALT_HOLD / POS_HOLD 模式下叠加水平速度修正，主动抵抗漂移，实现垂直起飞。

**核心改动一（互补滤波）**：旧实现直接用裸光流速度跑 PID。PV3901L1 模块有内部死区（连续两帧位移太小直接输出 0），导致小漂移检测不到 → 等漂大了才介入 → 锁不住。新实现引入 **IMU 加速度积分 + 光流测量互补滤波**。

**核心改动二（全链路米制化）**：光流物理特性是 `counts ∝ 水平速度 / 高度`，直接消费原始 counts 会让等效环增益随高度漂移 5~10 倍——定点只在调参高度稳、起飞爬升期（高度剧变）必然失配，这是"定点悬停不稳/无法垂直起飞"的主根因。现在 update 阶段按 TOF 高度换算成米制速度：`v(m/s) = counts × flow_scale(rad/count) × height(m) / dt_frame(s)`，`flow_scale` 默认 0.00244（PMW3901 系光学参数 4.2°FOV/30px），经 `{"cmd":"flow_comp","scale":..}` 试飞标定。IMU 加速度（m/s²）与光流速度（m/s）量纲天然一致，旧的经验系数 `imu_scale` 随之删除。

- `flow_hold_t`：封装两个速度 PID + **vx_est/vy_est 互补滤波速度状态 (m/s)** + **pos_x_m/pos_y_m 航位推算位置 (m)** + 输出修正角
- `flow_hold_init()`：初始化 PID（Kp=8.0 deg/(m/s), Ki=1.2, Kd=0.0，输出限幅 ±8°，积分状态限幅 3.0 m/s·s ≈ 3.6° 权限）

#### predict（100Hz，每帧调用）
```c
flow_hold_predict(fh, ax_world, ay_world, dt)
```
- IMU 加速度积分 + 慢衰减：`vx_est = (vx_est + ax·dt) * IMU_LEAK`（`IMU_LEAK=0.999`, τ≈10s @100Hz，防 accel 偏置积爆）
- 航位推算：`pos_x_m += vx_est·dt` —— position 环的位置反馈源（米制、含 IMU 高频信息、低质量时速度自然衰减 → 位置冻结而非积累噪声）
- 输入是经过 **DLPF=4 硬件低通 + 软件 EMA（α=0.3, ~5.7Hz）** 双层滤波的 accel_x/y，振动噪声不会污染 vx_est
- 跑 PID（100Hz，比旧 50Hz 多一倍带宽）：`out_pitch_deg = pid_update(pid_vx, setpoint_vx, vx_est, dt) * quality_gain`
- 低 quality 时（`quality_gain < FLOW_QUALITY_FREEZE_I=0.5`）冻结 PID 积分避免 windup

#### update（仅在新光流帧调用）
```c
flow_hold_update(fh, flow_x, flow_y, gyro_x, gyro_y, qual, height_m)
```
- **陀螺补偿**（关键，**米制域**）：`vx −= kx·gyro_y·h`、`vy −= ky·gyro_x·h`。旋转引起的视速度恒等于 ω×高度（小角度精确、与帧率无关），kx/ky 是无量纲方向/微调系数，标称 ±1.0。PV3901L1 无内部补偿，不做会导致纠偏与姿态动作耦合 → 悬停晃动、定点漂走。标定 = 定符号：标定模式下手持 ~0.5m 缓慢倾斜（≤15°、不平移），`flow.cx/cy` 跟着摆动就翻转对应 K 的符号。⚠️ 不能在 counts 域用常数 k 补偿——旋转 counts = ω·dt_frame/scale，实测 dt_frame 逐帧抖动会让任何常数 k 都无法恒零（历史教训 2026-07-04）
- **米制换算**：`v = fx × flow_scale × height_m / dt_frame`，帧间隔 dt_frame 用 `esp_timer_get_time()` 实测（钳位 5~50ms），不假定模块帧率恒定
- **速度 EMA 平滑 + 死区**（`FLOW_VEL_SMOOTH=0.3`, `FLOW_DEADBAND=0.02 m/s`）：先 EMA 平滑；死区清零静止小信号防"追假速度"
- **互补滤波修正**：`vx_est += FLOW_CORRECT_K · quality_gain · (fxd − vx_est)`（`FLOW_CORRECT_K=0.30`），新光流把 vx_est 拉向测量值的比例由 quality_gain 缩放
- 即使模块输出 0（连续两帧位移太小），fxd 就是 0 → vx_est 会被拉向 0（慢衰减），同时 IMU 积分仍在 predict 推进 → 短时小漂移仍可被捕捉
- 质量门控：**连续 quality 权重**（`FLOW_QUALITY_LOW=30` 起步、`FLOW_QUALITY_HIGH=80` 满权，线性插值），EMA 平滑（`FLOW_QUALITY_SMOOTH=0.3`）。原 binary 50 切断在 qual 30-50 徘徊时完全无位置控制 → 漂走才锁
- 高度门控：0.04m < height < 3.0m

#### 其他
- `flow_hold_set_velocity(vx, vy)`：设置速度指令（m/s），0=静止保持，非零=主动移动
- `flow_hold_set_gyro_comp(kx, ky)`：设置陀螺补偿方向系数（米制域无量纲，标称 ±1，默认 -1/-1，运行时标定符号）
- `flow_hold_set_flow_scale(scale)`：米制换算系数 rad/count（运行时试飞标定，默认 0.00244）
- `flow_hold_reset()`：DISARMED / 低油门时清零 PID、vx_est/vy_est、pos_x_m/pos_y_m（标定值 kx/ky/scale 保留）
- `flow_hold_is_active()`：`quality_gain > 0.01` 时返回 true
- 修正叠加在摇杆目标角度上（±30° + ±8°），不影响飞行员操控权限

```
                ┌─ predict @100Hz (IMU 高频快通道) ──┐
ax/ay (DLPF+EMA)→│ vx_est += a·dt (m/s)             │── PID → 修正角 ±8°
                │ vx_est *= IMU_LEAK               │       ↓
                │ pos_x_m += vx_est·dt (position 反馈)│  叠加到 stick 目标角度
                └────────────────┬───────────────────┘   → Angle P → Rate PID → Mixer
                                 │
                 ┌─ update @新光流帧 (低频绝对参考) ──────────────┐
flow_x/y, gyro → │ gyro 补偿 → ×scale×height/dt 米制 → EMA → 死区 │
                │ vx_est += K·quality·(fxd−vx_est)              │
                └───────────────────────────────────────────────┘
```

### 3.3.3 水平移动控制（Web 按钮 / P4 API）

#### 速度指令（Web 前端方向按钮）

Web 前端新增 4 个方向按钮（▲前/▼后/◀左/▶右），按住移动、松开停止：

- 按钮事件：`mousedown`/`touchstart` → `vel_x`/`vel_y` = ±0.5，`mouseup`/`touchend` → 清零
- STOP 按钮：立即清零 + 发送 `move_stop` 命令
- 速度值通过 50Hz 摇杆数据流发送：`{"vel_x": 0.5, "vel_y": 0.0}`
- `commander_parse()` 解析 `vel_x`/`vel_y` 字段，钳位 -1.0~1.0
- 主循环将归一化速度映射为米制速度（×`MANUAL_VEL_MS`=0.6，按钮 ±0.5 → ±0.3 m/s）直接送入 flow_hold PID（本机实测 前/右 = 光流正方向，无需取反；修正角反号统一在 flow_hold_predict 输出处理）
- 在所有非 DISARMED 模式下生效，有光流质量门控

#### 位置指令 `move_to`（P4 视觉对接）

P4 检测到垃圾后发送相对位置偏移指令，飞控通过航位推算位置闭环移动：

```json
{"cmd": "move_to", "x": 0.5, "y": -0.3}
```

- x: 前向偏移（**米**，钳位 ±3），y: 右向偏移（米）
- 作为延迟命令（`pending_cmd`），在主循环中执行
- 启动时捕获当前航位推算位置（flow_hold.pos_x_m/pos_y_m）作为起点，目标 = 起点 + 偏移
- 位置 PID 输出速度指令 (m/s) → flow_hold 速度环 → 姿态修正 → 电机

#### 停止指令 `move_stop`

```json
{"cmd": "move_stop"}
```

复位 move_to 位置控制器；在 ALT_HOLD/POS_HOLD 下，下一循环会自动在当前点重新进入位置保持。

```
控制链:
  Web按钮 → vel_x/vel_y → 速度映射 → flow_hold 速度环 ┐
  P4 move_to → position 位置环 → 速度指令 → flow_hold 速度环 ┘
                                                          ↓
                                              flow_hold PID → 姿态修正角 ±5°
                                                          ↓
                                              叠加摇杆 → Angle P → Rate PID → 电机
```

### 3.3.4 位置控制器 `control/position.h`

用于 P4 的自主位置控制：

- `position_ctrl_t`：封装两个位置 PID + 目标位置 (m) + `hold` 标志（区分持续保持 vs 一次性 move_to）
- `position_init()`：初始化 PID（Kp=1.5 /s —— 10cm 误差 → 0.15 m/s 拉回，Ki=0.15, Kd=0.0，输出限幅 ±0.5 m/s，积分状态限幅 1.0 m·s）。米制化后 Kp 量纲是 1/s，含义不随高度漂移
- `position_set_target(offset_x, offset_y, current_x_m, current_y_m)`：**move_to** 一次性移动，目标 = 当前位置 + 偏移（米，hold=false）
- `position_hold_start(current_x_m, current_y_m)`：**位置保持**，锁定当前点（hold=true），不因到达而退出，用于对抗漂移
- `position_update(x_m, y_m, dt)`：运行位置 PID，输出速度指令 (m/s)；move_to 时顺带做到达防抖计数
- `position_reached()`：连续 10 个周期位置误差 < 5cm 才判定到达（防单帧噪声擦线触发）
- `position_reset()`：停用并清零所有状态
- 位置反馈源是 `flow_hold` 的航位推算 `pos_x_m/pos_y_m`（融合速度积分），不再直接消费驱动的裸光流积分（后者仅保留在驱动内部）
- **默认位置保持**：ALT_HOLD/POS_HOLD 下离地且光流质量达标（qual>30）即自动 `position_hold_start` 锁定当前点；move_to 到达后**转入该点位置保持**（不再回退到 velocity=0）；Web 速度按钮临时接管、松手后在新位置重新锁定
- **takeoff 期间延迟锁定**：上升阶段机身倾斜让位置推算带噪声 → 假漂移 → position 误判漂移而输出 vel_cmd 干扰起飞。`g_position_lock_pending` 标志在 takeoff 时置位，跳过默认锁定分支；爬升全程由速度环（setpoint=0，米制化后低空增益不再失配）压制水平漂移；高度环判定 `|current − target_final| < 10cm && |vz| < 0.15 m/s && qual > 30` 三者全满足时再启动位置锁定

未来扩展（Phase 3）：

| 控制器 | 频率 | 输入 | 输出 |
|--------|------|------|------|
| 角度环 (P) | 1000Hz | roll/pitch/yaw 期望角 | 期望角速率 |
| 角速率环 (PID) | 1000Hz | 期望角速率 - 实际角速率 | 力矩输出 |
| 高度环 (PID) | 100Hz | 期望高度 - TOF距离 | 油门补偿 ✅ 已实现 |
| 水平速度环 (PID) | 100Hz | 光流速度 (setpoint=0) | 姿态修正角 ✅ 已实现 |
| 水平位置环 (PID) | 100Hz | 期望位置 - 航位推算位置 (m) | 期望速度 (m/s) ✅ 已实现 |

### 3.4 混控器 `control/mixer.h`

X字型四轴混控：

```
Motor[0] = throttle - roll + pitch + yaw   // 前右 (FR, CCW)
Motor[1] = throttle + roll + pitch - yaw   // 前左 (FL, CW)
Motor[2] = throttle + roll - pitch + yaw   // 后左 (RL, CCW)
Motor[3] = throttle - roll - pitch - yaw   // 后右 (RR, CW)
```

- `mixer_apply(throttle, roll, pitch, yaw, motor[4])`
- `MIXER_SCALE = 0.2f`（roll/pitch 缩放），`MIXER_SCALE_YAW = 0.4f`（yaw 需要更大权限对抗扭矩不平衡）
- **Roll 取反**：`r = -roll * MIXER_SCALE`（匹配实际布线方向）
- Yaw 不再取反（电机实际转向与标准 X-quad 相反，经测试验证直接正逻辑）
- **MOTOR_MIN 地板 + 顶缩放**（非 shift-up，防止低油门安全隐患）：
  1. 每个电机输出钳位到 `[MOTOR_MIN, 1.0]`（MOTOR_MIN = 0.05）
  2. 若 max(motor) > 1.0，等比例缩放使最大值为 1.0
- 输出最终钳位 0.0–1.0

### 3.5 电机驱动 `drivers/motor.h`

- 4通道 LEDC PWM，频率 400Hz（标准电调PWM频率）
- 占空比范围：1000μs–2000μs（对应 0%–100% 油门）
- 分辨率 13-bit
- GPIO 分配：M0(FR)=GPIO14, M1(FL)=GPIO11, M2(RL)=GPIO13, M3(RR)=GPIO12
- 支持 ESC 油门校准：`motor_calibrate()`（先输出 MAX 2000μs 进入校准模式，再输出 MIN 1000μs 确认），耗时约 12 秒
- 支持单电机校准：`motor_calibrate_single(index)`（仅对指定电机执行校准序列，用于更换电调后单独校准）

### 3.6 通信模块 `communication/`

#### WiFi 配置
- ESP32-S3 作为 AP 模式
- SSID: `Drone-XXXX`（XXXX = MAC 后 4 位），密码 `12345678`
- 固定 IP: `192.168.4.1`，内置 DHCP 服务器

#### HTTP + WebSocket 服务器
- HTTP 端口 80：`GET /` 返回嵌入式 Web 前端（单文件 HTML/CSS/JS）
- WebSocket `/ws`：双向 JSON 通信，最多 4 客户端（**满员拒绝握手**：不入列表的客户端能发命令却不参与断连安全统计）
- 帧类型处理：PING 自动回复 PONG，CLOSE 正常断开，非 TEXT 帧忽略
- 命令回调模式：`http_server_set_command_cb(commander_parse)`
- **断连安全回调**：`http_server_set_disconnect_cb()` 注册回调，当 `g_ws_count` 降至 0（所有客户端断开）时触发 `commander_reset_setpoint()` 强制 DISARMED。在两处触发：`ws_close_handler`（TCP 断开）和 `bcast_work`（异步发送失败清理）
- **异步广播**：`http_server_broadcast()` 只拷贝 JSON 到缓冲并 `httpd_queue_work()`，真正的 socket 发送在 httpd 任务里做——`httpd_ws_send_frame_async` 实际同步写 socket，若在主循环里直接调，WiFi 拥塞时会阻塞控制循环（电机保持旧 PWM，等效失控）。上一帧未发完则丢弃新帧，绝不等待；`send_wait_timeout=1s` 限制卡死客户端的影响。fd 列表的增删（连接/断开/发送失败）全部收敛到 httpd 任务，消除跨任务竞态

#### 遥测数据（ESP → 浏览器，20Hz）
```json
{
  "accel": [x, y, z],
  "gyro": [x, y, z],
  "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
  "tof": 1234,
  "alt": {"target": 1.20, "out": 0.015, "vz": 0.00},
  "flow": {"x": 0.0, "y": 0.0, "qual": 0, "cr": 0.0, "cp": 0.0},
  "motor": [0.0, 0.0, 0.0, 0.0],
  "mtrim": [0.0, 0.0, 0.0, 0.0],
  "pid": [0.0, 0.0, 0.0],
  "trim": {"roll": 0.0, "pitch": 0.0},
  "mode": "stabilize"
}
```

> 注：`battery` 字段前端已预留 UI，但 ADC 读取尚未实现，暂不回传。`trim` 字段显示当前水平修正量。

#### 遥控指令（浏览器 → ESP，50Hz）
```json
{
  "throttle": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
  "vel_x": 0.0, "vel_y": 0.0,
  "mode": "stabilize"
}
```

特殊命令：
```json
{"cmd": "calibrate"}         // ESC 油门校准（四路）
{"cmd": "gyro_calib"}        // 陀螺仪零偏再校准
{"cmd": "level_trim"}        // 水平校准：捕获当前姿态角作为水平零位
{"cmd": "reset_trim"}        // 重置水平修正量为零
{"cmd": "calibrate_motor", "motor_index": 0}  // 单电机电调校准 (0=FR,1=FL,2=RL,3=RR)
{"cmd": "move_to", "x": 0.5, "y": -0.3}  // P4 位置偏移指令（米，钳位 ±3）
{"cmd": "move_stop"}         // 停止所有水平移动
{"cmd": "takeoff", "height": 0.5, "base_throttle": 0.4}  // 自动起飞（高度0.2~2.0m，油门0.25~0.6）
```

#### Web 前端 `web_page.h`
- **油门控制**：HTML range 滑块（0–100%），直观易用
- **Roll/Pitch 控制**：Canvas 虚拟摇杆（360° 模拟量）
- **Yaw 控制**：底部 range 滑块（-1.0 ~ 1.0）
- **方向键**：▲▼◀▶ 短按 ±0.5 快捷控制 roll/pitch（onmousedown/ontouchstart）
- **模式按钮**：锁定 / 自稳 / 定高 / 悬停
  - `click` + `touchstart` 双事件绑定，解决移动端模式切换问题
  - 间隔发送 `sendStick()`（不含 mode），按钮点击调用 `send()`（含 mode），消除遥控反馈回环
- **校准按钮**：陀螺仪校准、水平校准、重置水平
- **自动起飞面板**：目标高度滑块（0.2–2.0m）+ 基准油门滑块（0.25–0.6）+ 起飞按钮
- **单电机校准**：每路电机独立校准按钮（拆桨安全确认）
- **逐电机微调**（Mtrim）：每路电机 ± 按钮（范围 ±0.15），补偿硬件个体差异
- **实时传感器数据面板**：姿态数值（含水平修正量显示）、IMU 原始数据、TOF 高度、光流位置、PID 输出
- **电机 PWM 实时显示**（M1–M4 百分比）+ All MAX/MIN/STOP 快捷按钮
- 深色主题、移动优先布局、触摸友好
- WebSocket 自动重连机制（2 秒间隔），**重连时复位所有控制变量**（throttle/roll/pitch/yaw → 0, mode → disarmed, motorPWM → 1000μs, motorTrim → 0），UI 同步复位，并发送 `send()` 同步 DISARMED 到 ESP，防止重连后发送残留的飞行参数

### 3.7 逐电机微调 Mtrim

- Web 前端为每个电机提供 ± 按钮，范围 ±0.15
- 混控输出后叠加：`m[i] += sp->mtrim[i]`
- 用途：补偿电机/电调/螺旋桨个体差异导致的推力不平衡
- 微调值通过 WebSocket 随摇杆数据一起发送（`mtrim` 字段）

### 3.8 飞行模式 `control/`

#### Commander `commander.h/.c`
- 解析 WebSocket JSON 遥控指令（cJSON）
- 维护全局 `setpoint_t` 结构体（throttle, roll, pitch, yaw, mode, motor[], motor_active, mtrim[]）
- 值域钳位，字符串模式名映射到枚举
- 支持手动单电机控制（`motor[]` 数组，用于调试/测试）
  - `motor_active` 仅当 motor[] 任一值 > 0 时置 true，否则飞行模式正常接管
- 特殊命令（延迟到主循环执行，避免阻塞 HTTP Server 任务）：
  - `CMD_CALIBRATE` — 四路电调油门校准
  - `CMD_GYRO_CALIB` — 陀螺仪零偏再校准
  - `CMD_LEVEL_TRIM` — 捕获当前姿态角作为水平零位
  - `CMD_RESET_TRIM` — 重置水平修正量为零
  - `CMD_CALIBRATE_MOTOR` — 单电机电调校准（带 `motor_index` 参数）
  - `CMD_TAKEOFF` — 自动起飞（设定目标高度，切入 ALT_HOLD，设定基准油门）
- 命令通过 `pending_cmd` 字段延迟到主循环执行，确保阻塞操作（校准）不冻结 HTTP/WebSocket 通信
- **takeoff 命令原子化**：CMD_TAKEOFF 在 `commander_parse()` 中**同时设置** `mode=MODE_ALT_HOLD, throttle=takeoff_throttle, yaw=0`（连同 `pending_cmd` 一起一次 struct 赋值）。修复了前端发送 `takeoff` 与 `setMode` 是两包 WebSocket 消息、之间主循环跑了 DISARMED 分支调 `altitude_reset` 把目标清零的竞态
- **原子更新**：`commander_parse()` 先将当前 `g_sp` 复制到局部变量，在局部变量上修改，最后一次性 struct 赋值写回 `g_sp`，将竞态窗口缩小为单条 memcpy
- **命令超时**：`commander_is_command_timeout()` 记录最后一次收到有效 WebSocket 命令的时间戳（`esp_timer_get_time()`），若超过 500ms 未收到任何命令 → 主循环强制调用 `commander_reset_setpoint()` 回到 DISARMED
- **断连复位**：`commander_reset_setpoint()` 将 `g_sp` 重置为安全状态（DISARMED, 油门=0, motor_active=false），同时清零时间戳防止循环触发超时

| 模式 | 描述 | 当前实现 | 需要传感器 |
|------|------|----------|------------|
| **DISARMED** | 锁定，电机停止 | 电机停止，PID 积分清零 | — |
| **STABILIZE** | 自稳模式（Angle + Rate） | 角度环 P + 角速率环 PID + 混控 | MPU6050 |
| **ALT_HOLD** | 定高（含位置保持） | 自稳 + 高度环 PID（TOF，目标斜坡）+ 位置保持环（光流锁定水平位置抗漂移），切入时自动捕获目标高度 | MPU6050 + TOF400F + PV3901L1 |
| **POS_HOLD** | 定点悬停 | 自稳 + 高度环 + 位置保持 + 光流速度环，**位置推算 (pos_x_m) 飞行中不清零**（位置环用它做相对锁定，跨模式连续保持，仅 DISARM/低油门复位） | MPU6050 + TOF400F + PV3901L1 |

### 3.9 系统管理

- **参数系统**：运行时调参（PID参数可通过WiFi修改，持久化到NVS）— 待实现
- **日志系统**：分级日志（ERROR/WARN/INFO/DEBUG）
- **看门狗**：任务看门狗 + 硬件看门狗防死机 — 待实现
- **电池监测**：ADC 读取电池电压，低电量告警 — 待实现

---

## 4. 目录结构

```
26_Drone/
├── CMakeLists.txt              # 顶层 CMake
├── sdkconfig                   # ESP-IDF 配置 (CONFIG_HTTPD_WS_SUPPORT=y)
├── main/
│   ├── CMakeLists.txt
│   └── main.c                  # 入口：初始化 → 主循环 100Hz 遥测+控制
├── components/
│   ├── drivers/                # 硬件驱动
│   │   ├── CMakeLists.txt
│   │   ├── i2c_bus.h / .c      # I2C0 总线初始化（共享）✅
│   │   ├── mpu6050.h / .c      # MPU6050 陀螺仪驱动 ✅
│   │   ├── tof400f.h / .c      # TOF400F 激光测距驱动（VL53L1X 原生协议）✅
│   │   ├── pv3901l1.h / .c     # PV3901L1 光流驱动 ✅
│   │   └── motor.h / .c        # 电机 PWM 驱动 ✅
│   ├── control/                # 飞行控制
│   │   ├── CMakeLists.txt
│   │   ├── commander.h / .c    # 遥控指令解析 + setpoint 管理 ✅
│   │   ├── pid.h / .c          # PID 控制器 ✅
│   │   ├── mixer.h / .c        # X-quad 混控器 ✅
│   │   ├── altitude.h / .c     # 定高 PID 控制器 ✅
│   │   └── flow_hold.h / .c    # 光流速度保持控制器 ✅
│   ├── communication/          # 通信
│   │   ├── CMakeLists.txt
│   │   ├── wifi_ap.h / .c      # WiFi AP 模式 ✅
│   │   ├── http_server.h / .c  # HTTP + WebSocket 服务器 ✅
│   │   └── web_page.h          # 嵌入式 Web 前端（HTML/CSS/JS）✅
│   ├── estimation/             # 姿态估计
│   │   ├── CMakeLists.txt
│   │   └── attitude.h / .c     # Mahony 互补滤波 ✅
│   └── system/                 # 系统管理（待实现）
│       └── params.h / .c       # 参数存储 (NVS)
└── docs/
    └── (外设文档/数据手册)
```

---

## 5. 开发阶段

### Phase 1：硬件驱动验证
- [x] MPU6050 原始数据读取 + 校准
- [x] TOF400F 距离读取（VL53L1X ST ULD 协议 + skip-counter）
- [x] PV3901L1 光流数据解析
- [x] WiFi AP + HTTP/WebSocket 服务器
- [x] Web 前端（虚拟摇杆 + 传感器数据面板 + 模式切换）
- [x] Commander 遥控指令解析
- [x] 电机 PWM 输出 + ESC 校准（四路 + 单电机）

### Phase 2：飞控核心
- [x] Mahony 姿态估计（100Hz 集成）
- [x] PID 控制器 + 电机混控（rate mode）
- [x] 角度外环自稳（Angle + Rate 串级，STABILIZE 模式）
- [ ] 1kHz 稳定器主循环（从 100Hz 轮询重构为定时中断）
- [ ] 基础自稳起飞测试

### Phase 3：高级飞行模式
- [x] 定高模式（TOF PID 高度环 + vz 阻尼 + 起飞目标斜坡）
- [x] 光流速度保持（IMU+光流互补滤波速度环）+ 位置保持/move_to
- [x] **水平控制链路全米制化**（光流按 TOF 高度换算 m/s、位置反馈航位推算，2026-07-02 —— 修复环增益随高度漂移导致的定点不稳/起飞漂移，**待试飞验证**）
- [x] WiFi 遥控 + 遥测（WebSocket 已实现，遥测 20Hz 异步）
- [x] 失控保护 + 安全逻辑（命令超时/断连 → DISARMED，校准命令地面闸门，电机测试仅限锁定，IMU/TOF 失效保护，2026-07-02 加固）

### Phase 4：调优与完善
- [ ] flow_scale 米制系数实测标定（默认 0.00244 rad/count）
- [ ] 米制化 + 安全加固上机试飞验证
- [ ] PID 参数整定（已按米制折算旧调优值，仍需试飞确认）
- [ ] 电池监测（ADC）+ 低压保护
- [ ] 起飞前自检（pre-arm check）
- [ ] 参数系统 + NVS 持久化

### Phase 5：传感器扩展（BN-880 GPS+磁力计，已购）
- [ ] 磁力计驱动（并入 I2C0；双地址探测 HMC5883L@0x1E / QMC5883L@0x0D；硬磁/软磁标定）
- [ ] Mahony 九轴（磁力计 yaw 修正，解决 yaw 漂移旋转位置锁参考系的问题）
- [ ] 光流增量按 yaw 旋转到世界系再积分（定点从机体系升级为世界系锁定）
- [ ] GPS 接入（仅室外导航需求确认后；米级精度对室内定点无用；UART 引脚另选，GPIO44 已被光流占用）

---

## 6. 安全设计原则

1. **上锁/解锁机制**：上电默认 DISARMED，需显式切换模式解锁
2. **油门死区**：throttle < 5% → 停转 + PID 复位，防止地面角度环意外驱动电机
3. **MOTOR_MIN 地板**：mixer 输出不低于 5%，确保电机不意外停转，但采用 floor-clip 而非 shift-up（避免零油门安全隐患）
4. **失控保护（命令超时）**：超过 500ms 未收到 WebSocket 命令 → `commander_is_command_timeout()` 返回 true → 主循环调用 `commander_reset_setpoint()` 强制 DISARMED ✅
5. **断连保护**：所有 WebSocket 客户端断开（`g_ws_count == 0`）→ `http_server` 触发 `ws_disconnect_cb` → `commander_reset_setpoint()` 强制 DISARMED ✅
6. **前端重连安全**：WebSocket 重连时前端复位所有控制变量（throttle/roll/pitch/yaw → 0, mode → disarmed, motorPWM → 1000μs），UI 同步复位，发送 DISARMED 到 ESP ✅
7. **setpoint 原子更新**：`commander_parse()` 先在局部变量构建完整 setpoint，再一次 struct 赋值写入 `g_sp`，避免逐字段修改被主循环读到中间态 ✅
8. **校准命令地面闸门**：`calibrate`/`gyro_calib`/`calibrate_motor`/`level_trim`/`reset_trim` 仅在 DISARMED 下执行（ESC 校准会把电机打到满油门 6 秒 + 阻塞主循环 ~11 秒，飞行中触发等于炸机）✅
9. **电机测试仅限锁定**：`motor_active` 手动电机数组只在 DISARMED（台架测试）下生效，飞行模式完全忽略——否则前端 All MAX 按钮在飞行中等于全油门指令 ✅
10. **IMU 失效保护**：连续 20 次（~200ms）`mpu6050_read` 失败 → 姿态反馈不可信 → 强制 DISARMED；瞬时失败保留上一帧有效值（冻结）而非清零，避免 gyro=0 假反馈造成扭矩突跳 ✅
11. **TOF 数据过期检测**：超过 500ms 无新样本 → 驱动报失效（tof=0）而非永远返回旧缓存 → 定高环跳过，ALT_HOLD 退化为按摇杆基准油门飞（可控降落）✅
12. **遥测不阻塞控制循环**：20Hz + httpd 任务异步发送，上一帧未完成直接丢帧 ✅
13. **低电量保护**：电池电压 < 阈值 → LED告警 → 自动降落 — 待实现
14. **看门狗**：任一关键任务卡死 → 系统复位 — 待实现
15. **电机输出限幅**：PWM 范围硬限制，mixer 输出钳位 0.0–1.0
16. **命令延迟执行**：校准等阻塞操作通过 `pending_cmd` 延迟到主循环执行，避免冻结 HTTP/WebSocket 通信任务

---

## 7. 参考资料

- [ESP32-S3 技术手册](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [esp-drone 开源飞控](https://github.com/espressif/esp-drone)
- [Crazyflie 飞控架构](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
- [Mahony AHRS 论文](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- MPU6050 数据手册 (RM-MPU-6000A.pdf)
- TOF400F 规格书 (TOF400F规格书.pdf)
- PV3901L1 光流模块说明书 (PV3901L1mini光流模块说明书.pdf)
