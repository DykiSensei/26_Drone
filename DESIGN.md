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
| Core 1 | **main** | 100 Hz | 1 | 传感器读取 → 姿态计算 → 控制 → 遥测广播 |
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
- 配置：陀螺仪 ±2000°/s, 加速度计 ±16g, DLPF 256Hz, 采样率 8kHz (gyro) / 1kHz (accel)
- 上电自动校准：采样 500 次求零偏（陀螺仪 xyz + 加速度 xy）
- 输出单位：加速度 m/s², 角速度 rad/s, 已扣除零偏
- **运行时陀螺仪再校准**：`mpu6050_recalibrate_gyro()`，采样 100 次（1 秒），用于起飞前零偏修正

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
- 数据包解析：帧头 0xFE 0x04，9字节包，和校验
- 提取 flow_x, flow_y, qual（质量）
- 积分位移：flow_x_i += flow_x, flow_y_i += flow_y（双轴积分，通过遥测返回）

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
  - roll/pitch rate PID：`Kp=0.5, Ki=0.02, Kd=0.04`，输出限幅 ±0.5，积分限幅 0.15
  - yaw rate PID：`Kp=0.8, Ki=0.05, Kd=0.0`，输出限幅 ±0.5，积分限幅 0.15
- 角度环参数（STABILIZE 模式）：
  - `MAX_ANGLE_DEG = 30°` — 最大倾斜角
  - `ANGLE_KP = 6.0 * 0.017453293f` (≈ 0.105 rad/s 每度误差) — 纯 P 控制，无积分
- **水平修正 (Level Trim)**：补偿 MPU6050 安装偏移
  - `CMD_LEVEL_TRIM` 指令 → `g_trim_roll/g_trim_pitch` 捕获当前 roll/pitch 作为零位
  - 角度环计算时扣除：`roll_err = roll - g_trim_roll`
- 低油门 (< 5%) 时停转 + PID 复位，防止地面翘机
- 混控 `MIXER_SCALE = 0.2f`，电机最低怠速 5%（`MOTOR_MIN`）

**定高/悬停模式**：ALT_HOLD / POS_HOLD 均使用角度外环（同 STABILIZE）+ 定高 PID（TOF 反馈）。POS_HOLD 的位置环待实现。

### 3.3.1 定高控制器 `control/altitude.h`

- `altitude_ctrl_t`：封装 altitude PID + 目标高度
- `altitude_init()`：初始化 PID（Kp=0.5, Ki=0.05, Kd=0.0，输出限幅 ±0.3 油门，积分限幅 0.15）
- `altitude_capture_target(current_m)`：切入定高模式时捕获当前 TOF 距离为目标
- `altitude_update(target_m, current_m, dt)`：运行 PID，返回油门修正量
- `altitude_reset()`：清零积分（DISARMED 或低油门时调用）
- 有效油门 = 摇杆基准 + 高度 PID 修正（钳位 0.0–1.0）

### 3.3.2 光流速度保持控制器 `control/flow_hold.h`

在 STABILIZE / ALT_HOLD / POS_HOLD 模式下叠加水平速度修正，主动抵抗漂移，实现垂直起飞。

- `flow_hold_t`：封装两个速度 PID + 输出修正角
- `flow_hold_init()`：初始化 PID（Kp=0.05, Ki=0.01, Kd=0.0，输出限幅 ±5°，积分限幅 2°）
- `flow_hold_set_velocity(vx, vy)`：设置速度指令（flow 原始单位），0=静止保持，非零=主动移动
- `flow_hold_update(flow_x, flow_y, qual, height_m)`：有新光流数据时更新
  - 质量门控：`qual > 100` 才有效
  - 高度门控：0.04m < height < 3.0m
  - 用 `esp_timer_get_time()` 计算真实 dt（光流帧率低于 100Hz）
  - 质量不满足时修正指数衰减（×0.95）
- `flow_hold_reset()`：DISARMED / 低油门时清零 PID 和输出
- `flow_hold_is_active()`：`quality_gain > 0.01` 时返回 true
- 修正叠加在摇杆目标角度上（±30° + ±5°），不影响飞行员操控权限
- 速度环的积分项隐含位置保持效果（∫速度误差 = 位置误差）

```
光流 flow_x/flow_y (速度测量，setpoint 可设为非零)
    → 速度 PID → 修正角 ±5°
    → 叠加到 stick 目标角度 → Angle P → Rate PID → Mixer → 电机
```

### 3.3.3 水平移动控制（Web 按钮 / P4 API）

#### 速度指令（Web 前端方向按钮）

Web 前端新增 4 个方向按钮（▲前/▼后/◀左/▶右），按住移动、松开停止：

- 按钮事件：`mousedown`/`touchstart` → `vel_x`/`vel_y` = ±0.5，`mouseup`/`touchend` → 清零
- STOP 按钮：立即清零 + 发送 `move_stop` 命令
- 速度值通过 50Hz 摇杆数据流发送：`{"vel_x": 0.5, "vel_y": 0.0}`
- `commander_parse()` 解析 `vel_x`/`vel_y` 字段，钳位 -1.0~1.0
- 主循环将归一化速度映射为光流单位（×80），取反后送入 flow_hold PID
- 在所有非 DISARMED 模式下生效，有光流质量门控

#### 位置指令 `move_to`（P4 视觉对接）

P4 检测到垃圾后发送相对位置偏移指令，飞控通过光流积分闭环移动：

```json
{"cmd": "move_to", "x": 0.5, "y": -0.3}
```

- x: 前向偏移（光流积分单位），y: 右向偏移
- 作为延迟命令（`pending_cmd`），在主循环中执行
- 启动时捕获当前光流积分位置作为起点，目标 = 起点 + 偏移
- 位置 PID 输出速度指令 → flow_hold 速度环 → 姿态修正 → 电机

#### 停止指令 `move_stop`

```json
{"cmd": "move_stop"}
```

立即清零速度指令 + 复位位置控制器，回到静止保持模式。

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

- `position_ctrl_t`：封装两个位置 PID + 目标位置
- `position_init()`：初始化 PID（Kp=0.5, Ki=0.02, Kd=0.0，输出限幅 ±80，积分限幅 30）
- `position_set_target(offset_x, offset_y, current_ix, current_iy)`：设置目标 = 当前位置 + 偏移
- `position_update(flow_ix, flow_iy, dt)`：运行位置 PID，输出速度指令
- `position_reached()`：位置误差 < 20 (flow 单位) 判定到达
- `position_reset()`：停用并清零所有状态
- 到达后自动复位 → 回退到静止保持 (velocity setpoint=0)

未来扩展（Phase 3）：

| 控制器 | 频率 | 输入 | 输出 |
|--------|------|------|------|
| 角度环 (P) | 1000Hz | roll/pitch/yaw 期望角 | 期望角速率 |
| 角速率环 (PID) | 1000Hz | 期望角速率 - 实际角速率 | 力矩输出 |
| 高度环 (PID) | 100Hz | 期望高度 - TOF距离 | 油门补偿 ✅ 已实现 |
| 水平速度环 (PID) | 100Hz | 光流速度 (setpoint=0) | 姿态修正角 ✅ 已实现 |
| 水平位置环 (PID) | 100Hz | 期望位置 - 光流积分 | 期望速度 |

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
- WebSocket `/ws`：双向 JSON 通信，最多 4 客户端
- 帧类型处理：PING 自动回复 PONG，CLOSE 正常断开，非 TEXT 帧忽略
- 命令回调模式：`http_server_set_command_cb(commander_parse)`
- **断连安全回调**：`http_server_set_disconnect_cb()` 注册回调，当 `g_ws_count` 降至 0（所有客户端断开）时触发 `commander_reset_setpoint()` 强制 DISARMED。在两处触发：`ws_close_handler`（TCP 断开）和 `http_server_broadcast`（异步发送失败清理）

#### 遥测数据（ESP → 浏览器，100Hz）
```json
{
  "accel": [x, y, z],
  "gyro": [x, y, z],
  "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
  "tof": 1234,
  "alt": {"target": 1.20, "out": 0.015},
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
{"cmd": "move_to", "x": 0.5, "y": -0.3}  // P4 位置偏移指令（光流积分单位）
{"cmd": "move_stop"}         // 停止所有水平移动
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
- 命令通过 `pending_cmd` 字段延迟到主循环执行，确保阻塞操作（校准）不冻结 HTTP/WebSocket 通信
- **原子更新**：`commander_parse()` 先将当前 `g_sp` 复制到局部变量，在局部变量上修改，最后一次性 struct 赋值写回 `g_sp`，将竞态窗口缩小为单条 memcpy
- **命令超时**：`commander_is_command_timeout()` 记录最后一次收到有效 WebSocket 命令的时间戳（`esp_timer_get_time()`），若超过 500ms 未收到任何命令 → 主循环强制调用 `commander_reset_setpoint()` 回到 DISARMED
- **断连复位**：`commander_reset_setpoint()` 将 `g_sp` 重置为安全状态（DISARMED, 油门=0, motor_active=false），同时清零时间戳防止循环触发超时

| 模式 | 描述 | 当前实现 | 需要传感器 |
|------|------|----------|------------|
| **DISARMED** | 锁定，电机停止 | 电机停止，PID 积分清零 | — |
| **STABILIZE** | 自稳模式（Angle + Rate） | 角度环 P + 角速率环 PID + 混控 | MPU6050 |
| **ALT_HOLD** | 定高模式 | 自稳 + 高度环 PID（TOF 反馈），切入时自动捕获目标高度 | MPU6050 + TOF400F |
| **POS_HOLD** | 定点悬停 | 自稳 + 光流速度保持 + 高度环，切入时重置光流积分 | MPU6050 + TOF400F + PV3901L1 |

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
- [x] 定高模式（TOF PID 高度环）
- [x] 光流速度保持（速度环，所有模式生效）
- [x] WiFi 遥控 + 遥测（WebSocket 已实现）
- [x] 失控保护 + 安全逻辑（命令超时 500ms → DISARMED，断连 → DISARMED，前端重连安全复位，motor_active 路径增加安全门禁）

### Phase 4：调优与完善
- [ ] PID 参数整定（已从初始值调整，仍需试飞确认）
- [ ] 传感器融合优化
- [ ] 电池监测（ADC）
- [ ] 参数系统 + NVS 持久化

---

## 6. 安全设计原则

1. **上锁/解锁机制**：上电默认 DISARMED，需显式切换模式解锁
2. **油门死区**：throttle < 5% → 停转 + PID 复位，防止地面角度环意外驱动电机。`motor_active` 手动电机控制路径同样受此约束
3. **MOTOR_MIN 地板**：mixer 输出不低于 5%，确保电机不意外停转，但采用 floor-clip 而非 shift-up（避免零油门安全隐患）
4. **失控保护（命令超时）**：超过 500ms 未收到 WebSocket 命令 → `commander_is_command_timeout()` 返回 true → 主循环调用 `commander_reset_setpoint()` 强制 DISARMED ✅
5. **断连保护**：所有 WebSocket 客户端断开（`g_ws_count == 0`）→ `http_server` 触发 `ws_disconnect_cb` → `commander_reset_setpoint()` 强制 DISARMED ✅
6. **前端重连安全**：WebSocket 重连时前端复位所有控制变量（throttle/roll/pitch/yaw → 0, mode → disarmed, motorPWM → 1000μs），UI 同步复位，发送 DISARMED 到 ESP ✅
7. **setpoint 原子更新**：`commander_parse()` 先在局部变量构建完整 setpoint，再一次 struct 赋值写入 `g_sp`，避免逐字段修改被主循环读到中间态 ✅
8. **低电量保护**：电池电压 < 阈值 → LED告警 → 自动降落 — 待实现
9. **看门狗**：任一关键任务卡死 → 系统复位 — 待实现
10. **传感器失效检测**：I2C 通信失败重试3次 → 切换降落模式 — 待实现
11. **电机输出限幅**：PWM 范围硬限制，mixer 输出钳位 0.0–1.0
12. **命令延迟执行**：校准等阻塞操作通过 `pending_cmd` 延迟到主循环执行，避免冻结 HTTP/WebSocket 通信任务

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
