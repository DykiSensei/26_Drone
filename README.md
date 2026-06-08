# 26_Drone

基于 ESP32-S3 的四旋翼飞控，ESP-IDF 5.5.4 开发。目标：自主识别垃圾并控制机械臂捡拾。

- **主控**：ESP32-S3（飞控核心，本仓库）
- **协处理器**：ESP32-P4（目标识别，检测垃圾位置，通过 WebSocket 将坐标发送给 S3）

## 硬件清单

| 模块 | 型号 | 接口 | 地址/参数 |
|------|------|------|-----------|
| 主控 | ESP32-S3-DEV-KIT-N16R8 | — | 16MB Flash / 8MB PSRAM, 240MHz 双核 |
| IMU | MPU6050 | I2C | 0x68 |
| 激光测距 | TOF400F (VL53L1X) | I2C | 0x29 |
| 光流 | PV3901L1 | UART | RX=GPIO44, 波特率 19200 |
| 电机×4 | 无刷电机 + 电调 | LEDC PWM | 400Hz |

## 引脚分配

| 信号 | GPIO | 说明 |
|------|------|------|
| I2C0 SDA | 9 | 共用：MPU6050 + TOF400F |
| I2C0 SCL | 8 | 共用：MPU6050 + TOF400F |
| UART1 RX | 44 | PV3901L1 光流（模块仅有 TX） |
| YAW_MODE | 15 | 光流偏航模式选择 |
| M0 (FR, CCW) | 14 | 前右电机, LEDC PWM |
| M1 (FL, CW) | 11 | 前左电机, LEDC PWM |
| M2 (RL, CCW) | 13 | 后左电机, LEDC PWM |
| M3 (RR, CW) | 12 | 后右电机, LEDC PWM |
| RGB LED | 48 | WS2812 状态指示 |

## 飞行模式

| 模式 | 说明 | 传感器依赖 |
|------|------|------------|
| **DISARMED** | 电机锁定，安全状态 | — |
| **STABILIZE** | 自稳（角度 + 角速率串级 PID） | MPU6050 |
| **ALT_HOLD** | 定高 + 水平位置保持（TOF 高度 PID + vz 阻尼 + 光流锁位抗漂移） | MPU6050 + TOF400F + PV3901L1 |
| **POS_HOLD** | 定点悬停（同 ALT_HOLD 基础上叠加光流速度环） | MPU6050 + TOF400F + PV3901L1 |

## 控制链路

```
MPU6050 → DLPF 21Hz + accel EMA → Mahony AHRS → 欧拉角 (roll/pitch/yaw)
                            ↓
摇杆 → 目标倾角 ±30° + 光流修正 ±8° → Angle P → 目标角速率
                            ↓
                    Rate PID (D项LPF) → 力矩输出
                            ↓
              Mixer (X-quad) → ÷cos(roll)·cos(pitch) 倾角补偿 → 4×PWM → 电机
```

定高模式：摇杆油门 + 高度 PID（含 vz 互补滤波、起飞目标斜坡、地效区禁I）→ 有效油门
悬停模式：光流速度保持（连续 quality 权重 + IMU 互补滤波 + 陀螺补偿 + 死区）→ 姿态修正角 ±8°；位置环锁定水平位置抗漂移
起飞：NVS 学习的悬停油门作前馈 + 200ms ESC 同步 idle 阶段 + 目标高度斜坡（0.3 m/s）+ 地效区禁 I

## 软件架构

```
26_Drone/
├── main/main.c                    # 入口 — 初始化 + 100Hz 主循环（实测dt + vTaskDelayUntil 锁周期）
├── components/
│   ├── drivers/                   # 硬件驱动 (I2C, MPU6050, TOF400F, PV3901L1, Motor)
│   ├── estimation/                # Mahony AHRS 姿态估计
│   ├── control/                   # 飞行控制 (Commander, PID, Mixer, Altitude, Flow_Hold, Position)
│   ├── communication/             # WiFi AP + HTTP/WebSocket + 嵌入式 Web 前端
│   └── system/                    # NVS 参数持久化（悬停油门学习）
└── build/                         # 构建产物
```

**FreeRTOS 任务分配**（双核 240MHz）：

| 核心 | 任务 | 频率 | 说明 |
|------|------|------|------|
| Core 1 | `main` | 100Hz | 传感器→姿态→控制→遥测 |
| Core 1 | `flow_rx` | 事件驱动 | 光流 UART 解析 |
| Core 1 | `http_server` | 事件驱动 | HTTP + WebSocket |
| Core 0 | WiFi | — | ESP-IDF 协议栈 |

## 构建与烧录

**环境要求**：ESP-IDF 5.5.4

```bash
# 首次构建（或在新机器上）
idf.py set-target esp32s3

# 构建
idf.py build

# 烧录 + 串口监视
idf.py flash monitor
```

Flash 端口默认 `COM14`，ESP-IDF 路径见 `.vscode/settings.json`。

## 连接与控制

1. 上电后 ESP32-S3 自动创建 WiFi AP
   - **SSID**：`Drone-XXXX`（XXXX = MAC 后 4 位）
   - **密码**：`12345678`
   - **IP**：`192.168.4.1`
2. 浏览器打开 `http://192.168.4.1`
3. 界面提供：虚拟摇杆（Roll/Pitch）、油门滑块、Yaw 滑块、模式按钮、方向键、校准按钮、自动起飞面板

## WebSocket API

### 遥控指令（浏览器 → ESP，50Hz）

```json
{
  "throttle": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
  "vel_x": 0.0, "vel_y": 0.0,
  "mode": "stabilize",
  "motor": [0.0, 0.0, 0.0, 0.0],
  "mtrim": [0.0, 0.0, 0.0, 0.0]
}
```

### 水平移动 API（Web 前端按钮 / P4 视觉指令）

| 命令 | JSON | 说明 |
|------|------|------|
| 速度指令 | `{"vel_x": 0.5, "vel_y": 0.0}` | 随 50Hz stick 流发送，按住移动松开停止 |
| 位置偏移 | `{"cmd": "move_to", "x": 0.5, "y": -0.3}` | P4 视觉定位，光流积分闭环 |
| 停止 | `{"cmd": "move_stop"}` | 紧急停止水平移动 |

`vel_x`/`vel_y` 约定：+x=前、-x=后、+y=右、-y=左，范围 -1.0~1.0。在所有非 DISARMED 模式下生效。

### 特殊命令

```json
{"cmd": "calibrate"}                      // 四路 ESC 油门校准
{"cmd": "gyro_calib"}                     // 陀螺仪+加速度计零偏再校准（放水平面，保持静止 1 秒，自动复位 Mahony）
{"cmd": "level_trim"}                     // 捕获当前姿态角作为水平零位（gyro_calib 后等 2 秒再执行）
{"cmd": "reset_trim"}                     // 重置水平修正量为零
{"cmd": "calibrate_motor", "motor_index": 0}  // 单电机校准 (0=FR,1=FL,2=RL,3=RR)
{"cmd": "takeoff", "height": 0.5, "base_throttle": 0.4}  // 自动起飞（带 ESC 200ms 同步 + 悬停油门前馈）
{"cmd": "flow_comp", "kx": -2.5, "ky": -2.5}             // 光流陀螺补偿系数运行时标定
{"cmd": "flow_scale", "s": 1.0}                          // IMU 互补滤波 scale 运行时标定（观察 flow.vx vs flow.cx 调）
```

> **起飞前校准流程**：放在起飞面 → `gyro_calib` → 等 2 秒 → `level_trim` → 油门拉到底 → 解锁起飞
>
> **Arming 前置**：从 DISARMED 切到其他模式时，油门必须 < 5%，否则切换被拒（log "ARM REFUSED"）

### 遥测数据（ESP → 浏览器，100Hz）

```json
{
  "accel": [x, y, z],           // 加速度 m/s²（未滤波，调试用；Mahony 用 EMA 后的值）
  "gyro": [x, y, z],            // 角速度 rad/s
  "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},  // 姿态角 °
  "tof": 1234,                  // TOF 距离 mm
  "alt": {"target": 1.20, "out": 0.015, "vz": 0.0},  // 定高目标/输出/垂直速度
  "flow": {                     // 光流综合数据
    "x": 0.0, "y": 0.0,         // 积分位移（dead-reckoning）
    "qual": 0,                  // 原始 quality 0-255
    "cr": 0.0, "cp": 0.0,       // out_roll_deg / out_pitch_deg（PID 输出修正角）
    "cx": 0.0, "cy": 0.0,       // 陀螺补偿+EMA 平滑后的单帧光流（标定用）
    "vx": 0.0, "vy": 0.0        // IMU+flow 互补滤波后的速度估计（控制器实际用的反馈）
  },
  "motor": [0.0, 0.0, 0.0, 0.0],  // 电机 0-1
  "mtrim": [0.0, 0.0, 0.0, 0.0],  // 逐电机微调
  "pid": [0.0, 0.0, 0.0],       // PID 输出
  "trim": {"roll": 0.0, "pitch": 0.0},  // 水平修正量 °
  "mode": "stabilize",
  "failsafe": "normal"          // 分级 failsafe 状态: normal/hold/descend/land
}
```

## PID 参数（当前调优值）

| 轴 | Kp | Ki | Kd | 输出限幅 | 积分限幅 |
|----|----|----|----|----|----|
| Roll Rate | 0.25 | 0.02 | 0.01 | ±0.8 | 0.15 |
| Pitch Rate | 0.25 | 0.02 | 0.01 | ±0.8 | 0.15 |
| Yaw Rate | 0.8 | 0.05 | 0.0 | ±0.5 | 0.15 |
| Angle P | 6.0×DEG2RAD | — | — | ±30° | — |
| Altitude | 0.25 | 0.02 | 0.0 | ±0.3 油门 | 0.15 |
| Flow Velocity | 0.4 | 0.06 | 0.0 | ±8° | 6° |
| Position | 0.8 | 0.02 | 0.0 | ±80 flow | 30 |

> **PID 通用**: D 项 derivative-on-measurement + EMA LPF（α=0.5，截止≈20Hz），可外部 `freeze_integral` 冻结积分。
> **Altitude**: vz 垂直速度阻尼（KD_VZ=0.5，IMU 加速度积分 + TOF 修正 K=0.2 互补滤波）+ 起飞目标斜坡（0.3 m/s）+ 地效区禁 I（高度 < 0.3 m）。
> **Flow Velocity**: 陀螺补偿（Kx=Ky=−2.5，可 `flow_comp` 调）+ EMA 平滑（α=0.3） + 死区（1.0）+ 连续 quality 权重（30-80 线性）+ IMU 互补滤波（scale 可 `flow_scale` 调，默认 1.0，慢衰减 τ≈10s）。
> **Throttle**: 倾角补偿 `÷ cos(roll)·cos(pitch)`（clamp 1.5x） + 起飞 ESC 同步 idle 200ms + 悬停油门 NVS 学习（α=0.001，差距>1% 才写 flash）。

## 安全设计

- **上锁/解锁**：上电默认 DISARMED，需手动切换模式解锁；**arming 前置**——从 DISARMED 切到其他模式时油门必须 < 5%，否则强制保持 DISARMED 并 log "ARM REFUSED"
- **任务看门狗 (TWDT)**：主循环注册 1s 超时，I2C/UART 卡死自动 panic + 重启；校准类阻塞命令期间临时摘除，校准完重新注册（同时复位 dt 时间戳）
- **分级 Failsafe**（取代旧的 500ms→DISARMED 自由落体）：
  - 0.5s 无命令 → **HOLD**：roll/pitch/yaw 归零（保持姿态 + 当前油门 + 模式 → 悬停）
  - 1.0s → **DESCEND**：强制 STABILIZE + 油门按 0.05/s 斜坡降
  - 5.0s → **LAND**：DISARMED 熄火 + `commander_reset_setpoint()` 强制重新走 arming 流程
  - 任意阶段链路恢复 → 立即退出 failsafe
- **油门死区**：throttle < 5% → 停转 + 全 PID 复位 + spool-up 窗口清零，防止地面角度环翘机
- **MOTOR_MIN 地板**：mixer 输出不低于 5%（floor-clip，非 shift-up）
- **倾角补偿油门**：飞机倾斜时按 `1/cos(roll)·cos(pitch)` 提升油门保持升力，避免银行→掉高度→alt PID 过冲
- **ESC 同步 spool-up**：takeoff 后 200ms 强制所有电机 idle 5%，等 4 个电调完全 sync（避免起飞瞬间推力不平衡导致倾翻）
- **命令延迟执行**：ESC 校准、陀螺仪校准等阻塞操作通过 `pending_cmd` 延迟到主循环执行，避免冻结 WebSocket 通信
- **光流质量软启动**：qual 30-80 之间线性权重渐变（旧版 50 binary 切断），起飞期 qual 低也按比例介入避免"位置开窗"；qual 低时 PID 冻结积分防噪声 windup
- **光流陀螺补偿**：扣除姿态变化引起的旋转光流污染（`flow_comp = flow − K·gyro`，实测 Kx=Ky=−2.5），否则飞行中纠偏反向、持续漂移
- **IMU+光流互补滤波**：水平速度估计 = IMU 加速度积分（100Hz 快通道）+ 光流校正（50Hz 绝对参考），起飞期 flow 还没准备好也有可靠速度反馈
- **地效区禁 I**：高度 < 0.3 m 时 alt PID 不积分，避免离开地效瞬间残留负积分把油门拉死
- **悬停油门学习**：起飞用 NVS 中学习的悬停油门作前馈，alt PID 只填差量，不必从 0 慢慢积；稳定悬停时持续学习；DISARMED 时存 flash
- **起飞保护**：目标高度斜坡缓升防过冲坠机；起飞即锁定水平位置抗漂移；命令竞态修复（takeoff 不被后续 stick 冲掉）

## 开发状态

- [x] MPU6050 / TOF400F / PV3901L1 驱动
- [x] WiFi AP + HTTP/WebSocket 服务器
- [x] Web 前端（虚拟摇杆 + 数据面板 + 方向按钮）
- [x] Mahony 姿态估计
- [x] 角度 + 角速率串级 PID（STABILIZE）
- [x] 定高模式（TOF 高度 PID）
- [x] 光流速度保持 + 位置控制
- [x] 水平移动 API（vel_x/vel_y + move_to）+ 前端方向按钮
- [x] 安全机制完善（命令超时 + 断连保护 + 校准链路修复）
- [x] 姿态校准修复（gyro+accel 同步重校准 + Mahony 复位）
- [x] 推力不对称补偿（Mtrim 逐电机微调）
- [x] 自动起飞功能（前端滑块设定高度+基准油门，一键起飞）
- [x] 光流传感器重新启用（`FLOW_ENABLED=1`）
- [x] 高度环 vz 速度阻尼（IMU/TOF 互补）+ 起飞目标斜坡（治理上下摇摆/起飞过冲坠机）
- [x] 光流陀螺补偿（旋转-平移解耦）+ 速度 EMA/死区 + 起飞位置锁位
- [x] 失控保护：任务看门狗 TWDT（1s 超时）+ 分级 failsafe（hold/descend/land）+ arming 前置（油门必须先归零）
- [x] PID 升级：D 项 EMA LPF + 实测 dt + `vTaskDelayUntil` 锁周期 + `freeze_integral`（地效区/低质量场景）
- [x] MPU6050 DLPF 21Hz + accel 二次 EMA（抗电机振动）
- [x] 倾角补偿油门 + ESC 同步 idle 200ms（起飞稳）
- [x] 地效区禁 alt I 项（< 0.3 m 不积分，离开地效不掉高度）
- [x] 悬停油门 + IMU 互补滤波 scale NVS 持久化（DISARMED 时按字段独立脏标记存 flash）
- [x] 光流软启动（连续 quality 权重 30-80）+ IMU/光流速度互补滤波（100Hz 高频通道）
- [ ] 光流 imu_scale 试飞标定（默认 1.0，通过 web UI 或 `flow_scale` 命令调，DISARMED 时存 NVS）
- [ ] 1kHz 稳定器主循环重构
- [ ] 电池监测 (ADC) — 硬件限制不支持
- [ ] PID 参数系统 + 完整 NVS 持久化（目前仅持久化 hover_throttle）

## 参考资料

- [DESIGN.md](DESIGN.md) — 完整架构、设计决策
- [esp-drone 开源飞控](https://github.com/espressif/esp-drone)
- [Crazyflie 飞控架构](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
