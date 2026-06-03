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
| **ALT_HOLD** | 定高（自稳 + TOF 高度 PID，切入时自动捕获目标高度） | MPU6050 + TOF400F |
| **POS_HOLD** | 悬停（自稳 + 定高 + 光流速度保持 + 位置控制） | MPU6050 + TOF400F + PV3901L1 |

## 控制链路

```
MPU6050 → Mahony AHRS → 欧拉角 (roll/pitch/yaw)
                            ↓
摇杆 → 目标倾角 ±30° + 光流修正 ±5° → Angle P → 目标角速率
                            ↓
                    Rate PID → 力矩输出
                            ↓
              Mixer (X-quad) → 4×PWM → 电机
```

定高模式：摇杆油门 + 高度 PID 修正 → 有效油门
悬停模式：光流速度保持 (setpoint=0) → 姿态修正角，叠加到目标倾角

## 软件架构

```
26_Drone/
├── main/main.c                    # 入口 — 初始化 + 100Hz 主循环
├── components/
│   ├── drivers/                   # 硬件驱动 (I2C, MPU6050, TOF400F, PV3901L1, Motor)
│   ├── estimation/                # Mahony AHRS 姿态估计
│   ├── control/                   # 飞行控制 (Commander, PID, Mixer, Altitude, Flow_Hold, Position)
│   └── communication/             # WiFi AP + HTTP/WebSocket + 嵌入式 Web 前端
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
3. 界面提供：虚拟摇杆（Roll/Pitch）、油门滑块、Yaw 滑块、模式按钮、方向键、校准按钮

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
{"cmd": "gyro_calib"}                     // 陀螺仪零偏再校准（保持静止 1 秒）
{"cmd": "level_trim"}                     // 捕获当前姿态角作为水平零位
{"cmd": "reset_trim"}                     // 重置水平修正量为零
{"cmd": "calibrate_motor", "motor_index": 0}  // 单电机校准 (0=FR,1=FL,2=RL,3=RR)
```

### 遥测数据（ESP → 浏览器，100Hz）

```json
{
  "accel": [x, y, z],           // 加速度 m/s²
  "gyro": [x, y, z],            // 角速度 rad/s
  "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},  // 姿态角 °
  "tof": 1234,                  // TOF 距离 mm
  "alt": {"target": 1.20, "out": 0.015},  // 定高目标/输出
  "flow": {"x": 0.0, "y": 0.0, "qual": 0, "cr": 0.0, "cp": 0.0},  // 光流
  "motor": [0.0, 0.0, 0.0, 0.0],  // 电机 0-1
  "mtrim": [0.0, 0.0, 0.0, 0.0],  // 逐电机微调
  "pid": [0.0, 0.0, 0.0],       // PID 输出
  "trim": {"roll": 0.0, "pitch": 0.0},  // 水平修正量 °
  "mode": "stabilize"
}
```

## PID 参数（当前调优值）

| 轴 | Kp | Ki | Kd | 输出限幅 | 积分限幅 |
|----|----|----|----|----|----|
| Roll Rate | 0.5 | 0.02 | 0.04 | ±0.5 | 0.15 |
| Pitch Rate | 0.5 | 0.02 | 0.04 | ±0.5 | 0.15 |
| Yaw Rate | 0.8 | 0.05 | 0.0 | ±0.5 | 0.15 |
| Angle P | 6.0×DEG2RAD | — | — | ±30° | — |
| Altitude | 0.5 | 0.05 | 0.0 | ±0.3 油门 | 0.15 |
| Flow Velocity | 0.05 | 0.01 | 0.0 | ±5° | 2° |
| Position | 0.5 | 0.02 | 0.0 | ±80 flow | 30 |

## 安全设计

- **上锁/解锁**：上电默认 DISARMED，需手动切换模式解锁
- **油门死区**：throttle < 5% → 停转 + 全 PID 复位，防止地面角度环翘机
- **MOTOR_MIN 地板**：mixer 输出不低于 5%（floor-clip，非 shift-up）
- **命令延迟执行**：ESC 校准、陀螺仪校准等阻塞操作通过 `pending_cmd` 延迟到主循环执行，避免冻结 WebSocket 通信
- **光流质量门控**：qual > 100 + 0.04m < height < 3.0m 才生效，否则指数衰减修正

## 开发状态

- [x] MPU6050 / TOF400F / PV3901L1 驱动
- [x] WiFi AP + HTTP/WebSocket 服务器
- [x] Web 前端（虚拟摇杆 + 数据面板 + 方向按钮）
- [x] Mahony 姿态估计
- [x] 角度 + 角速率串级 PID（STABILIZE）
- [x] 定高模式（TOF 高度 PID）
- [x] 光流速度保持 + 位置控制
- [x] 水平移动 API（vel_x/vel_y + move_to）+ 前端方向按钮
- [ ] 1kHz 稳定器主循环重构
- [ ] 失控保护 + 安全逻辑
- [ ] 电池监测 (ADC)
- [ ] 参数系统 + NVS 持久化

## 参考资料

- [DESIGN.md](DESIGN.md) — 完整架构、设计决策
- [esp-drone 开源飞控](https://github.com/espressif/esp-drone)
- [Crazyflie 飞控架构](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
