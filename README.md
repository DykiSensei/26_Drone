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
| **ALT_HOLD** | 定高 + 水平位置保持（TOF 高度 PID + vz 阻尼 + IMU/光流互补悬停） | MPU6050 + TOF400F + PV3901L1 |
| **POS_HOLD** | 定点悬停（同 ALT_HOLD，预留扩展接口） | MPU6050 + TOF400F + PV3901L1 |

## 控制链路

```
MPU6050 → Mahony AHRS → 欧拉角 (roll/pitch/yaw)
                            ↓
摇杆 → 目标倾角 ±30° + 光流修正 ±8° → Angle P → 目标角速率
                            ↓
                    Rate PID → 力矩输出
                            ↓
              Mixer (X-quad) → 4×PWM → 电机
```

定高模式：摇杆油门 + 高度 PID（含 vz 垂直速度阻尼、起飞目标斜坡）→ 有效油门
悬停模式：IMU 加速度积分 + 光流测量互补滤波 → 速度 PID → 姿态修正角 ±8°，叠加到目标倾角；位置环（到达目标高度后启动）锁定水平位置抗漂移

---

## 🚁 起飞指南

> **⚠️ 安全须知**：
> - 室内试飞请选择 ≥3×3m 空旷场地，远离人员、宠物、易碎物品
> - 首次试飞**务必拆下螺旋桨**，仅观察电机方向与遥测数据
> - 飞控随时可由"DISARMED"按钮锁定（油门归零亦同效）
> - 命令 500ms 未送达自动 DISARMED；浏览器关闭/断网亦立即 DISARMED

### 1. 起飞前检查

- [ ] 电池电压充足（≥3.7V × 2S/3S）、各电机接线牢固
- [ ] 螺旋桨方向正确：FR/RL 装 CCW 桨，FL/RR 装 CW 桨（参考机架印记）
- [ ] 飞机放在水平面上，**机头方向**与 Web 前端 "↑前" 按钮一致
- [ ] 浏览器已连接 WiFi `Drone-XXXX`（密码 `12345678`），打开 `http://192.168.4.1`
- [ ] 前端右上角连接状态显示 "WS connected"
- [ ] 模式按钮当前为红色 **DISARMED**

### 2. 校准流程（每次开机做一次）

按顺序执行：

1. **陀螺仪 + 加速度零偏校准**
   - 把飞机放在**真正水平的桌面**（用手机水平仪辅助）
   - 点击前端 `Gyro Calib` 按钮
   - 1 秒内保持飞机**完全静止**（采样 100 次）
   - 校准完成后串口日志 `Mahony reset — wait 2s before level trim!`

2. **等待 2 秒** — 让 Mahony 滤波器重新收敛到水平姿态

3. **水平校准（Level Trim）**
   - 飞机仍然水平放置不动
   - 点击 `Level Trim` 按钮
   - 串口日志 `level trim captured: roll=X, pitch=X`
   - 检查遥测面板 `trim.roll` / `trim.pitch` 应记录当前安装偏移

> 跳过 Gyro Calib 仅做 Level Trim 会捕获带零偏的角度 → 飞行中持续漂移。

### 3. 推荐：自动起飞

最稳定的起飞方式，使用斜坡式高度爬升 + 自动位置锁定：

1. 在 Web 前端 "自动起飞" 面板设定：
   - **目标高度**：建议 **0.4–0.6 m**（首飞低一点容错）
   - **基准油门**：建议 **0.35–0.45**（机重不同需微调，过低爬不起来、过高过冲）
2. 点击 **Takeoff** 按钮，飞机会：
   - 自动切入 ALT_HOLD 模式
   - 以 0.3 m/s 的斜坡速度爬升到目标高度
   - 到达目标高度且 |vz| < 0.15 m/s 且 qual > 30 时自动锁定当前 X/Y 位置
3. 观察遥测：
   - `alt.target` 应平滑爬升到目标值
   - `alt.vz` 应在 ±0.2 m/s 之间
   - `flow.ps` 0=idle 1=等达标 2=锁定 3=move_to 中
4. **悬停状态**：飞机应稳定保持在起飞点 ±20cm 范围
5. **降落**：拖动油门滑块到 0 → 飞机停止，或直接点击 `DISARMED`

### 4. 手动起飞（适合熟练后）

1. 校准完成后点击 `STABILIZE` 模式
2. 缓慢推油门到 30–40%，观察四电机均匀启动
3. 飞机刚离地瞬间快速增加到 50%（穿越地效）
4. 微调摇杆保持稳定，达到合适高度后切入 `ALT_HOLD` 自动定高
5. 切换瞬间会**自动捕获**当前高度为目标，无需手动设置

### 5. 飞行中操作

| 操作 | 效果 |
|------|------|
| Roll/Pitch 摇杆 | 期望倾角 ±30° |
| Yaw 滑块 | 期望角速率 ±3 rad/s |
| 油门滑块 | STABILIZE 直接控制；ALT_HOLD 作为基准油门，PID 在此基础上修正 |
| ↑↓←→ 方向按钮 | ALT_HOLD/POS_HOLD 下临时覆盖位置保持，按住移动、松开在新位置重新锁定 |
| STOP 按钮 | 立即清零水平速度，恢复位置保持 |
| 模式切换为 DISARMED | 立即停转所有电机 |

### 6. 常见故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 解锁后电机不转 | 油门 < 5% 安全门禁 | 油门推到 5% 以上 |
| 切入 ALT_HOLD 立即跳高/掉高 | 切入瞬间高度估计错误 | 在 0.5–1m 高度切换；检查 TOF 读数 `tof` 是否合理（40–4000 mm） |
| 飞机持续朝一个方向漂 | 水平校准未做或安装偏移大 | 重新执行 Gyro Calib + Level Trim |
| 飞机起飞后旋转（自转） | 电机方向/螺旋桨错装 | 检查 CCW/CW 桨位 |
| 自动起飞爬不高 | 基准油门过低 | 调高到 0.4–0.45 重试 |
| 自动起飞过冲然后掉 | 基准油门过高 / vz 阻尼不足 | 降低基准油门到 0.35–0.4 |
| 悬停时左右抖动 | 光流陀螺补偿系数偏移 | 飞行中用 `flow_comp` 命令微调 Kx/Ky（默认 -2.5）|
| WebSocket 断连 | 信号弱 / 同时连过多客户端 | 靠近飞机；前端会自动重连并复位为 DISARMED |

---

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

Flash 端口默认 `COM14`，ESP-IDF 路径见 `CLAUDE.md`。

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
{"cmd": "takeoff", "height": 0.5, "base_throttle": 0.4}  // 自动起飞到指定高度
{"cmd": "flow_comp", "kx": -2.5, "ky": -2.5}             // 光流陀螺补偿系数运行时标定
```

### 遥测数据（ESP → 浏览器，100Hz）

```json
{
  "accel": [x, y, z],           // 加速度 m/s²
  "gyro": [x, y, z],            // 角速度 rad/s
  "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},  // 姿态角 °
  "tof": 1234,                  // TOF 距离 mm
  "alt": {"target": 1.20, "out": 0.015, "vz": 0.0},  // 定高目标/输出/垂直速度
  "flow": {
    "x": 0.0, "y": 0.0,         // 光流积分（绝对位置参考）
    "qual": 0,                  // 光流质量 0-255
    "cr": 0.0, "cp": 0.0,       // PID 输出 roll/pitch 修正角 °
    "cx": 0.0, "cy": 0.0,       // 平滑+死区后的光流速度
    "ps": 0,                    // 位置环状态 0=idle 1=pending 2=hold 3=move_to
    "tx": 0, "ty": 0,           // 位置锁定目标
    "fc": 0, "ec": 0,           // 光流帧计数 / 校验错误计数
    "fx": 0, "fy": 0,           // 原始 flow_x/y（本帧）
    "vx": 0.0, "vy": 0.0        // IMU/flow 互补滤波速度估计
  },
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
| Roll Rate | 0.25 | 0.02 | 0.01 | ±0.8 | 0.15 |
| Pitch Rate | 0.25 | 0.02 | 0.01 | ±0.8 | 0.15 |
| Yaw Rate | 0.8 | 0.05 | 0.0 | ±0.5 | 0.15 |
| Angle P | 6.0×DEG2RAD | — | — | ±30° | — |
| Altitude | 0.25 | 0.02 | 0.0 | ±0.3 油门 | 0.15 |
| Flow Velocity | 0.4 | 0.06 | 0.0 | ±8° | 6° |
| Position | 0.15 | 0.02 | 0.0 | ±20 flow | 30 |

> Altitude 另含 **vz 垂直速度阻尼**（KD_VZ=0.5，IMU/TOF 互补滤波）+ **起飞目标斜坡**（0.3 m/s）。
> Flow Velocity 跑在 **IMU+光流互补滤波速度估计**上：IMU 加速度 EMA 滤波后做 100Hz 预测（积分 + 慢衰减），新光流帧（~50Hz）做修正。

## 安全设计

- **上锁/解锁**：上电默认 DISARMED，需手动切换模式解锁
- **油门死区**：throttle < 5% → 停转 + 全 PID 复位，防止地面角度环翘机
- **MOTOR_MIN 地板**：mixer 输出不低于 5%（floor-clip，非 shift-up）
- **命令延迟执行**：ESC 校准、陀螺仪校准等阻塞操作通过 `pending_cmd` 延迟到主循环执行，避免冻结 WebSocket 通信
- **命令超时**：500ms 未收到 WebSocket 指令 → 自动 DISARMED
- **断连保护**：所有 WebSocket 客户端断开 → 自动 DISARMED
- **前端重连复位**：WebSocket 重连时前端复位所有控制变量并发送 DISARMED
- **光流质量门控**：连续 quality 权重（30 起步、80 满权），低质量时冻结 PID 积分防 windup
- **光流陀螺补偿**：扣除姿态变化引起的旋转光流污染（`flow_comp = flow − K·gyro`，实测 Kx=Ky=−2.5）
- **起飞双保险**：高度目标斜坡缓升防过冲坠机；位置锁定延迟到达目标高度后启动（避免上升阶段倾斜让光流积分噪声误判为漂移）
- **IMU 振动抑制**：MPU6050 DLPF=4（~21Hz BW）+ 软件 EMA（~5.7Hz），防止电机振动噪声污染速度估计
- **takeoff 命令原子化**：commander_parse 将 mode/throttle/yaw 在同一 struct 赋值中设置，消除前端"setMode + takeoff"两包之间被主循环切回 DISARMED 的竞态

## 开发状态

- [x] MPU6050 / TOF400F / PV3901L1 驱动
- [x] WiFi AP + HTTP/WebSocket 服务器
- [x] Web 前端（虚拟摇杆 + 数据面板 + 方向按钮）
- [x] Mahony 姿态估计
- [x] 角度 + 角速率串级 PID（STABILIZE）
- [x] 定高模式（TOF 高度 PID + vz 阻尼 + 起飞目标斜坡）
- [x] IMU + 光流互补滤波速度估计
- [x] 光流速度保持 + 位置控制 + 位置锁定延迟到达目标高度后启动
- [x] 水平移动 API（vel_x/vel_y + move_to）+ 前端方向按钮
- [x] 安全机制完善（命令超时 + 断连保护 + 校准链路修复 + takeoff 原子化）
- [x] 姿态校准修复（gyro+accel 同步重校准 + Mahony 复位）
- [x] 推力不对称补偿（Mtrim 逐电机微调）
- [x] 自动起飞功能（前端滑块设定高度+基准油门，一键起飞）
- [x] IMU 振动抑制（DLPF=4 硬件低通 + 软件 EMA）
- [ ] 光流定点最终试飞验证（互补滤波方案上机待飞）
- [ ] 1kHz 稳定器主循环重构
- [ ] 电池监测 (ADC)
- [ ] 参数系统 + NVS 持久化

## 参考资料

- [DESIGN.md](DESIGN.md) — 完整架构、设计决策
- [CLAUDE.md](CLAUDE.md) — Claude Code 协作开发指引
- [esp-drone 开源飞控](https://github.com/espressif/esp-drone)
- [Crazyflie 飞控架构](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)
