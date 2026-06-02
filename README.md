# 26_Drone

基于 ESP32-S3 的四旋翼飞控，ESP-IDF 5.5.4 开发。

## 硬件

- MCU: ESP32-S3
- IMU: MPU6050（I2C, 0x68）
- 定高: TOF400F 激光测距（I2C, 0x29）
- 光流: PV3901L1（UART）
- 机架: X 型四旋翼

## 飞行模式

| 模式 | 说明 |
|------|------|
| DISARMED | 电机锁定 |
| STABILIZE | 自稳（角速度 + 角度串级 PID） |
| ALT_HOLD | 定高（自稳 + TOF 高度 PID） |
| POS_HOLD | 悬停（自稳 + 定高 + 光流速度保持） |

## 控制方式

WiFi AP 模式，浏览器打开 `192.168.4.1` 即可遥控，支持虚拟摇杆和键盘。

## 构建

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

ESP-IDF 环境路径和串口号见 `.vscode/settings.json.shared`。

详细架构、引脚分配和设计决策见 [DESIGN.md](DESIGN.md)。
