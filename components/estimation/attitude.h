#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mahony AHRS 姿态估计器
 *
 * 融合加速度计（m/s^2）和陀螺仪（rad/s），输出 roll/pitch/yaw（度）。
 */

void attitude_init(void);

/**
 * @brief 更新姿态估计
 * @param ax, ay, az 加速度计，单位 m/s^2
 * @param gx, gy, gz 陀螺仪，单位 rad/s
 * @param dt         采样周期，单位 s
 */
void attitude_update(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt);

/**
 * @brief 获取当前欧拉角
 * @param roll_deg  输出 roll，单位度
 * @param pitch_deg 输出 pitch，单位度
 * @param yaw_deg   输出 yaw，单位度（相对初始化时的朝向）
 */
void attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

#ifdef __cplusplus
}
#endif
