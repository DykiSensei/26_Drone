#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "motor.h"

/**
 * @brief X-quad 混控器
 *
 * 输入：油门 (0-1)、roll / pitch / yaw 控制量 (-1 ~ 1)
 * 输出：4 路电机 PWM 比例 (0-1)
 *
 * 电机布局（俯视，机头朝上）：
 *     M1(FL,CW,GPIO11)   M0(FR,CCW,GPIO14)
 *          \                 /
 *           \       ↑       /
 *            \             /
 *             \           /
 *              \         /
 *               \       /
 *              /       \
 *             /         \
 *            /           \
 *           /             \
 *     M2(RL,CCW,GPIO13)  M3(RR,CW,GPIO12)
 *
 * Roll  正 = 右倾  = M1/M2 增，M0/M3 减
 * Pitch 正 = 抬头  = M0/M1 增，M2/M3 减
 * Yaw   正 = 右旋  = CW电机降，CCW电机升
 */
void mixer_apply(float throttle,
                 float roll,    /* -1.0 ~ 1.0 */
                 float pitch,   /* -1.0 ~ 1.0 */
                 float yaw,     /* -1.0 ~ 1.0 */
                 float motor[MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif
