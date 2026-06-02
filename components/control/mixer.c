#include "mixer.h"

/* 姿态控制量相对油门的权重 */
#define MIXER_SCALE      0.2f
#define MIXER_SCALE_YAW  0.4f   /* yaw 需要更大权限对抗扭矩不平衡 */
#define MOTOR_MIN        0.05f  /* 怠速 5%，防止低油门停转 */

void mixer_apply(float throttle,
                 float roll, float pitch, float yaw,
                 float motor[MOTOR_COUNT])
{
    /* 缩放遥控输入到实际控制量（roll 取反以匹配实际布线，
     * yaw 取反已移除——电机实际转向与标准 X-quad 相反） */
    float r = -roll  * MIXER_SCALE;
    float p =  pitch * MIXER_SCALE;
    float y =  yaw   * MIXER_SCALE_YAW;

    /* X-quad 混控（FR/RL=CCW, FL/RR=CW） */
    motor[0] = throttle - r + p + y;   /* Front-Right (CCW) */
    motor[1] = throttle + r + p - y;   /* Front-Left  (CW)  */
    motor[2] = throttle + r - p + y;   /* Rear-Left   (CCW) */
    motor[3] = throttle - r - p - y;   /* Rear-Right  (CW)  */

    /* 底部截断：保证电机不低于怠速，不抬升整体油门 */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motor[i] < MOTOR_MIN) motor[i] = MOTOR_MIN;
    }

    /* 顶部缩放：如果有电机高于 1.0，整体等比例缩小 */
    float max_m = motor[0];
    for (int i = 1; i < MOTOR_COUNT; i++) {
        if (motor[i] > max_m) max_m = motor[i];
    }
    if (max_m > 1.0f) {
        float scale = 1.0f / max_m;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motor[i] *= scale;
        }
    }
}
