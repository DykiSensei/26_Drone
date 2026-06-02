#include "attitude.h"
#include <math.h>

/* Mahony 参数 — Kp 越大收敛越快，Ki 消除零偏漂移 */
#define MAHONY_KP   2.0f
#define MAHONY_KI   0.005f

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integral_x = 0.0f, integral_y = 0.0f, integral_z = 0.0f;

void attitude_init(void)
{
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    integral_x = integral_y = integral_z = 0.0f;
}

void attitude_update(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float dt)
{
    /* 1. 归一化加速度计 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    /* 2. 从四元数估计重力方向（世界坐标系 Z 轴转到机体坐标系） */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 3. 误差 = 加速度计测量方向 × 估计方向（叉积） */
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    /* 4. 积分误差（PI 控制器） */
    integral_x += ex * MAHONY_KI * dt;
    integral_y += ey * MAHONY_KI * dt;
    integral_z += ez * MAHONY_KI * dt;

    /* 5. 修正陀螺仪（误差反馈） */
    gx += MAHONY_KP * ex + integral_x;
    gy += MAHONY_KP * ey + integral_y;
    gz += MAHONY_KP * ez + integral_z;

    /* 6. 四元数导数 */
    float q0_dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_dot = 0.5f * ( q0 * gx - q3 * gy + q2 * gz);
    float q2_dot = 0.5f * ( q3 * gx + q0 * gy - q1 * gz);
    float q3_dot = 0.5f * (-q2 * gx + q1 * gy + q0 * gz);

    /* 7. 积分 */
    q0 += q0_dot * dt;
    q1 += q1_dot * dt;
    q2 += q2_dot * dt;
    q3 += q3_dot * dt;

    /* 8. 归一化 */
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) {
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
    }
}

void attitude_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    /* 四元数 → 欧拉角（Z-Y-X 顺序，航空惯例） */
    float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                         1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
    float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                         1.0f - 2.0f * (q2 * q2 + q3 * q3));

    *roll_deg  = roll  * 57.29578f;
    *pitch_deg = pitch * 57.29578f;
    *yaw_deg   = yaw   * 57.29578f;
}
