#include "flow_hold.h"
#include "esp_timer.h"
#include <math.h>

/* 光流信号幅度很小（实测 30cm 平移 comp 才 <10），故 KP 取大值，
 * 否则纠偏角小到可忽略、拦不住漂移。 */
#define FLOW_KP              0.40f  /* comp≈10 → ~4° 纠偏 */
#define FLOW_KI              0.06f  /* 积分消除持续漂移/机身固有偏移 */
#define FLOW_KD              0.0f
#define FLOW_OUTPUT_LIMIT    8.0f   /* 最大 ±8° 修正角 */
#define FLOW_INTEGRAL_LIMIT  6.0f   /* 积分权限（°） */
/* 连续 quality 门限：30 起步介入（低权重），80 满权。
 * 原来 binary 50 切断会让起飞期 qual 在 30-50 徘徊时完全无位置控制 → 漂走才锁。 */
#define FLOW_QUALITY_LOW     30
#define FLOW_QUALITY_HIGH    80
#define FLOW_QUALITY_FREEZE_I  0.5f /* quality_gain 低于此值时冻结 PID 积分，避免噪声 windup */
#define FLOW_MAX_HEIGHT_M    3.0f
#define FLOW_QUALITY_SMOOTH  0.3f

#define DT_MIN  0.005f
#define DT_MAX  0.200f
#define DECAY   0.95f

#define FLOW_VEL_SMOOTH 0.3f   /* 速度 EMA 平滑系数 */
#define FLOW_DEADBAND   1.0f   /* 速度死区：静止残留≈0，设小以免清掉真实小漂移信号 */

/* --- IMU + 光流 互补滤波参数 ---
 * 高频快通道：IMU 加速度积分（短期响应快，长期偏置漂移）
 * 低频绝对通道：光流速度测量（50Hz 离散，绝对参考但延迟）
 * predict 每帧推进 vx_est，update 用 flow 拉回 → 高频细节 + 长期不漂 */
#define IMU_LEAK         0.999f  /* 每帧慢衰减，防 accel 偏置积爆（τ≈10s @100Hz） */
#define FLOW_CORRECT_K   0.30f   /* flow 修正强度：每次新 flow 帧把 vx_est 拉向测量值的比例 */
#define IMU_SCALE_DEFAULT 1.0f   /* m/s² → flow_unit/s²；需要试飞标定，默认 1.0 */

void flow_hold_init(flow_hold_t *fh)
{
    pid_init(&fh->pid_vx, FLOW_KP, FLOW_KI, FLOW_KD,
             FLOW_OUTPUT_LIMIT, FLOW_INTEGRAL_LIMIT);
    pid_init(&fh->pid_vy, FLOW_KP, FLOW_KI, FLOW_KD,
             FLOW_OUTPUT_LIMIT, FLOW_INTEGRAL_LIMIT);
    fh->setpoint_vx   = 0.0f;
    fh->setpoint_vy   = 0.0f;
    fh->out_pitch_deg = 0.0f;
    fh->out_roll_deg  = 0.0f;
    fh->quality_gain  = 0.0f;
    fh->last_update_us = 0;
    fh->active = false;
    fh->gyro_kx = 0.0f;
    fh->gyro_ky = 0.0f;
    fh->flow_x_f = 0.0f;
    fh->flow_y_f = 0.0f;
    fh->flow_x_comp = 0.0f;
    fh->flow_y_comp = 0.0f;
    fh->vx_est = 0.0f;
    fh->vy_est = 0.0f;
    fh->imu_scale = IMU_SCALE_DEFAULT;
    fh->flow_x_corr = 0.0f;
    fh->flow_y_corr = 0.0f;
}

void flow_hold_set_imu_scale(flow_hold_t *fh, float scale)
{
    fh->imu_scale = scale;
}

void flow_hold_predict(flow_hold_t *fh, float ax_world, float ay_world, float dt)
{
    /* IMU 加速度积分 + 慢衰减（防 accel 偏置 / 长期漂移积爆） */
    fh->vx_est = (fh->vx_est + fh->imu_scale * ax_world * dt) * IMU_LEAK;
    fh->vy_est = (fh->vy_est + fh->imu_scale * ay_world * dt) * IMU_LEAK;

    /* PID 100Hz 更新（之前在 update 里 50Hz，损失了 IMU 高频信息）。
     * 低 quality 时冻结积分避免噪声 windup */
    bool freeze = (fh->quality_gain < FLOW_QUALITY_FREEZE_I);
    fh->pid_vx.freeze_integral = freeze;
    fh->pid_vy.freeze_integral = freeze;

    fh->out_pitch_deg = pid_update(&fh->pid_vx, fh->setpoint_vx, fh->vx_est, dt)
                      * fh->quality_gain;
    fh->out_roll_deg  = pid_update(&fh->pid_vy, fh->setpoint_vy, fh->vy_est, dt)
                      * fh->quality_gain;

    fh->active = (fh->quality_gain > 0.01f);
}

void flow_hold_set_velocity(flow_hold_t *fh, float vx, float vy)
{
    fh->setpoint_vx = vx;
    fh->setpoint_vy = vy;
}

void flow_hold_set_gyro_comp(flow_hold_t *fh, float kx, float ky)
{
    fh->gyro_kx = kx;
    fh->gyro_ky = ky;
}

void flow_hold_update(flow_hold_t *fh, int16_t flow_x, int16_t flow_y,
                      float gyro_x, float gyro_y, uint8_t qual, float height_m)
{
    bool height_ok = (height_m > 0.04f) && (height_m < FLOW_MAX_HEIGHT_M);
    bool valid     = height_ok && (qual > FLOW_QUALITY_LOW);

    /* 连续 quality 权重：qual≤LOW → 0；qual≥HIGH → 1；中间线性插值 */
    float q_target;
    if (!height_ok || qual <= FLOW_QUALITY_LOW) {
        q_target = 0.0f;
    } else if (qual >= FLOW_QUALITY_HIGH) {
        q_target = 1.0f;
    } else {
        q_target = (float)(qual - FLOW_QUALITY_LOW)
                 / (float)(FLOW_QUALITY_HIGH - FLOW_QUALITY_LOW);
    }
    fh->quality_gain += FLOW_QUALITY_SMOOTH * (q_target - fh->quality_gain);

    fh->last_update_us = esp_timer_get_time();

    if (valid) {
        /* 陀螺补偿：扣除姿态变化引起的旋转光流，只留平移分量。
         * pitch rate(gyro_y) 污染 x 轴光流，roll rate(gyro_x) 污染 y 轴 */
        float fx = (float)flow_x - fh->gyro_kx * gyro_y;
        float fy = (float)flow_y - fh->gyro_ky * gyro_x;

        /* 速度 EMA 平滑（裸 flow 噪声大） */
        fh->flow_x_f += FLOW_VEL_SMOOTH * (fx - fh->flow_x_f);
        fh->flow_y_f += FLOW_VEL_SMOOTH * (fy - fh->flow_y_f);

        /* 死区：静止小信号清零 → 防止光流残留噪声被当成真实运动 */
        float fxd = (fabsf(fh->flow_x_f) < FLOW_DEADBAND) ? 0.0f : fh->flow_x_f;
        float fyd = (fabsf(fh->flow_y_f) < FLOW_DEADBAND) ? 0.0f : fh->flow_y_f;
        fh->flow_x_comp = fh->flow_x_f;
        fh->flow_y_comp = fh->flow_y_f;

        /* 互补滤波修正：用 flow 测量把 vx_est 拉回真值
         * 修正强度按 quality_gain 缩放：低 qual 时少信任 flow */
        float k = FLOW_CORRECT_K * fh->quality_gain;
        fh->vx_est += k * (fxd - fh->vx_est);
        fh->vy_est += k * (fyd - fh->vy_est);

        fh->flow_x_corr = fxd;
        fh->flow_y_corr = fyd;
    } else {
        /* invalid 时不动 vx_est（让 IMU 通道继续推进，慢衰减自动回归）
         * 也不动 out_*（predict 仍在跑，但 quality_gain → 0 输出自然衰减） */
    }
}

void flow_hold_reset(flow_hold_t *fh)
{
    pid_reset(&fh->pid_vx);
    pid_reset(&fh->pid_vy);
    fh->setpoint_vx   = 0.0f;
    fh->setpoint_vy   = 0.0f;
    fh->out_pitch_deg = 0.0f;
    fh->out_roll_deg  = 0.0f;
    fh->quality_gain  = 0.0f;
    fh->last_update_us = 0;
    fh->active = false;
    fh->flow_x_f = 0.0f;
    fh->flow_y_f = 0.0f;
    fh->vx_est = 0.0f;
    fh->vy_est = 0.0f;
    fh->flow_x_corr = 0.0f;
    fh->flow_y_corr = 0.0f;
}

bool flow_hold_is_active(const flow_hold_t *fh)
{
    return fh->active;
}
