#include "flow_hold.h"
#include "esp_timer.h"
#include <math.h>

/* 速度环 PID —— 全链路米制化后增益含义固定：修正角(deg) per 速度误差(m/s)。
 * 旧实现直接消费 flow 原始单位，同样的物理速度在 10cm 高度产生的读数是
 * 50cm 高度的 5 倍 → 等效环增益随高度漂移 5~10 倍，定点只在调参高度稳、
 * 起飞爬升期（高度剧变）必然失配。 */
#define FLOW_KP              8.0f   /* 0.2 m/s 漂移 → 1.6° 纠偏 */
#define FLOW_KI              1.2f   /* 积分消除持续漂移/机身固有偏移 */
#define FLOW_KD              0.0f
#define FLOW_OUTPUT_LIMIT    8.0f   /* 最大 ±8° 修正角 */
#define FLOW_INTEGRAL_LIMIT  3.0f   /* 积分状态上限 (m/s·s)：KI×3 ≈ 3.6° 权限 */
/* 连续 quality 门限：30 起步介入（低权重），80 满权。
 * 原 binary 50 切断在 qual 30-50 徘徊时完全无位置控制 → 漂走才锁。 */
#define FLOW_QUALITY_LOW     30
#define FLOW_QUALITY_HIGH    80
#define FLOW_QUALITY_FREEZE_I  0.5f /* quality_gain 低于此值冻结 PID 积分防 windup */
#define FLOW_MAX_HEIGHT_M    3.0f
#define FLOW_QUALITY_SMOOTH  0.3f

#define FLOW_VEL_SMOOTH 0.3f    /* 速度 EMA 平滑系数 */
#define FLOW_DEADBAND   0.02f   /* m/s 死区：抗陀螺补偿残差噪声，2cm/s 以下视为静止 */

/* --- IMU + 光流 互补滤波参数 ---
 * 高频快通道：IMU 加速度积分（短期响应快，长期偏置漂移）
 * 低频绝对通道：光流米制速度（离散帧，绝对参考但延迟）
 * 米制统一后两通道量纲一致（m/s），不再需要经验性的 imu_scale。 */
#define IMU_LEAK         0.999f  /* 每帧慢衰减，防 accel 偏置积爆（τ≈10s @100Hz） */
#define FLOW_CORRECT_K   0.30f   /* flow 修正强度：新 flow 帧把 vx_est 拉向测量值的比例 */

/* 米制换算：v(m/s) = counts × FLOW_SCALE(rad/count) × height(m) / dt_frame(s)。
 * PMW3901 系光学参数（4.2° FOV / 30 px）≈ 0.00244 rad/count，
 * PV3901L1 疑似同系，实际系数经 {"cmd":"flow_comp","scale":..} 试飞标定。 */
#define FLOW_SCALE_DEFAULT   0.00244f
#define FRAME_DT_MIN         0.005f  /* 帧间隔钳位 (s) */
#define FRAME_DT_MAX         0.05f
#define FRAME_DT_DEFAULT     0.01f

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
    fh->flow_scale = FLOW_SCALE_DEFAULT;
    fh->flow_x_f = 0.0f;
    fh->flow_y_f = 0.0f;
    fh->flow_x_comp = 0.0f;
    fh->flow_y_comp = 0.0f;
    fh->vx_est = 0.0f;
    fh->vy_est = 0.0f;
    fh->pos_x_m = 0.0f;
    fh->pos_y_m = 0.0f;
    fh->flow_x_corr = 0.0f;
    fh->flow_y_corr = 0.0f;
}

void flow_hold_set_flow_scale(flow_hold_t *fh, float scale)
{
    if (scale > 0.0f) fh->flow_scale = scale;
}

void flow_hold_predict(flow_hold_t *fh, float ax_world, float ay_world, float dt)
{
    /* IMU 加速度积分 + 慢衰减（防 accel 偏置 / 长期漂移积爆）。米制统一后
     * m/s² × s 直接得 m/s，与光流通道量纲一致。 */
    fh->vx_est = (fh->vx_est + ax_world * dt) * IMU_LEAK;
    fh->vy_est = (fh->vy_est + ay_world * dt) * IMU_LEAK;

    /* 航位推算位置（m）：积分融合速度。position 环用它做锁定反馈——
     * 比旧的裸 flow 积分好在：米制（不随高度变尺度）、含 IMU 高频信息、
     * 低质量时 vx_est 自然衰减 → 位置冻结而不是积累噪声。 */
    fh->pos_x_m += fh->vx_est * dt;
    fh->pos_y_m += fh->vy_est * dt;

    /* PID 100Hz 更新（比 flow 帧率高的带宽，能跟住 IMU 高频信息）。
     * 低 quality 时冻结积分，避免噪声 windup。
     * 输出取反（关键符号）：本机实测 前推 fx>0、右推 fy>0（取决于光流模块
     * 安装方向，换装后需重新核对），即 前/右 = 光流正方向。而正 pitch 角 =
     * 抬头 = 向后加速、正 roll 角 = 右侧抬 = 向左加速 —— 正修正角产生负方向
     * 加速度。前漂 (vx_est>0) 必须抬头 (+pitch) 拦截，pid(0, +v) 输出为负，
     * 故取反。不取反则速度环为正反馈：漂移方向被加速 → 定点直接漂走。 */
    bool freeze = (fh->quality_gain < FLOW_QUALITY_FREEZE_I);
    fh->pid_vx.freeze_integral = freeze;
    fh->pid_vy.freeze_integral = freeze;

    fh->out_pitch_deg = -pid_update(&fh->pid_vx, fh->setpoint_vx, fh->vx_est, dt)
                      * fh->quality_gain;
    fh->out_roll_deg  = -pid_update(&fh->pid_vy, fh->setpoint_vy, fh->vy_est, dt)
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

    /* 帧间隔实测（米制换算的分母）：模块帧率会随环境变化，不能假定常数 */
    int64_t now = esp_timer_get_time();
    float dt_frame = (fh->last_update_us == 0)
                   ? FRAME_DT_DEFAULT
                   : (float)(now - fh->last_update_us) * 1e-6f;
    if (dt_frame < FRAME_DT_MIN) dt_frame = FRAME_DT_MIN;
    if (dt_frame > FRAME_DT_MAX) dt_frame = FRAME_DT_MAX;
    fh->last_update_us = now;

    if (valid) {
        /* 陀螺补偿：在 counts/帧 域内做（kx/ky 为该域的实测标定值）。
         * pitch rate(gyro_y) 污染 x 轴光流，roll rate(gyro_x) 污染 y 轴 */
        float fx = (float)flow_x - fh->gyro_kx * gyro_y;
        float fy = (float)flow_y - fh->gyro_ky * gyro_x;

        /* 米制换算：v = counts × (rad/count) × 高度 / 帧间隔 */
        float k_m = fh->flow_scale * height_m / dt_frame;
        float vx_m = fx * k_m;
        float vy_m = fy * k_m;

        /* 速度 EMA 平滑（裸 flow 噪声大） */
        fh->flow_x_f += FLOW_VEL_SMOOTH * (vx_m - fh->flow_x_f);
        fh->flow_y_f += FLOW_VEL_SMOOTH * (vy_m - fh->flow_y_f);

        /* 死区：静止小信号清零 → 防止光流残留噪声被当成真实运动 */
        float fxd = (fabsf(fh->flow_x_f) < FLOW_DEADBAND) ? 0.0f : fh->flow_x_f;
        float fyd = (fabsf(fh->flow_y_f) < FLOW_DEADBAND) ? 0.0f : fh->flow_y_f;
        fh->flow_x_comp = fh->flow_x_f;
        fh->flow_y_comp = fh->flow_y_f;

        /* 互补滤波修正：用 flow 测量把 vx_est 拉回真值。
         * 修正强度按 quality_gain 缩放：低 qual 时少信任 flow。
         * 即使模块输出 0（连续两帧位移太小），fxd 就是 0 → vx_est 会被
         * 拉向 0（衰减），同时 IMU 积分仍在 predict 阶段推进。 */
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
    /* 注意：不复位 gyro_kx/ky 和 flow_scale —— 都是标定值，跨解锁保留 */
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
    fh->pos_x_m = 0.0f;
    fh->pos_y_m = 0.0f;
    fh->flow_x_corr = 0.0f;
    fh->flow_y_corr = 0.0f;
}

bool flow_hold_is_active(const flow_hold_t *fh)
{
    return fh->active;
}
