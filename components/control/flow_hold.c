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
#define FLOW_QUALITY_THRESHOLD 50
#define FLOW_MAX_HEIGHT_M    3.0f
#define FLOW_QUALITY_SMOOTH  0.3f

#define DT_MIN  0.005f
#define DT_MAX  0.200f
#define DECAY   0.95f

#define FLOW_VEL_SMOOTH 0.3f   /* 速度 EMA 平滑系数 */
#define FLOW_DEADBAND   1.0f   /* 速度死区：静止残留≈0，设小以免清掉真实小漂移信号 */

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
    bool valid = (qual > FLOW_QUALITY_THRESHOLD)
              && (height_m > 0.04f)
              && (height_m < FLOW_MAX_HEIGHT_M);

    /* EMA smooth of quality signal */
    float q_target = valid ? 1.0f : 0.0f;
    fh->quality_gain += FLOW_QUALITY_SMOOTH * (q_target - fh->quality_gain);

    int64_t now = esp_timer_get_time();

    if (valid) {
        float dt;
        if (fh->last_update_us == 0) {
            dt = 0.01f;  /* first frame, use nominal dt */
        } else {
            dt = (float)(now - fh->last_update_us) * 1e-6f;
        }
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;

        /* Flow is velocity; setpoint = 0 means "resist any motion".
         * measurement = +flow (bench-verified): forward drift (flow_x>0) →
         * out_pitch = KP*(0-flow_x) < 0. pitch stick + = forward, so a negative
         * target pitch tilts backward and cancels the forward drift.
         * Active move: setpoint>0 drives forward until flow_x tracks it. */
        /* 陀螺补偿：扣除姿态变化引起的旋转光流，只留平移分量。
         * pitch rate(gyro_y) 污染 x 轴光流，roll rate(gyro_x) 污染 y 轴。 */
        float fx = (float)flow_x - fh->gyro_kx * gyro_y;
        float fy = (float)flow_y - fh->gyro_ky * gyro_x;

        /* 速度 EMA 平滑（裸 flow 噪声大） */
        fh->flow_x_f += FLOW_VEL_SMOOTH * (fx - fh->flow_x_f);
        fh->flow_y_f += FLOW_VEL_SMOOTH * (fy - fh->flow_y_f);

        /* 死区：静止小信号清零——防止光流残留噪声/偏置(陀螺补偿不可能完美)被
         * 速度环当成真实运动而持续纠偏漂移。这是光流定点的标准防漂手段
         * (参考 LiteWing/iNav)。死区内不响应，避免"追假速度"飞走。 */
        float fxd = (fabsf(fh->flow_x_f) < FLOW_DEADBAND) ? 0.0f : fh->flow_x_f;
        float fyd = (fabsf(fh->flow_y_f) < FLOW_DEADBAND) ? 0.0f : fh->flow_y_f;
        fh->flow_x_comp = fh->flow_x_f;  /* 遥测显示平滑后(死区前)，用于标定死区阈值 */
        fh->flow_y_comp = fh->flow_y_f;
        fh->out_pitch_deg = pid_update(&fh->pid_vx, fh->setpoint_vx, fxd, dt);
        fh->out_roll_deg  = pid_update(&fh->pid_vy, fh->setpoint_vy, fyd, dt);
    } else {
        /* Fade corrections smoothly when quality drops */
        fh->out_pitch_deg *= DECAY;
        fh->out_roll_deg  *= DECAY;
    }

    fh->last_update_us = now;
    fh->active = (fh->quality_gain > 0.01f);
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
}

bool flow_hold_is_active(const flow_hold_t *fh)
{
    return fh->active;
}
