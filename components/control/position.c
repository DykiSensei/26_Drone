#include "position.h"
#include <math.h>

/* 位置环 PID 参数 */
#define POS_KP         0.15f  /* 位置误差 → 速度 setpoint。实测悬停级光流速度
                               * 只有个位数（30cm 平移 comp<10），0.8 会在几 cm
                               * 误差时就把速度环打到 ±8° 满舵 → 围绕锁点振荡
                               * 永不收敛。0.15: 10cm 误差 ≈ 温和拉回速度。 */
#define POS_KI         0.02f  /* 慢积分消除稳态误差 */
#define POS_KD         0.0f
#define POS_OUT_LIMIT  20.0f  /* 速度 setpoint 上限：压在光流可测的真实速度
                               * 量级内，避免速度环长期饱和（手动按钮仍 ±80）。 */
#define POS_INT_LIMIT  30.0f

/* 到达目标判定 */
#define POS_TOLERANCE  20.0f     /* 位置误差容差 (flow 积分单位) */
#define POS_REACHED_CNT 10       /* 连续到达周期数确认 */

void position_init(position_ctrl_t *pos)
{
    pid_init(&pos->pid_x, POS_KP, POS_KI, POS_KD,
             POS_OUT_LIMIT, POS_INT_LIMIT);
    pid_init(&pos->pid_y, POS_KP, POS_KI, POS_KD,
             POS_OUT_LIMIT, POS_INT_LIMIT);
    pos->target_x = 0.0f;
    pos->target_y = 0.0f;
    pos->start_x  = 0.0f;
    pos->start_y  = 0.0f;
    pos->out_vx   = 0.0f;
    pos->out_vy   = 0.0f;
    pos->active   = false;
    pos->hold     = false;
    pos->reached_count = 0;
}

void position_set_target(position_ctrl_t *pos, float offset_x, float offset_y,
                         float current_flow_ix, float current_flow_iy)
{
    pos->start_x  = current_flow_ix;
    pos->start_y  = current_flow_iy;
    pos->target_x = current_flow_ix + offset_x;
    pos->target_y = current_flow_iy + offset_y;
    pos->reached_count = 0;
    pos->active   = true;
    pos->hold     = false;
    pid_reset(&pos->pid_x);
    pid_reset(&pos->pid_y);
}

void position_hold_start(position_ctrl_t *pos, float current_ix, float current_iy)
{
    pos->start_x  = pos->target_x = current_ix;
    pos->start_y  = pos->target_y = current_iy;
    pos->reached_count = 0;
    pos->active   = true;
    pos->hold     = true;
    pid_reset(&pos->pid_x);
    pid_reset(&pos->pid_y);
}

void position_update(position_ctrl_t *pos, float flow_ix, float flow_iy, float dt)
{
    if (!pos->active) {
        pos->out_vx = 0.0f;
        pos->out_vy = 0.0f;
        return;
    }

    float err_x = pos->target_x - flow_ix;
    float err_y = pos->target_y - flow_iy;

    pos->out_vx = pid_update(&pos->pid_x, 0.0f, -err_x, dt); /* setpoint=0, measurement=-err */
    pos->out_vy = pid_update(&pos->pid_y, 0.0f, -err_y, dt);
}

void position_reset(position_ctrl_t *pos)
{
    pid_reset(&pos->pid_x);
    pid_reset(&pos->pid_y);
    pos->target_x = 0.0f;
    pos->target_y = 0.0f;
    pos->start_x  = 0.0f;
    pos->start_y  = 0.0f;
    pos->out_vx   = 0.0f;
    pos->out_vy   = 0.0f;
    pos->active   = false;
    pos->hold     = false;
    pos->reached_count = 0;
}

bool position_reached(const position_ctrl_t *pos, float current_ix, float current_iy)
{
    if (!pos->active) return true;

    float err_x = fabsf(pos->target_x - current_ix);
    float err_y = fabsf(pos->target_y - current_iy);

    return (err_x < POS_TOLERANCE && err_y < POS_TOLERANCE);
}
