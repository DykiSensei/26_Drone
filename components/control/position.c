#include "position.h"
#include <math.h>

/* 位置环 PID 参数 —— 米制：位置误差 (m) → 速度 setpoint (m/s)。
 * KP 量纲是 1/s，增益含义不再随悬停高度漂移。 */
#define POS_KP         1.5f   /* 10cm 误差 → 0.15 m/s 温和拉回 */
#define POS_KI         0.15f  /* 慢积分消除稳态误差 */
#define POS_KD         0.0f
#define POS_OUT_LIMIT  0.5f   /* 速度 setpoint 上限 (m/s)：压在光流可测量级内，
                               * 避免速度环长期饱和 */
#define POS_INT_LIMIT  1.0f   /* 积分状态上限 (m·s)：KI×1.0 = 0.15 m/s 权限 */

/* 到达目标判定 */
#define POS_TOLERANCE  0.05f     /* 位置误差容差 (m) */
#define POS_REACHED_CNT 10       /* 连续到达周期数确认（防单帧噪声误判） */

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
                         float current_x_m, float current_y_m)
{
    pos->start_x  = current_x_m;
    pos->start_y  = current_y_m;
    pos->target_x = current_x_m + offset_x;
    pos->target_y = current_y_m + offset_y;
    pos->reached_count = 0;
    pos->active   = true;
    pos->hold     = false;
    pid_reset(&pos->pid_x);
    pid_reset(&pos->pid_y);
}

void position_hold_start(position_ctrl_t *pos, float current_x_m, float current_y_m)
{
    pos->start_x  = pos->target_x = current_x_m;
    pos->start_y  = pos->target_y = current_y_m;
    pos->reached_count = 0;
    pos->active   = true;
    pos->hold     = true;
    pid_reset(&pos->pid_x);
    pid_reset(&pos->pid_y);
}

void position_update(position_ctrl_t *pos, float x_m, float y_m, float dt)
{
    if (!pos->active) {
        pos->out_vx = 0.0f;
        pos->out_vy = 0.0f;
        return;
    }

    float err_x = pos->target_x - x_m;
    float err_y = pos->target_y - y_m;

    /* move_to 到达判定防抖：连续 POS_REACHED_CNT 周期都在容差内才算到，
     * 避免单帧噪声擦线触发提前转 hold */
    if (!pos->hold) {
        if (fabsf(err_x) < POS_TOLERANCE && fabsf(err_y) < POS_TOLERANCE)
            pos->reached_count++;
        else
            pos->reached_count = 0;
    }

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

bool position_reached(const position_ctrl_t *pos)
{
    if (!pos->active) return true;
    return pos->reached_count >= POS_REACHED_CNT;
}
