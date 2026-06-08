#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_meas;      /* 上次测量值（derivative-on-measurement） */
    float d_filt;         /* D 项 EMA 平滑后的值 */
    float output_limit;   /* 输出绝对值上限 */
    float integral_limit; /* 积分项上限（防饱和） */
    bool  freeze_integral; /* true=本帧不积分（用于地效区抗 windup 等外部场景） */
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit);

/* 重置积分和历史误差 */
void pid_reset(pid_t *pid);

/* 更新 PID，返回控制量 */
float pid_update(pid_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif
