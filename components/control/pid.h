#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_limit;   /* 输出绝对值上限 */
    float integral_limit; /* 积分项上限（防饱和） */
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
