#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* 积分 */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    /* 微分（对测量值微分避免 setpoint 跳变引起冲击） */
    float d_meas = (measurement - pid->prev_error) / dt;
    pid->prev_error = measurement;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 - pid->kd * d_meas;

    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
