#include "pid.h"

/* D 项一阶 EMA 低通：截止≈20Hz @ 100Hz (α=2π·fc·dt/(1+2π·fc·dt))
 * 陀螺噪声 + 电机振动混叠 → 微分项会放大成尖刺，让 D 增益必须取很小。
 * 加这道滤波后 D 项才能起阻尼作用而不抖电机 */
#define PID_D_FILT_ALPHA  0.5f

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->freeze_integral = false;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_filt = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* 积分（freeze_integral 期间冻结，避免地效区/启动期等场景 windup） */
    if (!pid->freeze_integral) {
        pid->integral += error * dt;
        if (pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }

    /* 微分（对测量值微分避免 setpoint 跳变引起冲击） + EMA 低通 */
    float d_meas = (measurement - pid->prev_meas) / dt;
    pid->prev_meas = measurement;
    pid->d_filt += PID_D_FILT_ALPHA * (d_meas - pid->d_filt);

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 - pid->kd * pid->d_filt;

    if (output > pid->output_limit)  output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
