#pragma once

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_t   pid_vx;          /* 体轴 X 速度 → pitch 修正角 (deg) */
    pid_t   pid_vy;          /* 体轴 Y 速度 → roll 修正角 (deg) */
    float   setpoint_vx;     /* X 速度指令 (flow 原始单位) */
    float   setpoint_vy;     /* Y 速度指令 (flow 原始单位) */
    float   out_pitch_deg;   /* 最新 pitch 修正角 */
    float   out_roll_deg;    /* 最新 roll 修正角 */
    float   quality_gain;    /* 光流质量 EMA 平滑值 0~1 */
    int64_t last_update_us;  /* 上次 flow 校正的时间戳 */
    bool    active;          /* 是否正在输出有效修正 */
    float   gyro_kx, gyro_ky;         /* 陀螺补偿系数：flow 单位 per (rad/s) */
    float   flow_x_f, flow_y_f;       /* 速度 EMA 滤波状态 */
    float   flow_x_comp, flow_y_comp; /* 平滑+死区后的光流（遥测/标定用） */
    /* --- 互补滤波速度估计（flow 单位） ---
     * predict 每帧调（100Hz）：IMU 加速度积分 + PID 输出
     * update 仅在新 flow 帧（50Hz）：陀螺补偿后修正 vx_est/vy_est */
    float   vx_est, vy_est;           /* IMU+flow 互补的速度估计（flow 单位） */
    float   imu_scale;                /* m/s² → flow_unit/s² 转换系数（运行时标定） */
    float   flow_x_corr, flow_y_corr; /* 上次校正用的 flow 速度（用于 telemetry/调试） */
} flow_hold_t;

void flow_hold_init(flow_hold_t *fh);
void flow_hold_set_velocity(flow_hold_t *fh, float vx, float vy);
void flow_hold_set_gyro_comp(flow_hold_t *fh, float kx, float ky);
void flow_hold_set_imu_scale(flow_hold_t *fh, float scale);

/**
 * @brief 每帧调用（100Hz）：IMU 加速度积分推进 vx_est，跑 PID 算修正角。
 *        accel 应该是世界系水平加速度（m/s²，已扣除重力分量）；
 *        小角度悬停下直接用机体系 accel_x/y 即可。
 */
void flow_hold_predict(flow_hold_t *fh, float ax_world, float ay_world, float dt);

/**
 * @brief 有新光流帧时调用（~50Hz）：陀螺补偿 + 平滑 + 互补滤波校正 vx_est。
 *        不再跑 PID（已移到 predict 中）。
 */
void flow_hold_update(flow_hold_t *fh, int16_t flow_x, int16_t flow_y,
                      float gyro_x, float gyro_y, uint8_t qual, float height_m);

void flow_hold_reset(flow_hold_t *fh);
bool flow_hold_is_active(const flow_hold_t *fh);

#ifdef __cplusplus
}
#endif
