#pragma once

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_t   pid_vx;          /* 体轴 X 速度 (m/s) → pitch 修正角 (deg) */
    pid_t   pid_vy;          /* 体轴 Y 速度 (m/s) → roll 修正角 (deg) */
    float   setpoint_vx;     /* X 速度指令 (m/s) */
    float   setpoint_vy;     /* Y 速度指令 (m/s) */
    float   out_pitch_deg;   /* 最新 pitch 修正角 */
    float   out_roll_deg;    /* 最新 roll 修正角 */
    float   quality_gain;    /* 光流质量 EMA 平滑值 0~1 */
    int64_t last_update_us;  /* 上次 flow 帧时间戳（也用于测量帧间隔） */
    bool    active;          /* 是否正在输出有效修正 */
    float   gyro_kx, gyro_ky;         /* 陀螺补偿系数（米制域，无量纲，标称 ±1.0：
                                       * 补偿量 = k × ω × 高度，tilt 测试定符号） */
    float   flow_scale;               /* 米制换算系数 rad/count（PMW3901 系 ≈0.00244，可运行时标定）*/
    float   flow_x_f, flow_y_f;       /* 光流速度 EMA 滤波状态 (m/s) */
    float   flow_x_comp, flow_y_comp; /* 补偿+平滑后的光流速度 (m/s)（遥测/标定用） */
    /* --- 互补滤波速度估计（m/s，统一米制后与 IMU 加速度量纲一致） ---
     * predict 每帧调（100Hz）：IMU 加速度积分 + PID 输出
     * update 仅在新 flow 帧（~50Hz）：陀螺补偿 → 米制换算 → 修正 vx_est/vy_est */
    float   vx_est, vy_est;           /* IMU+flow 互补的速度估计 (m/s) */
    float   pos_x_m, pos_y_m;         /* 航位推算位置 (m)：积分 vx_est。position 环的反馈源 */
    float   flow_x_corr, flow_y_corr; /* 上次校正用的 flow 速度 (m/s)（telemetry/调试） */
} flow_hold_t;

void flow_hold_init(flow_hold_t *fh);
void flow_hold_set_velocity(flow_hold_t *fh, float vx, float vy);
void flow_hold_set_gyro_comp(flow_hold_t *fh, float kx, float ky);

/** 设置米制换算系数 rad/count（>0 才生效），经 {"cmd":"flow_comp","scale":..} 运行时标定 */
void flow_hold_set_flow_scale(flow_hold_t *fh, float scale);

/**
 * @brief 每帧调用（100Hz）：IMU 加速度积分推进 vx_est（m/s），积分出 pos_x_m，
 *        并跑速度 PID 算修正角。accel 应是世界系水平加速度（m/s²，已扣重力分量）；
 *        小角度悬停下直接用机体系 accel_x/y 即可。
 */
void flow_hold_predict(flow_hold_t *fh, float ax_world, float ay_world, float dt);

/**
 * @brief 有新光流帧时调用（~50-100Hz）：陀螺补偿（counts 域）→ 按 TOF 高度换算
 *        米制速度 v = counts × flow_scale × height / dt_frame → 互补滤波校正 vx_est。
 *        米制化后环路增益不再随悬停高度漂移（这是定点不稳/起飞漂移的主根因）。
 */
void flow_hold_update(flow_hold_t *fh, int16_t flow_x, int16_t flow_y,
                      float gyro_x, float gyro_y, uint8_t qual, float height_m);

void flow_hold_reset(flow_hold_t *fh);
bool flow_hold_is_active(const flow_hold_t *fh);

#ifdef __cplusplus
}
#endif
