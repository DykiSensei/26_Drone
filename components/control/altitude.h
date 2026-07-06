#pragma once

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_t   pid;            /* altitude P+I (input: m, output: throttle adjust) */
    float   target_m;       /* current (ramped) target altitude in meters */
    float   target_final_m; /* final target the ramp converges toward */
    bool    target_valid;   /* true when target has been captured */
    float   prev_m;         /* last TOF reading (m), to detect fresh samples */
    float   vz;             /* estimated vertical speed (m/s, climbing = +) */
    int64_t last_change_us; /* timestamp of last measurement change */
    float   az_lp;          /* az_up 直流跟踪器 (慢 LP)：残余零偏 + 飞行中振动
                             * 整流偏移。vz 只积分高通分量 (az_up − az_lp)。
                             * ⚠️ 连续运行，altitude_reset 不清零（估计器设计
                             * 规则：输入调理滤波器绝不逐次复位） */
    bool    az_lp_init;     /* 首样本直接锁存，跳过从 0 收敛的 2s 瞬态 */
} altitude_ctrl_t;

/**
 * @brief Initialize altitude controller
 */
void altitude_init(altitude_ctrl_t *alt);

/**
 * @brief Capture current TOF reading as target altitude (call on mode entry).
 *        Final target == current, so the ramp stays put (hold in place).
 * @param current_m  current TOF distance in meters
 */
void altitude_capture_target(altitude_ctrl_t *alt, float current_m);

/**
 * @brief Set a final target the controller ramps toward from current height.
 *        Use for takeoff: the (ramped) target climbs smoothly from the
 *        ground to final_m, keeping the height error small the whole way.
 * @param final_m    desired final altitude in meters
 * @param current_m  current TOF distance in meters (ramp start point)
 */
void altitude_set_target(altitude_ctrl_t *alt, float final_m, float current_m);

/**
 * @brief az_up 直流跟踪（tau≈2s），主循环每拍调用——**任意模式，包括 DISARMED**。
 *        地面静止时收敛到 accel-Z 残余零偏，飞行中缓慢吸收电机振动整流造成的
 *        直流下移；vz 积分只用高通分量，消除 Vz 常值偏移
 *        （该偏移曾让起飞位置锁的 |vz|<0.15 门永远过不去）。
 * @param az_up  世界系垂直加速度，扣标准重力 (m/s², 向上为正)
 */
void altitude_track_az(altitude_ctrl_t *alt, float az_up, float dt);

/**
 * @brief Run altitude PID (P+I + vz damping via IMU/TOF complementary filter)
 * @param alt        controller instance
 * @param current_m  current TOF distance in meters
 * @param az_up      world-up acceleration, gravity removed (m/s², up = +)
 * @param dt         time step in seconds
 * @return throttle adjustment (-output_limit .. +output_limit)
 */
float altitude_update(altitude_ctrl_t *alt, float current_m, float az_up, float dt);

/**
 * @brief Reset PID integrator (call when disarmed or landed)
 */
void altitude_reset(altitude_ctrl_t *alt);

#ifdef __cplusplus
}
#endif
