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
