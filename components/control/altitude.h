#pragma once

#include "pid.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_t  pid;            /* altitude PID (input: m, output: throttle adjust) */
    float  target_m;       /* target altitude in meters */
    bool   target_valid;   /* true when target has been captured */
} altitude_ctrl_t;

/**
 * @brief Initialize altitude controller
 */
void altitude_init(altitude_ctrl_t *alt);

/**
 * @brief Capture current TOF reading as target altitude (call on mode entry)
 * @param current_m  current TOF distance in meters
 */
void altitude_capture_target(altitude_ctrl_t *alt, float current_m);

/**
 * @brief Run altitude PID
 * @param alt        controller instance
 * @param current_m  current TOF distance in meters
 * @param dt         time step in seconds
 * @return throttle adjustment (-output_limit .. +output_limit)
 */
float altitude_update(altitude_ctrl_t *alt, float current_m, float dt);

/**
 * @brief Reset PID integrator (call when disarmed or landed)
 */
void altitude_reset(altitude_ctrl_t *alt);

#ifdef __cplusplus
}
#endif
