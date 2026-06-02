#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Motor channel count */
#define MOTOR_COUNT 4

/**
 * @brief Initialize 4-channel LEDC PWM for ESC control.
 *        Outputs MIN (1000 us) to all motors — safe startup.
 * @return 0 on success, -1 on failure
 */
int motor_init(void);

/**
 * @brief Set throttle for all 4 motors.
 * @param motor[4]  throttle values 0.0 (stopped) – 1.0 (max)
 */
void motor_set(const float motor[MOTOR_COUNT]);

/**
 * @brief Stop all motors (set to MIN pulse).
 */
void motor_stop(void);

/**
 * @brief ESC throttle-range calibration (all 4 motors).
 */
void motor_calibrate(void);

/**
 * @brief Calibrate a single ESC by index (0=FR, 1=FL, 2=RL, 3=RR).
 *        Only that motor outputs the calibration sequence.
 */
void motor_calibrate_single(int index);

#ifdef __cplusplus
}
#endif
