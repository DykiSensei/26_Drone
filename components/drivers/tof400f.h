#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOF400F_ADDR        0x29

/**
 * @brief Initialize TOF400F: probe VL53L1X directly on I2C bus
 * @return 0 success, -1 failure
 */
int tof400f_init(void);

/**
 * @brief Read distance in mm
 * @param distance_mm output distance; holds last valid on failure
 * @return 0 valid reading, -1 stale/fallback
 */
int tof400f_get_distance(uint16_t *distance_mm);

#ifdef __cplusplus
}
#endif
