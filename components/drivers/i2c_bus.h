#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C0_SDA    9
#define I2C0_SCL    8

extern i2c_master_bus_handle_t g_i2c0_bus;

/**
 * @brief 初始化 I2C0 总线（所有 I2C 设备共享，只调用一次）
 * @return 0 成功, -1 失败
 */
int i2c_bus_init(void);

#ifdef __cplusplus
}
#endif
