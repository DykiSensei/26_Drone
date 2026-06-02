#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDR    0x68

typedef struct {
    float accel_x, accel_y, accel_z;   /* m/s^2 */
    float gyro_x,  gyro_y,  gyro_z;    /* rad/s  */
    float temp_c;                       /* temperature */
    bool  data_ready;
} mpu6050_data_t;

/**
 * @brief 初始化 MPU6050：挂载到 I2C0，配置量程和采样率，校准零偏
 * @return 0 成功, -1 失败
 */
int mpu6050_init(void);

/**
 * @brief 重新校准陀螺仪零偏（起飞前调用，无人机必须静止放置）
 * @return 0 成功, -1 失败
 */
int mpu6050_recalibrate_gyro(void);

/**
 * @brief 读取传感器数据（已扣除零偏）
 * @param out 输出数据结构体
 * @return 0 成功, -1 失败
 */
int mpu6050_read(mpu6050_data_t *out);

#ifdef __cplusplus
}
#endif
