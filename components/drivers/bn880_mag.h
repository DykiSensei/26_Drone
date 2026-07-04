#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BN-880 模块的磁力计部分（GPS 走 UART，驱动暂未实现）。
 * 不同批次 BN-880 出厂搭载两种磁力计芯片之一，驱动自动探测：
 *   QMC5883L @ 0x0D（国产替代，多数新批次）
 *   HMC5883L @ 0x1E（Honeywell 原厂，老批次）
 * 挂载在共享 I2C0 总线（SDA=GPIO9, SCL=GPIO8），与 MPU6050/TOF400F 共存。 */

#define QMC5883L_ADDR  0x0D
#define HMC5883L_ADDR  0x1E

typedef struct {
    float x, y, z;   /* 磁场强度（高斯，模块自身轴系——尚未对齐机体系，
                      * 轴系对齐与硬磁/软磁标定在 Mahony 九轴集成时做） */
    bool  valid;
} bn880_mag_data_t;

/**
 * @brief 探测并初始化磁力计（先试 QMC5883L@0x0D，再试 HMC5883L@0x1E）。
 *        配置为连续测量模式（~50-75Hz）。
 * @return 0 成功；-1 两个地址都没找到（非致命——飞控应能在无磁力计时正常工作）
 */
int bn880_mag_init(void);

/**
 * @brief 读取最新磁场数据（高斯）。未初始化/读失败时 out->valid=false 并返回 -1
 */
int bn880_mag_read(bn880_mag_data_t *out);

/** 探测到的芯片名（"QMC5883L"/"HMC5883L"/"none"，遥测与日志用） */
const char *bn880_mag_chip_name(void);

#ifdef __cplusplus
}
#endif
