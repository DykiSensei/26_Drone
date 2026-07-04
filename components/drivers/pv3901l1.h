#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PV3901L1_UART_PORT       UART_NUM_1
#define PV3901L1_RX_PIN          44
#define PV3901L1_YAW_MODE_PIN    15
#define PV3901L1_BAUDRATE        19200
#define PV3901L1_UART_BUF_SIZE   256

typedef enum {
    FLOW_MODE_NORMAL = 0,
    FLOW_MODE_YAW    = 1,
} flow_yaw_mode_t;

typedef struct {
    int16_t flow_x;          /* 当前帧X方向原始位移（遥测/调试） */
    int16_t flow_y;          /* 当前帧Y方向原始位移（遥测/调试） */
    int32_t acc_x;           /* 自上次 get_data 起累计位移 counts —— 控制用。
                              * UART 批量到达与主循环消费不同步时，"最新帧"
                              * 语义会丢掉批内其余帧（曾丢 ~95% → 速度缩水
                              * 十几倍且随机），累计语义无论怎么攒批都精确 */
    int32_t acc_y;
    float   flow_x_i;        /* X方向积分位移 */
    float   flow_y_i;        /* Y方向积分位移 */
    uint8_t qual;            /* 数据质量 (0–255) */
    uint32_t frame_count;    /* 有效帧计数 */
    uint32_t error_count;    /* 校验失败计数 */
    bool     data_ready;     /* 新数据就绪标志 */
} pv3901l1_data_t;

/**
 * @brief 初始化光流模块：UART + YAW_MODE GPIO
 * @return 0 成功, -1 失败
 */
int pv3901l1_init(void);

/**
 * @brief 获取最新光流数据（拷贝到用户提供的结构体）
 * @param out 输出数据指针
 * @return 0 有新数据, -1 无新数据
 */
int pv3901l1_get_data(pv3901l1_data_t *out);

/**
 * @brief 清零积分位移
 */
void pv3901l1_reset_integral(void);

/**
 * @brief 设置偏航模式
 * @param mode FLOW_MODE_NORMAL 或 FLOW_MODE_YAW
 */
void pv3901l1_set_yaw_mode(flow_yaw_mode_t mode);

#ifdef __cplusplus
}
#endif
