#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* 默认/边界（仍允许超出 update 钳位，但 NVS 加载时验证） */
#define HOVER_THROTTLE_DEFAULT  0.40f
#define HOVER_THROTTLE_MIN      0.20f
#define HOVER_THROTTLE_MAX      0.65f

#define FLOW_IMU_SCALE_DEFAULT  1.0f
#define FLOW_IMU_SCALE_MIN      0.0f
#define FLOW_IMU_SCALE_MAX      100.0f

/**
 * @brief 启动时读 NVS 拿持久化的参数（悬停油门 + 光流互补滤波 scale）。
 *        NVS 没值时使用默认。必须在 wifi_ap_init() 之后调用（NVS 已被 wifi 初始化）。
 * @return 0 = OK
 */
int params_init(void);

/**
 * @brief 获取当前学习到的悬停油门（飞行中 alt PID 起飞前馈用）
 */
float params_get_hover_throttle(void);

/**
 * @brief 慢速 EMA 学习：稳定悬停时调用，传入当前真实有效油门。
 *        外部判定稳态（alt_mode + 高度误差小 + vz 小）后才调用。
 *        钳位防异常值污染学习。
 */
void params_update_hover_throttle(float actual);

/**
 * @brief 获取当前持久化的 IMU+flow 互补滤波 scale
 */
float params_get_flow_imu_scale(void);

/**
 * @brief 直接设置 scale（覆盖式，非 EMA 学习——用户每次发 flow_scale 命令就是想用这个值）。
 *        外部钳位到合理范围，main 每帧调用同步 setpoint 值；params 内部判断"和当前不同才标脏"。
 */
void params_set_flow_imu_scale(float s);

/**
 * @brief 写 NVS（仅当与上次保存值差距超阈值时实际写入，减少 flash wear）。
 *        会一并检查所有字段是否脏。DISARMED 时调用即可。
 */
void params_save(void);

#ifdef __cplusplus
}
#endif
