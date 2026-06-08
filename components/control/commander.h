#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMMAND_TIMEOUT_US  500000   /* 500ms — auto-DISARM if no command received */

typedef enum {
    MODE_DISARMED  = 0,
    MODE_STABILIZE = 1,
    MODE_ALT_HOLD  = 2,
    MODE_POS_HOLD  = 3,
} flight_mode_t;

/** Command types that can be sent from the web frontend */
typedef enum {
    CMD_NONE = 0,
    CMD_CALIBRATE,        /* ESC throttle calibration */
    CMD_GYRO_CALIB,       /* Gyroscope zero-bias recalibration */
    CMD_LEVEL_TRIM,       /* Capture current attitude as level trim */
    CMD_RESET_TRIM,       /* Reset level trim to zero */
    CMD_CALIBRATE_MOTOR,  /* Single ESC calibration */
    CMD_MOVE_TO,          /* Move to relative position offset (P4) */
    CMD_MOVE_STOP,        /* Stop all horizontal movement */
    CMD_TAKEOFF,          /* Auto takeoff to specified height */
} commander_cmd_t;

typedef struct {
    float throttle;      /* 0.0 – 1.0 */
    float roll;          /* -1.0 – 1.0 */
    float pitch;         /* -1.0 – 1.0 */
    float yaw;           /* -1.0 – 1.0 */
    float vel_x;         /* 体轴前向速度指令 -1.0..1.0 */
    float vel_y;         /* 体轴右向速度指令 -1.0..1.0 */
    flight_mode_t mode;
    float motor[4];      /* manual per-motor throttle 0.0–1.0 */
    bool  motor_active;  /* true when motor[] should override flight ctrl */
    float mtrim[4];      /* per-motor trim offset -0.1–0.1 */
    int   calib_motor;   /* target motor index for single-ESC calibration */
    float move_to_x;     /* P4 move_to target X offset (flow unit, forward+) */
    float move_to_y;     /* P4 move_to target Y offset (flow unit, right+) */
    float takeoff_height;   /* takeoff target altitude in meters */
    float takeoff_throttle; /* takeoff base throttle 0.0–1.0 */
    float flow_kx, flow_ky; /* optical-flow gyro-compensation gains (runtime-tuned) */
    commander_cmd_t pending_cmd;  /* deferred cmd for main loop execution */
} setpoint_t;

/**
 * @brief 解析 WebSocket 发来的 JSON 遥控命令，更新内部 setpoint
 *        先解析到临时变量，再一次写入 g_sp，缩小竞态窗口
 * @param json JSON 字符串
 * @param len  字符串长度
 */
void commander_parse(const char *json, int len);

/**
 * @brief 获取当前 setpoint
 * @return 当前 setpoint 指针（只读）
 */
const setpoint_t *commander_get_setpoint(void);

/**
 * @brief 获取当前飞行模式名
 */
const char *commander_mode_name(flight_mode_t mode);

/**
 * @brief 强制复位 setpoint 到安全状态（DISARMED, 油门=0）
 *        WebSocket 全部断开或命令超时时调用
 */
void commander_reset_setpoint(void);

/**
 * @brief 检查距上次收到有效命令是否超时
 * @return true 已超时（应强制 DISARMED）
 */
bool commander_is_command_timeout(void);

/**
 * @brief Callback for handling special commands (calibrate, etc.)
 * @param cmd The command received
 */
typedef void (*commander_cmd_cb_t)(commander_cmd_t cmd);

/**
 * @brief Register a command callback
 */
void commander_set_cmd_callback(commander_cmd_cb_t cb);

/**
 * @brief Clear the pending command after main loop has executed it
 */
void commander_clear_pending_cmd(void);

#ifdef __cplusplus
}
#endif
