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
    CMD_GRAB_START,       /* 启动抓取任务 (grab_test 区分测试/P4 正式流程) */
    CMD_GRAB_ABORT,       /* 中止抓取/投放任务 */
    CMD_MARK_DROP,        /* 标记当前位置为投放点 (悬停筐上方时按) */
    CMD_DROP_START,       /* 启动投放任务 (drop_goto 区分返航/就地) */
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
    float move_to_x;     /* P4 move_to target X offset (meters, forward+) */
    float move_to_y;     /* P4 move_to target Y offset (meters, right+) */
    float takeoff_height;   /* takeoff target altitude in meters */
    float takeoff_throttle; /* takeoff base throttle 0.0–1.0 */
    float flow_kx, flow_ky; /* optical-flow gyro-compensation gains (runtime-tuned) */
    float flow_scale;       /* optical-flow metric scale rad/count (runtime-tuned) */
    bool  flow_calib;       /* calibration mode: in DISARMED keep the flow estimator
                             * running (motors stay stopped) so flow_scale can be
                             * calibrated by hand-carrying the drone — no arming,
                             * props stay on. Cleared on safety reset. */
    float grip_angle;       /* 机械爪目标角 0–90°（任意模式生效；失联复位时保留，
                             * 防止断连瞬间松爪掉落已抓取的目标） */
    float grab_tof_m;       /* 抓取触发高度: TOF 读数低于此即闭爪（TOF 离地安装
                             * 高度 ≈0.20m，默认 0.20；前端 grab_cfg 滑条调试；
                             * 失联复位保留） */
    bool  grab_test;        /* CMD_GRAB_START 参数: true=无 P4 测试流程
                             * (跳过视觉对准, 假设当前位置精确) */
    float drop_tof_m;       /* 投放张爪高度: TOF 读数 (默认 0.30, 前端可调;
                             * 失联复位保留) */
    bool  drop_goto;        /* CMD_DROP_START 参数: true=先返航到标记投放点 */
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

/**
 * @brief 程序侧设置机械爪目标角（抓取任务闭爪用，等效前端 grip 命令）。
 *        sp->grip_angle 仍是爪角唯一真源。与 commander_parse 的整结构体
 *        赋值存在窄竞态（丢写概率极低）——调用方在关键阶段每拍重写即自愈。
 */
void commander_set_grip(float deg);

#ifdef __cplusplus
}
#endif
