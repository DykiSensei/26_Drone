#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "tof400f.h"
#include "pv3901l1.h"
#include "wifi_ap.h"
#include "http_server.h"
#include "commander.h"
#include "motor.h"
#include "attitude.h"
#include "pid.h"
#include "mixer.h"
#include "altitude.h"
#include "flow_hold.h"
#include "position.h"
#include "params.h"

static const char *TAG = "main";

/* 光流模块开关 — 模块损坏时置 0 禁用 */
#define FLOW_ENABLED 1

/* 最大目标角速度 (rad/s) — 对应 setpoint 的 ±1.0 */
#define MAX_RATE_RAD_S  3.0f

/* 自稳外环参数 */
#define MAX_ANGLE_DEG   30.0f                   /* 最大倾斜角 ±30° */
#define ANGLE_KP        (6.0f * 0.017453293f)   /* ≈0.105 rad/s 每度误差 */

/* --- 分级 Failsafe 阈值（链路丢失后的分阶段降落）---
 * 之前的实现是 500ms 直接 DISARMED → 自由落体。
 * 新实现：先保持姿态（飞机不动），再缓慢降油门，最后才熄火 */
#define FS_HOLD_THRESHOLD_US     500000     /* 0.5s: 进入 HOLD（roll/pitch/yaw 归零，油门保持） */
#define FS_DESCEND_THRESHOLD_US  1000000    /* 1.0s: 进入 DESCEND（油门按斜坡降） */
#define FS_LAND_THRESHOLD_US     5000000    /* 5.0s: 强制 LAND（DISARMED 熄火） */
#define FS_DESCEND_RATE          0.05f      /* 油门下降速度 0.05/s — 约 8 秒从 0.4 降到 0 */

typedef enum {
    FS_NORMAL = 0,
    FS_HOLD,
    FS_DESCEND,
    FS_LAND,
} failsafe_state_t;

/* --- 主循环周期 / 看门狗 --- */
#define LOOP_PERIOD_MS    10        /* 100Hz 主循环 */
#define TWDT_TIMEOUT_MS   1000      /* 看门狗超时：1s 卡死即重启 */
#define DT_MIN            0.005f    /* 实测 dt 下限（防 PID 微分项暴冲） */
#define DT_MAX            0.050f    /* 实测 dt 上限（防积分项暴冲） */

/* --- ESC 同步起飞 ---
 * 4 个电调收到非零 PWM 后 sync 时间差 30-80ms（KV 误差 + stator 对位），
 * takeoff 瞬间推力不平衡 → 飞机起步就倾几度。
 * 先让所有电机停在 idle 200ms，等所有 ESC 完全 sync 后再交给控制环 */
#define MOTOR_IDLE_THROTTLE   0.05f      /* spool-up 阶段统一 idle 输出 */
#define SPOOLUP_DURATION_US   200000     /* 200ms 同步窗口 */

/* --- Accel 二次低通 ---
 * MPU6050 硬件 DLPF=4 已经 21Hz 截止，再叠加一道软件 EMA 进一步抗振动。
 * 主要给 Mahony 用：起飞期电机振动混进加速度计 → 重力方向估错 → 姿态偏。
 * α=0.3 截止≈5.7Hz @100Hz；遥测仍显示原始值便于调试 */
#define ACCEL_FILT_ALPHA  0.3f

/* --- 悬停油门学习窗口 ---
 * 严格稳态才学习：避免摇杆扰动/起飞过冲污染学习值 */
#define HOVER_LEARN_HEIGHT_MIN_MM   300     /* 离地 >= 30cm 才学（地效区外） */
#define HOVER_LEARN_HEIGHT_ERR_M    0.05f   /* |target - current| < 5cm */
#define HOVER_LEARN_VZ_MAX          0.10f   /* |vz| < 0.1 m/s */
#define HOVER_LEARN_STEADY_US       1000000 /* 持续稳态 1s 才开始更新 */

static pid_t pid_roll, pid_pitch, pid_yaw;
static float g_motor_out[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float g_trim_roll = 0.0f;   /* 水平修正：补偿 MPU6050 安装偏移 */
static float g_trim_pitch = 0.0f;
static bool  g_capture_trim = false;
static bool  g_move_to_pending = false;   /* CMD_MOVE_TO 延迟到主循环处理 */
static bool  g_move_stop_pending = false; /* CMD_MOVE_STOP 延迟到主循环处理 */
static bool  g_takeoff_pending = false;   /* CMD_TAKEOFF 延迟到主循环处理 */

static altitude_ctrl_t g_alt;            /* 定高 PID 控制器 */
static flow_hold_t     g_flow_hold;       /* 光流速度保持控制器 */
static position_ctrl_t g_position;        /* 光流位置控制器 (P4 move_to) */
static float           g_alt_out = 0.0f;  /* 定高 PID 输出（用于遥测） */
static flight_mode_t   g_prev_mode = MODE_DISARMED;
static int64_t         g_spoolup_end_us = 0;  /* ESC 同步窗口结束时间戳 */

static void on_all_clients_disconnected(void)
{
    commander_reset_setpoint();
    ESP_LOGW(TAG, "All WS clients disconnected — forcing DISARMED");
}

static void execute_pending_cmd(const setpoint_t *sp)
{
    switch (sp->pending_cmd) {
    case CMD_CALIBRATE:
        ESP_LOGW(TAG, "ESC calibration triggered from web");
        motor_calibrate();
        break;
    case CMD_GYRO_CALIB:
        ESP_LOGW(TAG, "Gyro recalibration started (keep drone still)");
        mpu6050_recalibrate_gyro();
        /* 必须复位 Mahony：gyro 零偏已更新，旧的积分项（为旧零偏累积）
           会立刻导致姿态漂移。复位后 Mahony 从水平姿态重新收敛，
           约 2 秒内稳定，之后再做水平校准才能捕获正确值。 */
        attitude_init();
        ESP_LOGW(TAG, "Mahony reset — wait 2s before level trim!");
        break;
    case CMD_LEVEL_TRIM:
        ESP_LOGW(TAG, "=== LEVEL TRIM: will capture on next loop ===");
        g_capture_trim = true;
        break;
    case CMD_RESET_TRIM:
        g_trim_roll = 0.0f;
        g_trim_pitch = 0.0f;
        ESP_LOGW(TAG, "=== LEVEL TRIM RESET to zero ===");
        break;
    case CMD_CALIBRATE_MOTOR: {
        int idx = sp->calib_motor;
        ESP_LOGW(TAG, "Single ESC calibration: motor %d", idx);
        motor_calibrate_single(idx);
        break;
    }
    case CMD_MOVE_TO:
        g_move_to_pending = true;
        ESP_LOGW(TAG, "move_to: x=%.1f y=%.1f (flow units)", sp->move_to_x, sp->move_to_y);
        break;
    case CMD_MOVE_STOP:
        g_move_stop_pending = true;
        ESP_LOGW(TAG, "move_stop");
        break;
    case CMD_TAKEOFF:
        g_takeoff_pending = true;
        ESP_LOGW(TAG, "takeoff: height=%.2f m, throttle=%.2f",
                 sp->takeoff_height, sp->takeoff_throttle);
        break;
    default:
        break;
    }
}

static const char *failsafe_name(int fs)
{
    switch (fs) {
    case FS_NORMAL:  return "normal";
    case FS_HOLD:    return "hold";
    case FS_DESCEND: return "descend";
    case FS_LAND:    return "land";
    default:         return "?";
    }
}

static void build_telemetry(char *buf, size_t sz,
                            const mpu6050_data_t *imu,
                            float roll, float pitch, float yaw,
                            uint16_t tof_mm,
                            const pv3901l1_data_t *flow,
                            float pid_r, float pid_p, float pid_y,
                            const setpoint_t *sp,
                            int fs_state)
{
    snprintf(buf, sz,
        "{"
        "\"accel\":[%.3f,%.3f,%.3f],"
        "\"gyro\":[%.5f,%.5f,%.5f],"
        "\"attitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"tof\":%u,"
        "\"alt\":{\"target\":%.2f,\"out\":%.3f,\"vz\":%.2f},"
        "\"flow\":{\"x\":%.1f,\"y\":%.1f,\"qual\":%u,\"cr\":%.2f,\"cp\":%.2f,\"cx\":%.1f,\"cy\":%.1f,\"vx\":%.1f,\"vy\":%.1f,\"s\":%.2f},"
        "\"motor\":[%.2f,%.2f,%.2f,%.2f],"
        "\"mtrim\":[%.2f,%.2f,%.2f,%.2f],"
        "\"pid\":[%.3f,%.3f,%.3f],"
        "\"trim\":{\"roll\":%.2f,\"pitch\":%.2f},"
        "\"mode\":\"%s\","
        "\"failsafe\":\"%s\""
        "}",
        imu->accel_x, imu->accel_y, imu->accel_z,
        imu->gyro_x, imu->gyro_y, imu->gyro_z,
        roll, pitch, yaw,
        tof_mm,
        g_alt.target_m, g_alt_out, g_alt.vz,
        flow->flow_x_i, flow->flow_y_i, flow->qual,
        g_flow_hold.out_roll_deg, g_flow_hold.out_pitch_deg,
        g_flow_hold.flow_x_comp, g_flow_hold.flow_y_comp,
        g_flow_hold.vx_est, g_flow_hold.vy_est,
        g_flow_hold.imu_scale,
        g_motor_out[0], g_motor_out[1], g_motor_out[2], g_motor_out[3],
        sp->mtrim[0], sp->mtrim[1], sp->mtrim[2], sp->mtrim[3],
        pid_r, pid_p, pid_y,
        g_trim_roll, g_trim_pitch,
        commander_mode_name(sp->mode),
        failsafe_name(fs_state)
    );
}

void app_main(void)
{
    printf("=== ESP32-S3 Drone ===\n");

    /* --- I2C bus --- */
    if (i2c_bus_init() != 0) {
        printf("FATAL: I2C bus init failed\n"); return;
    }

    /* --- Sensors --- */
    if (mpu6050_init() != 0) {
        printf("FATAL: mpu6050 init failed\n"); return;
    }
    if (tof400f_init() != 0) {
        printf("FATAL: tof init failed\n"); return;
    }
#if FLOW_ENABLED
    if (pv3901l1_init() != 0) {
        printf("FATAL: flow init failed\n"); return;
    }
#else
    ESP_LOGW(TAG, "FLOW DISABLED — PV3901L1 skipped");
#endif

    /* --- Motors --- */
    if (motor_init() != 0) {
        printf("FATAL: motor init failed\n"); return;
    }

    /* --- Attitude estimator --- */
    attitude_init();

    /* --- PID controllers (rate mode, balanced for initial flight) --- */
    pid_init(&pid_roll,  0.25f, 0.02f, 0.01f, 0.8f, 0.15f);
    pid_init(&pid_pitch, 0.25f, 0.02f, 0.01f, 0.8f, 0.15f);
    pid_init(&pid_yaw,   0.8f, 0.05f, 0.00f, 0.5f, 0.15f);
    altitude_init(&g_alt);
    flow_hold_init(&g_flow_hold);
    position_init(&g_position);

    /* --- WiFi AP --- */
    if (wifi_ap_init() != 0) {
        printf("FATAL: WiFi init failed\n"); return;
    }

    /* --- Params (NVS-backed)：必须在 wifi_ap_init 之后调用，
     *     因为 wifi_ap_init 里完成了 nvs_flash_init。
     *     加载完后立刻把 NVS 值灌进 commander（flow_imu_scale），
     *     这样 web 端连上来时显示的就是上次标定值 --- */
    params_init();
    commander_set_flow_imu_scale(params_get_flow_imu_scale());

    /* --- HTTP + WebSocket --- */
    http_server_set_command_cb(commander_parse);
    http_server_set_disconnect_cb(on_all_clients_disconnected);
    if (http_server_init() != 0) {
        printf("FATAL: HTTP server init failed\n"); return;
    }

    printf("=== Ready: connect to Drone WiFi, open http://192.168.4.1 ===\n");

    /* --- Task watchdog: 注册主循环，超时 1s 触发 panic 重启 ---
     * I2C 偶发卡总线时无看门狗会保持上一帧 PWM 直到电池没电 → 危险 */
    esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms = TWDT_TIMEOUT_MS,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_err_t wdt_err = esp_task_wdt_init(&twdt_cfg);
    if (wdt_err == ESP_ERR_INVALID_STATE) {
        /* ESP-IDF 默认已 init 过看门狗，改它的配置 */
        esp_task_wdt_reconfigure(&twdt_cfg);
    }
    esp_task_wdt_add(NULL);
    ESP_LOGW(TAG, "TWDT armed: %dms timeout, panic on miss", TWDT_TIMEOUT_MS);

    /* --- Main loop: 100Hz --- */
    mpu6050_data_t imu;
    uint16_t        tof_mm = 0;
    pv3901l1_data_t flow;
    float roll = 0, pitch = 0, yaw = 0;
    float pid_r = 0, pid_p = 0, pid_y = 0;
    char  json[768];

    /* 实测 dt + 锁定主循环节拍 */
    TickType_t next_wake = xTaskGetTickCount();
    int64_t    prev_us   = esp_timer_get_time();

    /* 分级 failsafe 状态 */
    failsafe_state_t fs_state = FS_NORMAL;
    float fs_descend_throttle = 0.0f;

    /* accel 二次 LPF 状态（首帧用真实读数初始化，避免从 0 缓慢爬升 ~1s） */
    float accel_f_x = 0.0f, accel_f_y = 0.0f, accel_f_z = 0.0f;
    bool  accel_f_inited = false;

    /* 悬停油门学习的稳态起始时间戳 */
    int64_t hover_stable_start_us = 0;

    while (1) {
        /* --- 实测 dt：vTaskDelay 不是固定周期，WiFi/HTTP 会抢占，
         *     原 dt=0.01 硬编码会让 PID 积分在抖动时算错 ±50% --- */
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - prev_us) * 1e-6f;
        prev_us = now_us;
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;

        mpu6050_read(&imu);
        tof400f_get_distance(&tof_mm);
#if FLOW_ENABLED
        int flow_new = pv3901l1_get_data(&flow);
#else
        int flow_new = -1;
        memset(&flow, 0, sizeof(flow));
#endif

        /* Accel 二次 EMA 低通（硬件 DLPF=21Hz 之上再加 ~5.7Hz 截止），给 Mahony / alt 用 */
        if (!accel_f_inited) {
            accel_f_x = imu.accel_x;
            accel_f_y = imu.accel_y;
            accel_f_z = imu.accel_z;
            accel_f_inited = true;
        } else {
            accel_f_x += ACCEL_FILT_ALPHA * (imu.accel_x - accel_f_x);
            accel_f_y += ACCEL_FILT_ALPHA * (imu.accel_y - accel_f_y);
            accel_f_z += ACCEL_FILT_ALPHA * (imu.accel_z - accel_f_z);
        }

        /* Mahony AHRS fusion（accel 用滤波值，gyro 直接用——DLPF 已经 20Hz） */
        attitude_update(accel_f_x, accel_f_y, accel_f_z,
                        imu.gyro_x,  imu.gyro_y,  imu.gyro_z,
                        dt);
        attitude_get_euler(&roll, &pitch, &yaw);

        /* 拿 setpoint 副本：分级 failsafe 会覆盖关键字段，不能污染 g_sp */
        setpoint_t local_sp = *commander_get_setpoint();

        /* --- 分级 Failsafe ---
         * 旧逻辑 500ms 立即 DISARMED 等于"飞行中突然熄火"=自由落体。
         * 新逻辑：
         *   0.5s: HOLD —— roll/pitch/yaw 归零（保持当前油门和模式悬停）
         *   1.0s: DESCEND —— 切 STABILIZE + 油门按 0.05/s 斜坡降
         *   5.0s: LAND —— 真正 DISARMED 熄火
         * 链路恢复（commander_parse 更新时间戳）任何阶段都立即退出 */
        int64_t no_cmd_us = commander_us_since_last_command();
        if (no_cmd_us > FS_LAND_THRESHOLD_US) {
            if (fs_state != FS_LAND) {
                ESP_LOGE(TAG, "Failsafe LAND (no cmd %lldms)", no_cmd_us / 1000);
                fs_state = FS_LAND;
                /* 同时把 g_sp 也清成 DISARMED：链路恢复后必须重新走 arming 流程
                 * （否则旧 mode 残留 + 电机突然全转） */
                commander_reset_setpoint();
            }
            local_sp.mode = MODE_DISARMED;
            local_sp.throttle = 0.0f;
        } else if (no_cmd_us > FS_DESCEND_THRESHOLD_US) {
            if (fs_state != FS_DESCEND) {
                ESP_LOGW(TAG, "Failsafe DESCEND (from thr %.2f)", local_sp.throttle);
                fs_descend_throttle = local_sp.throttle;
                fs_state = FS_DESCEND;
            }
            fs_descend_throttle -= FS_DESCEND_RATE * dt;
            if (fs_descend_throttle < 0.0f) fs_descend_throttle = 0.0f;
            local_sp.mode = MODE_STABILIZE;   /* 退出定高/定点，让 throttle 直接生效 */
            local_sp.throttle = fs_descend_throttle;
            local_sp.roll = local_sp.pitch = local_sp.yaw = 0.0f;
            local_sp.vel_x = local_sp.vel_y = 0.0f;
        } else if (no_cmd_us > FS_HOLD_THRESHOLD_US) {
            if (fs_state != FS_HOLD) {
                ESP_LOGW(TAG, "Failsafe HOLD attitude");
                fs_state = FS_HOLD;
            }
            local_sp.roll = local_sp.pitch = local_sp.yaw = 0.0f;
            local_sp.vel_x = local_sp.vel_y = 0.0f;
        } else {
            if (fs_state != FS_NORMAL) {
                ESP_LOGW(TAG, "Failsafe cleared");
                fs_state = FS_NORMAL;
            }
        }

        /* 处理延迟的 move_to / move_stop 命令（需要 flow 数据） */
        const setpoint_t *sp = &local_sp;
#if FLOW_ENABLED
        bool alt_mode_h = (sp->mode == MODE_ALT_HOLD || sp->mode == MODE_POS_HOLD);
        bool web_vel    = (fabsf(sp->vel_x) > 0.01f || fabsf(sp->vel_y) > 0.01f);

        if (g_move_to_pending) {
            position_set_target(&g_position,
                                sp->move_to_x, sp->move_to_y,
                                flow.flow_x_i, flow.flow_y_i);
            g_move_to_pending = false;
        }
        if (g_move_stop_pending) {
            position_reset(&g_position);   /* 下方默认分支会在当前点重新 hold */
            g_move_stop_pending = false;
        }

        /* 水平控制优先级（仅 ALT_HOLD / POS_HOLD 生效）：
         *   move_to 一次性移动 > Web 速度按钮 > 位置保持(默认，对抗漂移)
         * API: vel>0=前/右。flow_hold 用 +flow 测量(setpoint=0=静止保持)，
         * setpoint>0 直接表示前/右意图，无需取反 */
        float vel_cmd_x = 0.0f, vel_cmd_y = 0.0f;

        if (!alt_mode_h) {
            /* STABILIZE 等：交还飞手，不做位置保持 */
            if (g_position.active) position_reset(&g_position);
            if (web_vel) {
                vel_cmd_x = sp->vel_x * 80.0f;
                vel_cmd_y = sp->vel_y * 80.0f;
            }
        } else if (g_position.active && !g_position.hold) {
            /* move_to 进行中：到达后转为在该点持续位置保持 */
            position_update(&g_position, flow.flow_x_i, flow.flow_y_i, dt);
            if (position_reached(&g_position, flow.flow_x_i, flow.flow_y_i)) {
                ESP_LOGW(TAG, "move_to reached -> position hold");
                position_hold_start(&g_position, flow.flow_x_i, flow.flow_y_i);
            } else {
                vel_cmd_x = g_position.out_vx;
                vel_cmd_y = g_position.out_vy;
            }
        } else if (web_vel) {
            /* 手动速度优先：暂停位置保持，松手后默认分支在当前点重捕获 */
            if (g_position.active) position_reset(&g_position);
            vel_cmd_x = -(sp->vel_x * 80.0f);
            vel_cmd_y = -(sp->vel_y * 80.0f);
        } else {
            /* 默认：位置保持，锁定当前点对抗漂移。
             * 阈值同步降到 30 配合 flow_hold 软启动——早一点锁住"当前漂移起点"，
             * 比等到 qual=50 时锁到"已经漂走"的点更稳 */
            if (!g_position.active && flow.qual > 30) {
                position_hold_start(&g_position, flow.flow_x_i, flow.flow_y_i);
                ESP_LOGW(TAG, "position hold @ (%.0f, %.0f) qual=%u",
                         flow.flow_x_i, flow.flow_y_i, flow.qual);
            }
            if (g_position.active) {
                position_update(&g_position, flow.flow_x_i, flow.flow_y_i, dt);
                vel_cmd_x = g_position.out_vx;
                vel_cmd_y = g_position.out_vy;
            }
        }
        flow_hold_set_velocity(&g_flow_hold, vel_cmd_x, vel_cmd_y);

        /* 光流速度保持：陀螺补偿 + 互补滤波两条通道。
         * 同步 sp->flow_imu_scale 给 params 模块（web 端 flow_scale 命令改了它），
         * DISARMED 时 params_save 会把变更写入 NVS */
        flow_hold_set_gyro_comp(&g_flow_hold, sp->flow_kx, sp->flow_ky);
        flow_hold_set_imu_scale(&g_flow_hold, sp->flow_imu_scale);
        params_set_flow_imu_scale(sp->flow_imu_scale);

        /* 每帧 (100Hz) 推进 IMU 通道 + 跑 PID。
         * accel_f_x/y 是机体系；小角度悬停下 ≈ 世界系水平加速度。
         * 起飞稳态时倾角 < 10°，cos(10°)≈0.98，近似误差 < 3%，可接受。 */
        flow_hold_predict(&g_flow_hold, accel_f_x, accel_f_y, dt);

        /* 有新光流帧 (~50Hz) 时校正 vx_est */
        if (flow_new == 0) {
            flow_hold_update(&g_flow_hold, flow.flow_x, flow.flow_y,
                             imu.gyro_x, imu.gyro_y, flow.qual, tof_mm * 0.001f);
        }
#else
        g_move_to_pending = false;
        g_move_stop_pending = false;
#endif

        /* 捕获水平修正量（必须在水平放置时触发） */
        if (g_capture_trim) {
            g_trim_roll = roll;
            g_trim_pitch = pitch;
            g_capture_trim = false;
            ESP_LOGW(TAG, "level trim captured: roll=%.2f, pitch=%.2f", g_trim_roll, g_trim_pitch);
        }

        /* Motor control
         * 注：sp 指向 local_sp，分级 failsafe 在循环开头已覆盖关键字段 */

        /* Execute deferred commands (calibrate / trim) in main loop context
         * 校准类命令最长阻塞 12s（ESC 校准），会让 1s TWDT 误触发 panic →
         * 期间临时把当前任务从看门狗摘除，校准完再注册回来 */
        if (sp->pending_cmd != CMD_NONE) {
            bool is_blocking_cmd = (sp->pending_cmd == CMD_CALIBRATE ||
                                    sp->pending_cmd == CMD_CALIBRATE_MOTOR ||
                                    sp->pending_cmd == CMD_GYRO_CALIB);
            if (is_blocking_cmd) {
                esp_task_wdt_delete(NULL);
            }
            execute_pending_cmd(sp);
            commander_clear_pending_cmd();
            if (is_blocking_cmd) {
                esp_task_wdt_add(NULL);
                /* 校准期间主循环停了好几秒，把 prev_us 和 next_wake 重置，
                 * 避免下一轮算出超大 dt 让 PID 积分爆冲 */
                prev_us   = esp_timer_get_time();
                next_wake = xTaskGetTickCount();
            }
        }
        if (sp->motor_active) {
            /* 手动电机控制也必须检查 DISARMED 和油门安全 */
            if (sp->mode == MODE_DISARMED || sp->throttle < 0.05f) {
                motor_stop();
                memset(g_motor_out, 0, sizeof(g_motor_out));
            } else {
                motor_set(sp->motor);
                memcpy(g_motor_out, sp->motor, sizeof(g_motor_out));
            }
        } else if (sp->mode == MODE_DISARMED) {
            motor_stop();
            memset(g_motor_out, 0, sizeof(g_motor_out));
            pid_reset(&pid_roll);
            pid_reset(&pid_pitch);
            pid_reset(&pid_yaw);
            altitude_reset(&g_alt);
            flow_hold_reset(&g_flow_hold);
            position_reset(&g_position);
            g_spoolup_end_us = 0;
        } else {
            /* 安全：低油门时停转，防止地面角度环翘机 */
            if (sp->throttle < 0.05f) {
                motor_stop();
                memset(g_motor_out, 0, sizeof(g_motor_out));
                pid_reset(&pid_roll);
                pid_reset(&pid_pitch);
                pid_reset(&pid_yaw);
                altitude_reset(&g_alt);
                flow_hold_reset(&g_flow_hold);
                position_reset(&g_position);
                g_spoolup_end_us = 0;
            } else {
            bool alt_mode = (sp->mode == MODE_ALT_HOLD || sp->mode == MODE_POS_HOLD);

            /* 注：不再在 POS_HOLD 切入时清零光流积分。位置保持环用绝对积分值
             * 做相对锁定，清零会让 hold 锁点与测量瞬间错位、引发冲出。 */

            /* --- 定高：模式切入时捕获当前高度为目标 --- */
            bool entering_alt = alt_mode
                             && (g_prev_mode != MODE_ALT_HOLD && g_prev_mode != MODE_POS_HOLD);
            if (entering_alt && tof_mm >= 40 && tof_mm <= 4000) {
                altitude_capture_target(&g_alt, tof_mm * 0.001f);
                ESP_LOGW(TAG, "AltHold target captured: %.3f m (%u mm)",
                         g_alt.target_m, tof_mm);
            }

            /* --- 起飞：从当前地面高度斜坡爬升到目标，避免阶跃过冲 --- */
            if (g_takeoff_pending) {
                float h_now = (tof_mm >= 40 && tof_mm <= 4000) ? tof_mm * 0.001f : 0.0f;
                altitude_set_target(&g_alt, sp->takeoff_height, h_now);
                ESP_LOGW(TAG, "Takeoff: ramp %.2f m -> %.2f m", h_now, sp->takeoff_height);
                /* 悬停油门前馈：把学习/NVS 中的悬停油门直接灌进 g_sp.throttle，
                 * alt PID 只需要填差量，起飞瞬间就有正确推力，不必从 0 慢慢扛偏差。
                 * 之后用户碰摇杆滑块会立刻覆盖这个值。 */
                float hover = params_get_hover_throttle();
                commander_set_throttle(hover);
                ESP_LOGW(TAG, "Takeoff: hover throttle FF = %.3f", hover);
                /* ESC 同步窗口：接下来 200ms 强制 idle，让 4 个电调完全 sync */
                g_spoolup_end_us = esp_timer_get_time() + SPOOLUP_DURATION_US;
                ESP_LOGW(TAG, "Takeoff: spool-up 200ms");
#if FLOW_ENABLED
                /* 起飞即锁定当前点：避免等 qual 达标才锁、锁到已漂走的位置 */
                position_hold_start(&g_position, flow.flow_x_i, flow.flow_y_i);
                ESP_LOGW(TAG, "Takeoff: position lock @ (%.0f, %.0f)",
                         flow.flow_x_i, flow.flow_y_i);
#endif
                g_takeoff_pending = false;
            }

            /* --- 高度环 PID --- */
            g_alt_out = 0.0f;
            if (alt_mode && tof_mm >= 40 && tof_mm <= 4000) {
                /* 世界系垂直加速度（扣除重力，向上为正），用于 vz 互补滤波。
                 * 小角度悬停用 cos 倾斜补偿近似垂直分量即可。
                 * accel 用滤波值，避免振动尖刺破坏 vz 估计 */
                float cr = cosf(roll  * 0.0174532925f);
                float cp = cosf(pitch * 0.0174532925f);
                float az_up = accel_f_z * cr * cp - 9.81f;
                g_alt_out = altitude_update(&g_alt, tof_mm * 0.001f, az_up, dt);
            }

            /* --- 姿态角控制 --- */
            /* 所有辅助模式（STABILIZE / ALT_HOLD / POS_HOLD）都使用角度外环 */
            float target_roll_rate, target_pitch_rate;
            if (sp->mode != MODE_DISARMED) {
                /* 角度外环：摇杆映射到目标倾角，再换算为目标角速度 */
                float target_roll_angle  = sp->roll  * MAX_ANGLE_DEG;
                float target_pitch_angle = sp->pitch * MAX_ANGLE_DEG;

                /* 光流漂移修正：叠加速度环输出 */
                if (flow_hold_is_active(&g_flow_hold)) {
                    target_roll_angle  += g_flow_hold.out_roll_deg;
                    target_pitch_angle += g_flow_hold.out_pitch_deg;
                }

                /* 扣除水平修正量，补偿 MPU6050 安装偏移 */
                float roll_err  = roll  - g_trim_roll;
                float pitch_err = pitch - g_trim_pitch;

                target_roll_rate  = ANGLE_KP * (target_roll_angle - roll_err);
                target_pitch_rate = ANGLE_KP * (target_pitch_angle - pitch_err);
            } else {
                target_roll_rate  = sp->roll  * MAX_RATE_RAD_S;
                target_pitch_rate = sp->pitch * MAX_RATE_RAD_S;
            }

            /* Yaw 始终用 rate 模式 */
            float target_yaw_rate = sp->yaw * MAX_RATE_RAD_S;

            /* 角速率限幅 */
            if (target_roll_rate > MAX_RATE_RAD_S) target_roll_rate = MAX_RATE_RAD_S;
            if (target_roll_rate < -MAX_RATE_RAD_S) target_roll_rate = -MAX_RATE_RAD_S;
            if (target_pitch_rate > MAX_RATE_RAD_S) target_pitch_rate = MAX_RATE_RAD_S;
            if (target_pitch_rate < -MAX_RATE_RAD_S) target_pitch_rate = -MAX_RATE_RAD_S;

            float out_roll  = pid_update(&pid_roll,  target_roll_rate,  imu.gyro_x, dt);
            float out_pitch = pid_update(&pid_pitch, target_pitch_rate, imu.gyro_y, dt);
            float out_yaw   = pid_update(&pid_yaw,   target_yaw_rate,   imu.gyro_z, dt);

            pid_r = out_roll; pid_p = out_pitch; pid_y = out_yaw;

            /* 有效油门 = 摇杆基准 + 定高修正 */
            float effective_throttle = sp->throttle + g_alt_out;
            if (effective_throttle > 1.0f) effective_throttle = 1.0f;
            if (effective_throttle < 0.0f) effective_throttle = 0.0f;

            /* 倾角补偿（tilt compensation）：飞机倾斜 θ 时垂直推力 = throttle·cos(θ)，
             * 不补偿就会"银行 15° → 丢 3.4% 升力 → alt PID 加油门 → 摆正过冲"。
             * 用扣除 trim 后的真实倾角（trim 是 IMU 安装偏移）。
             * 钳位：cos < 0.667 对应 ~48°，超过即失控，倍率封顶 1.5x 防除零。 */
            float roll_act_rad  = (roll  - g_trim_roll)  * 0.0174532925f;
            float pitch_act_rad = (pitch - g_trim_pitch) * 0.0174532925f;
            float tilt_cos = cosf(roll_act_rad) * cosf(pitch_act_rad);
            if (tilt_cos < 0.667f) tilt_cos = 0.667f;
            effective_throttle /= tilt_cos;
            if (effective_throttle > 1.0f) effective_throttle = 1.0f;

            float m[MOTOR_COUNT];
            mixer_apply(effective_throttle, out_roll, out_pitch, out_yaw, m);

            /* 逐电机微调 + 钳位 */
            for (int i = 0; i < MOTOR_COUNT; i++) {
                m[i] += sp->mtrim[i];
                if (m[i] < 0.0f) m[i] = 0.0f;
                if (m[i] > 1.0f) m[i] = 1.0f;
            }

            /* ESC spool-up：takeoff 后 200ms 强制 idle，让 4 个电调完全 sync。
             * 控制环照常更新（防冷启动错位），只覆盖最终输出 */
            bool in_spoolup = (esp_timer_get_time() < g_spoolup_end_us);
            if (in_spoolup) {
                for (int i = 0; i < MOTOR_COUNT; i++) {
                    m[i] = MOTOR_IDLE_THROTTLE;
                }
            }

            motor_set(m);
            memcpy(g_motor_out, m, sizeof(g_motor_out));

            /* 悬停油门学习：仅在严格稳态时更新（防止扰动/过冲污染）。
             * 学习的是真实"维持高度所需的有效油门"，下次起飞作前馈，
             * 让 alt PID 从一开始就接近平衡点，I 项不需要累积补偿基线 */
            bool can_learn = !in_spoolup
                          && alt_mode
                          && tof_mm >= HOVER_LEARN_HEIGHT_MIN_MM && tof_mm <= 4000
                          && fabsf(g_alt.target_m - tof_mm * 0.001f) < HOVER_LEARN_HEIGHT_ERR_M
                          && fabsf(g_alt.vz) < HOVER_LEARN_VZ_MAX;
            if (can_learn) {
                if (hover_stable_start_us == 0) hover_stable_start_us = now_us;
                if (now_us - hover_stable_start_us > HOVER_LEARN_STEADY_US) {
                    params_update_hover_throttle(effective_throttle);
                }
            } else {
                hover_stable_start_us = 0;
            }
            }
        }

        /* DISARMED 切入瞬间持久化学习到的悬停油门（仅在与上次保存差距 > 1% 时实际写 flash） */
        if (sp->mode == MODE_DISARMED && g_prev_mode != MODE_DISARMED) {
            params_save();
        }

        g_prev_mode = sp->mode;

        build_telemetry(json, sizeof(json), &imu, roll, pitch, yaw,
                        tof_mm, &flow, pid_r, pid_p, pid_y, sp, fs_state);
        http_server_broadcast(json);

        /* 喂狗 + vTaskDelayUntil 锁定 10ms 周期（vTaskDelay 是相对延时，
         * 会被 WiFi/HTTP 的临时阻塞拉成 12-15ms） */
        esp_task_wdt_reset();
        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}
