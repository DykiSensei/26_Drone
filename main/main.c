#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
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

static const char *TAG = "main";

/* 光流模块开关 — 模块损坏时置 0 禁用 */
#define FLOW_ENABLED 0

/* 最大目标角速度 (rad/s) — 对应 setpoint 的 ±1.0 */
#define MAX_RATE_RAD_S  3.0f

/* 自稳外环参数 */
#define MAX_ANGLE_DEG   30.0f                   /* 最大倾斜角 ±30° */
#define ANGLE_KP        (6.0f * 0.017453293f)   /* ≈0.105 rad/s 每度误差 */

static pid_t pid_roll, pid_pitch, pid_yaw;
static float g_motor_out[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float g_trim_roll = 0.0f;   /* 水平修正：补偿 MPU6050 安装偏移 */
static float g_trim_pitch = 0.0f;
static bool  g_capture_trim = false;
static bool  g_move_to_pending = false;   /* CMD_MOVE_TO 延迟到主循环处理 */
static bool  g_move_stop_pending = false; /* CMD_MOVE_STOP 延迟到主循环处理 */

static altitude_ctrl_t g_alt;            /* 定高 PID 控制器 */
static flow_hold_t     g_flow_hold;       /* 光流速度保持控制器 */
static position_ctrl_t g_position;        /* 光流位置控制器 (P4 move_to) */
static float           g_alt_out = 0.0f;  /* 定高 PID 输出（用于遥测） */
static flight_mode_t   g_prev_mode = MODE_DISARMED;

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
    default:
        break;
    }
}

static void build_telemetry(char *buf, size_t sz,
                            const mpu6050_data_t *imu,
                            float roll, float pitch, float yaw,
                            uint16_t tof_mm,
                            const pv3901l1_data_t *flow,
                            float pid_r, float pid_p, float pid_y)
{
    snprintf(buf, sz,
        "{"
        "\"accel\":[%.3f,%.3f,%.3f],"
        "\"gyro\":[%.5f,%.5f,%.5f],"
        "\"attitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"tof\":%u,"
        "\"alt\":{\"target\":%.2f,\"out\":%.3f},"
        "\"flow\":{\"x\":%.1f,\"y\":%.1f,\"qual\":%u,\"cr\":%.2f,\"cp\":%.2f},"
        "\"motor\":[%.2f,%.2f,%.2f,%.2f],"
        "\"mtrim\":[%.2f,%.2f,%.2f,%.2f],"
        "\"pid\":[%.3f,%.3f,%.3f],"
        "\"trim\":{\"roll\":%.2f,\"pitch\":%.2f},"
        "\"mode\":\"%s\""
        "}",
        imu->accel_x, imu->accel_y, imu->accel_z,
        imu->gyro_x, imu->gyro_y, imu->gyro_z,
        roll, pitch, yaw,
        tof_mm,
        g_alt.target_m, g_alt_out,
        flow->flow_x_i, flow->flow_y_i, flow->qual,
        g_flow_hold.out_roll_deg, g_flow_hold.out_pitch_deg,
        g_motor_out[0], g_motor_out[1], g_motor_out[2], g_motor_out[3],
        commander_get_setpoint()->mtrim[0],
        commander_get_setpoint()->mtrim[1],
        commander_get_setpoint()->mtrim[2],
        commander_get_setpoint()->mtrim[3],
        pid_r, pid_p, pid_y,
        g_trim_roll, g_trim_pitch,
        commander_mode_name(commander_get_setpoint()->mode)
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

    /* --- HTTP + WebSocket --- */
    http_server_set_command_cb(commander_parse);
    http_server_set_disconnect_cb(on_all_clients_disconnected);
    if (http_server_init() != 0) {
        printf("FATAL: HTTP server init failed\n"); return;
    }

    printf("=== Ready: connect to Drone WiFi, open http://192.168.4.1 ===\n");

    /* --- Main loop: 100Hz --- */
    mpu6050_data_t imu;
    uint16_t        tof_mm = 0;
    pv3901l1_data_t flow;
    float roll = 0, pitch = 0, yaw = 0;
    float pid_r = 0, pid_p = 0, pid_y = 0;
    char  json[640];
    const float dt = 0.01f;

    while (1) {
        mpu6050_read(&imu);
        tof400f_get_distance(&tof_mm);
#if FLOW_ENABLED
        int flow_new = pv3901l1_get_data(&flow);
#else
        int flow_new = -1;
        memset(&flow, 0, sizeof(flow));
#endif

        /* Mahony AHRS fusion */
        attitude_update(imu.accel_x, imu.accel_y, imu.accel_z,
                        imu.gyro_x,  imu.gyro_y,  imu.gyro_z,
                        dt);
        attitude_get_euler(&roll, &pitch, &yaw);

        /* 处理延迟的 move_to / move_stop 命令（需要 flow 数据） */
        const setpoint_t *sp = commander_get_setpoint();
#if FLOW_ENABLED
        if (g_move_to_pending) {
            position_set_target(&g_position,
                                sp->move_to_x, sp->move_to_y,
                                flow.flow_x_i, flow.flow_y_i);
            g_move_to_pending = false;
        }
        if (g_move_stop_pending) {
            position_reset(&g_position);
            flow_hold_set_velocity(&g_flow_hold, 0.0f, 0.0f);
            g_move_stop_pending = false;
        }

        /* 水平速度指令优先级：位置控制器 > Web按钮 > 默认(0) */
        /* API 约定: vel_x>0=前向, vel_y>0=右向
         * flow_hold PID 内部用 -flow 作为测量值 (setpoint=0 即静止保持)
         * 所以前向移动需要负的 setpoint，此处取反 */
        float vel_cmd_x = 0.0f, vel_cmd_y = 0.0f;
        if (g_position.active) {
            position_update(&g_position, flow.flow_x_i, flow.flow_y_i, dt);
            if (position_reached(&g_position, flow.flow_x_i, flow.flow_y_i)) {
                ESP_LOGW(TAG, "position target reached");
                position_reset(&g_position);
            } else {
                vel_cmd_x = -g_position.out_vx;
                vel_cmd_y = -g_position.out_vy;
            }
        } else {
            if (fabsf(sp->vel_x) > 0.01f || fabsf(sp->vel_y) > 0.01f) {
                vel_cmd_x = -(sp->vel_x * 80.0f);  /* 归一化 → flow 速度单位 */
                vel_cmd_y = -(sp->vel_y * 80.0f);
            }
        }
        flow_hold_set_velocity(&g_flow_hold, vel_cmd_x, vel_cmd_y);

        /* 光流速度保持：有新数据时更新 */
        if (flow_new == 0) {
            flow_hold_update(&g_flow_hold, flow.flow_x, flow.flow_y,
                             flow.qual, tof_mm * 0.001f);
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

        /* Motor control */
        /* 注意：sp 已在上面定义（处理 move_to / move_stop 时） */

        /* 命令超时检测：超过 500ms 没收遥控指令 → 强制 DISARMED */
        if (commander_is_command_timeout()) {
            ESP_LOGE(TAG, "Command timeout! Forcing DISARMED");
            commander_reset_setpoint();
            sp = commander_get_setpoint();
        }

        /* Execute deferred commands (calibrate / trim) in main loop context */
        if (sp->pending_cmd != CMD_NONE) {
            execute_pending_cmd(sp);
            commander_clear_pending_cmd();
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
            } else {
            bool alt_mode = (sp->mode == MODE_ALT_HOLD || sp->mode == MODE_POS_HOLD);

            /* POS_HOLD 切入时重置光流积分 */
#if FLOW_ENABLED
            bool entering_poshold = (sp->mode == MODE_POS_HOLD)
                                 && (g_prev_mode != MODE_POS_HOLD);
            if (entering_poshold) {
                pv3901l1_reset_integral();
                ESP_LOGW(TAG, "POS_HOLD: flow integral reset");
            }
#endif

            /* --- 定高：模式切入时捕获当前高度为目标 --- */
            bool entering_alt = alt_mode
                             && (g_prev_mode != MODE_ALT_HOLD && g_prev_mode != MODE_POS_HOLD);
            if (entering_alt && tof_mm >= 40 && tof_mm <= 4000) {
                altitude_capture_target(&g_alt, tof_mm * 0.001f);
                ESP_LOGW(TAG, "AltHold target captured: %.3f m (%u mm)",
                         g_alt.target_m, tof_mm);
            }

            /* --- 高度环 PID --- */
            g_alt_out = 0.0f;
            if (alt_mode && tof_mm >= 40 && tof_mm <= 4000) {
                g_alt_out = altitude_update(&g_alt, tof_mm * 0.001f, dt);
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

            float m[MOTOR_COUNT];
            mixer_apply(effective_throttle, out_roll, out_pitch, out_yaw, m);

            /* 逐电机微调 + 钳位 */
            for (int i = 0; i < MOTOR_COUNT; i++) {
                m[i] += sp->mtrim[i];
                if (m[i] < 0.0f) m[i] = 0.0f;
                if (m[i] > 1.0f) m[i] = 1.0f;
            }

            motor_set(m);
            memcpy(g_motor_out, m, sizeof(g_motor_out));
            }
        }

        g_prev_mode = sp->mode;

        build_telemetry(json, sizeof(json), &imu, roll, pitch, yaw,
                        tof_mm, &flow, pid_r, pid_p, pid_y);
        http_server_broadcast(json);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
