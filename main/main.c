#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
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
#define FLOW_ENABLED 1

/* 最大目标角速度 (rad/s) — 对应 setpoint 的 ±1.0 */
#define MAX_RATE_RAD_S  3.0f

/* 自稳外环参数 */
#define MAX_ANGLE_DEG   30.0f                   /* 最大倾斜角 ±30° */
#define ANGLE_KP        (6.0f * 0.017453293f)   /* ≈0.105 rad/s 每度误差 */

/* Web 方向按钮满偏对应的水平速度指令 (m/s)。按钮发 ±0.5 → ±0.3 m/s */
#define MANUAL_VEL_MS   0.6f

static pid_t pid_roll, pid_pitch, pid_yaw;
static float g_motor_out[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float g_trim_roll = 0.0f;   /* 水平修正：补偿 MPU6050 安装偏移 */
static float g_trim_pitch = 0.0f;
static bool  g_capture_trim = false;
static bool  g_move_to_pending = false;   /* CMD_MOVE_TO 延迟到主循环处理 */
static bool  g_move_stop_pending = false; /* CMD_MOVE_STOP 延迟到主循环处理 */
static bool  g_takeoff_pending = false;   /* CMD_TAKEOFF 延迟到主循环处理 */
static bool  g_position_lock_pending = false;  /* takeoff 期间延迟启动位置环：等飞机
                                                * 稳定在目标高度后再锁定当前点，避免
                                                * 上升阶段机身倾斜让光流积分带噪声 →
                                                * position 误判为漂移 → 输出 vel_cmd 干扰起飞 */

static altitude_ctrl_t g_alt;            /* 定高 PID 控制器 */
static flow_hold_t     g_flow_hold;       /* 光流速度保持控制器 */
static position_ctrl_t g_position;        /* 光流位置控制器 (P4 move_to) */
static float           g_alt_out = 0.0f;  /* 定高 PID 输出（用于遥测） */
static float           g_ax_filt = 0.0f;  /* accel X EMA for flow predict */
static float           g_ay_filt = 0.0f;  /* accel Y EMA for flow predict */
#define ACCEL_EMA_ALPHA 0.3f              /* ~5.7Hz cutoff @ 100Hz */
static flight_mode_t   g_prev_mode = MODE_DISARMED;

static void on_all_clients_disconnected(void)
{
    commander_reset_setpoint();
    ESP_LOGW(TAG, "All WS clients disconnected — forcing DISARMED");
}

/* 只允许在地面（DISARMED）执行的命令：校准会阻塞主循环数秒甚至把电机打到
 * 满油门（ESC 校准），飞行中触发等于炸机；水平修正类飞行中触发会把当前
 * 飞行姿态当成"水平"，同样危险。 */
static bool cmd_requires_disarmed(commander_cmd_t cmd)
{
    return cmd == CMD_CALIBRATE || cmd == CMD_GYRO_CALIB
        || cmd == CMD_CALIBRATE_MOTOR
        || cmd == CMD_LEVEL_TRIM || cmd == CMD_RESET_TRIM;
}

static void execute_pending_cmd(const setpoint_t *sp)
{
    if (cmd_requires_disarmed(sp->pending_cmd) && sp->mode != MODE_DISARMED) {
        ESP_LOGE(TAG, "cmd %d rejected: calibration/trim only allowed in DISARMED",
                 sp->pending_cmd);
        return;
    }
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
        ESP_LOGW(TAG, "move_to: x=%.2f y=%.2f (m)", sp->move_to_x, sp->move_to_y);
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

static void build_telemetry(char *buf, size_t sz,
                            const mpu6050_data_t *imu,
                            float roll, float pitch, float yaw,
                            uint16_t tof_mm,
                            const pv3901l1_data_t *flow,
                            float pid_r, float pid_p, float pid_y)
{
    /* 位置环状态编码（前端无串口时观察）：
     *   0 = idle（无 hold、无 pending）
     *   1 = pending（takeoff 上升中等达标后启动）
     *   2 = hold（已锁定，正常工作）
     *   3 = move_to（一次性移动中） */
    int pos_state = 0;
    if (g_position.active) {
        pos_state = g_position.hold ? 2 : 3;
    } else if (g_position_lock_pending) {
        pos_state = 1;
    }

    snprintf(buf, sz,
        "{"
        "\"accel\":[%.3f,%.3f,%.3f],"
        "\"gyro\":[%.5f,%.5f,%.5f],"
        "\"attitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
        "\"tof\":%u,"
        "\"alt\":{\"target\":%.2f,\"out\":%.3f,\"vz\":%.2f},"
        "\"flow\":{\"x\":%.2f,\"y\":%.2f,\"qual\":%u,\"qg\":%.2f,\"cr\":%.2f,\"cp\":%.2f,\"cx\":%.2f,\"cy\":%.2f,\"ps\":%d,\"tx\":%.2f,\"ty\":%.2f,\"fc\":%lu,\"ec\":%lu,\"fx\":%d,\"fy\":%d,\"vx\":%.2f,\"vy\":%.2f},"
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
        g_alt.target_m, g_alt_out, g_alt.vz,
        g_flow_hold.pos_x_m, g_flow_hold.pos_y_m, flow->qual,
        g_flow_hold.quality_gain,
        g_flow_hold.out_roll_deg, g_flow_hold.out_pitch_deg,
        g_flow_hold.flow_x_comp, g_flow_hold.flow_y_comp,
        pos_state, g_position.target_x, g_position.target_y,
        (unsigned long)flow->frame_count, (unsigned long)flow->error_count,
        (int)flow->flow_x, (int)flow->flow_y,
        g_flow_hold.vx_est, g_flow_hold.vy_est,
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
    mpu6050_data_t imu = {0};   /* 读失败时保留上一帧有效值（冻结而非清零），
                                 * 避免 gyro=0 假反馈让角速率环输出扭矩突跳 */
    mpu6050_data_t imu_new;
    int             imu_fail = 0;
    int             telem_div = 0;
    uint16_t        tof_mm = 0;
    pv3901l1_data_t flow = {0};   /* 必须清零：首帧光流到达前 get_data 不写入，
                                   * 否则 qual/积分读到栈上垃圾值 */
    float roll = 0, pitch = 0, yaw = 0;
    float pid_r = 0, pid_p = 0, pid_y = 0;
    char  json[768];

    /* 固定节拍 + 实测 dt：vTaskDelayUntil 保证周期不被循环执行时间拉长，
     * 积分器用实测 dt 消除残余抖动（旧 vTaskDelay + 硬编码 0.01 会让实际
     * 周期 = 10ms + 执行时间，所有积分系统性偏大）。 */
    TickType_t last_wake = xTaskGetTickCount();
    int64_t    prev_us   = esp_timer_get_time();

    while (1) {
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - prev_us) * 1e-6f;
        prev_us = now_us;
        if (dt < 0.005f) dt = 0.005f;
        if (dt > 0.03f)  dt = 0.03f;

        if (mpu6050_read(&imu_new) == 0) {
            imu = imu_new;
            imu_fail = 0;
        } else if (imu_fail < 1000) {
            imu_fail++;
        }
        /* IMU 失效保护：连续 200ms 读不到数据 → 姿态反馈不可信，继续飞
         * 就是开环失控，强制 DISARMED（重新上锁尝试也会被立刻再锁）。 */
        if (imu_fail >= 20 && commander_get_setpoint()->mode != MODE_DISARMED) {
            ESP_LOGE(TAG, "IMU failure (%d consecutive reads) — forcing DISARMED", imu_fail);
            commander_reset_setpoint();
        }

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
        bool alt_mode_h = (sp->mode == MODE_ALT_HOLD || sp->mode == MODE_POS_HOLD);
        bool web_vel    = (fabsf(sp->vel_x) > 0.01f || fabsf(sp->vel_y) > 0.01f);

        if (g_move_to_pending) {
            position_set_target(&g_position,
                                sp->move_to_x, sp->move_to_y,
                                g_flow_hold.pos_x_m, g_flow_hold.pos_y_m);
            g_move_to_pending = false;
        }
        if (g_move_stop_pending) {
            position_reset(&g_position);   /* 下方默认分支会在当前点重新 hold */
            g_move_stop_pending = false;
        }

        /* 水平控制优先级（仅 ALT_HOLD / POS_HOLD 生效）：
         *   move_to 一次性移动 > Web 速度按钮 > 位置保持(默认，对抗漂移)
         * 全链路米制：位置反馈用 flow_hold 的航位推算 pos_x_m/pos_y_m (m)，
         * 速度指令统一 m/s。符号约定（本机实测）：前推 fx>0、右推 fy>0，
         * 前/右 = 光流正方向。API 的 vel>0=前/右 直接作为正 setpoint 送入
         * flow_hold，无需取反（朝向修正角的反号在 flow_hold_predict 输出处
         * 统一处理）。 */
        float vel_cmd_x = 0.0f, vel_cmd_y = 0.0f;

        if (!alt_mode_h) {
            /* STABILIZE 等：交还飞手，不做位置保持 */
            if (g_position.active) position_reset(&g_position);
            if (web_vel) {
                vel_cmd_x = sp->vel_x * MANUAL_VEL_MS;
                vel_cmd_y = sp->vel_y * MANUAL_VEL_MS;
            }
        } else if (g_position.active && !g_position.hold) {
            /* move_to 进行中：到达后转为在该点持续位置保持 */
            position_update(&g_position, g_flow_hold.pos_x_m, g_flow_hold.pos_y_m, dt);
            if (position_reached(&g_position)) {
                ESP_LOGW(TAG, "move_to reached -> position hold");
                position_hold_start(&g_position, g_flow_hold.pos_x_m, g_flow_hold.pos_y_m);
            } else {
                vel_cmd_x = g_position.out_vx;
                vel_cmd_y = g_position.out_vy;
            }
        } else if (web_vel) {
            /* 手动速度优先：暂停位置保持，松手后默认分支在当前点重捕获 */
            if (g_position.active) position_reset(&g_position);
            vel_cmd_x = sp->vel_x * MANUAL_VEL_MS;
            vel_cmd_y = sp->vel_y * MANUAL_VEL_MS;
        } else {
            /* 默认：位置保持，锁定当前点对抗漂移。
             *  - flow.qual 阈值降到 30 配合 flow_hold 软启动
             *  - takeoff 上升期 (g_position_lock_pending=true) 跳过自动锁定，
             *    此期间速度环 (setpoint=0) 仍全程压制水平漂移——米制化后
             *    速度环增益在低空/爬升段不再失配，这才是垂直起飞的关键 */
            if (!g_position.active && !g_position_lock_pending && flow.qual > 30) {
                position_hold_start(&g_position, g_flow_hold.pos_x_m, g_flow_hold.pos_y_m);
                ESP_LOGW(TAG, "position hold @ (%.2f, %.2f) m",
                         g_flow_hold.pos_x_m, g_flow_hold.pos_y_m);
            }
            if (g_position.active) {
                position_update(&g_position, g_flow_hold.pos_x_m, g_flow_hold.pos_y_m, dt);
                vel_cmd_x = g_position.out_vx;
                vel_cmd_y = g_position.out_vy;
            }
        }
        flow_hold_set_velocity(&g_flow_hold, vel_cmd_x, vel_cmd_y);
        flow_hold_set_gyro_comp(&g_flow_hold, sp->flow_kx, sp->flow_ky);
        flow_hold_set_flow_scale(&g_flow_hold, sp->flow_scale);

        /* IMU + 光流 互补滤波：
         *   - predict 每帧（100Hz）跑：IMU 加速度积分 → vx_est，并跑 PID 算修正角
         *   - update 仅在新光流帧（~50Hz）跑：陀螺补偿后用 flow 校正 vx_est
         * 解决 PV3901L1 模块"连续两帧位移太小输出 0"导致的微小漂移检测不到问题：
         * 即使 flow 给 0，IMU 积分通道仍能捕捉短时位移，光流只做长期校正防漂。
         * accel_x/y 直接当世界系水平加速度用（小角度悬停 cos<10° → 误差 <3%）。 */
        g_ax_filt += ACCEL_EMA_ALPHA * (imu.accel_x - g_ax_filt);
        g_ay_filt += ACCEL_EMA_ALPHA * (imu.accel_y - g_ay_filt);
        /* accel 轴向映射（2026-07-04 快推实测验证，不再是推导值）：
         *   快推前 → accel_x 读负 ⇒ IMU X 朝后 ⇒ 取反后 = 机体前向加速度；
         *   快推左 → accel_y 读负 ⇒ IMU Y 朝右 ⇒ 直通   = 机体右向加速度。
         * 与光流机体系（前/右=正）一致。接错则 predict 与光流校正互相
         * 拉扯，速度估计振荡。 */
        flow_hold_predict(&g_flow_hold, -g_ax_filt, g_ay_filt, dt);

        if (flow_new == 0) {
            /* 传累计 counts（acc），不是最新帧（flow_x）——批量到达不丢帧 */
            flow_hold_update(&g_flow_hold, (float)flow.acc_x, (float)flow.acc_y,
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
        if (sp->mode == MODE_DISARMED && sp->motor_active) {
            /* 台架电机测试：只在 DISARMED（锁定）下生效，飞行模式下 motor
             * 数组被完全忽略。旧逻辑相反（armed 才生效），导致前端 All MAX
             * 按钮在飞行中等于四电机满油门指令。 */
            motor_set(sp->motor);
            memcpy(g_motor_out, sp->motor, sizeof(g_motor_out));
        } else if (sp->mode == MODE_DISARMED) {
            motor_stop();
            memset(g_motor_out, 0, sizeof(g_motor_out));
            pid_reset(&pid_roll);
            pid_reset(&pid_pitch);
            pid_reset(&pid_yaw);
            altitude_reset(&g_alt);
            position_reset(&g_position);
            g_position_lock_pending = false;
            /* flow_calib 标定模式：本分支电机始终停转（上面 motor_stop），
             * 但跳过速度/位置估计器复位 —— 手持移动飞机、看遥测 X(m) 即可
             * 安全标定 flow_scale，无需拆桨解锁。断连/超时复位 setpoint 时
             * flow_calib 自动清零，恢复正常复位行为；解锁起飞时低油门分支
             * 无条件复位，手持残留不会带入飞行。
             * 注意：g_ax_filt/g_ay_filt 绝不能在这里清零 —— 它们是输入调理
             * 滤波器，必须连续运行。每拍清零会让估计器的加速度直流跟踪器
             * (ax_lp) 锚定在 0.3 折的错误基线上，模式切换瞬间输入阶跃 →
             * 跟踪器重收敛的 ~2s 内积出 ~0.5 m/s 假速度、漂移几十 cm
             * （2026-07-04 标定模式开启瞬间实测复现，起飞推油门同理）。 */
            if (!sp->flow_calib) {
                flow_hold_reset(&g_flow_hold);
            }
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
                g_position_lock_pending = false;
                /* g_ax_filt/g_ay_filt 不清零：输入滤波必须连续运行，否则
                 * 推油门瞬间加速度输入阶跃 → 起飞头 2s 出现假速度瞬态
                 * （详见 DISARMED 分支注释） */
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
#if FLOW_ENABLED
                /* 上升阶段不锁位置：机身倾斜让光流积分带噪声 → 假漂移 → 误干预。
                 * 标记 g_position_lock_pending，等到达目标高度且稳定后再锁。
                 * 若 default 分支之前已经启动了 position（地面静止时 qual 达标），
                 * 这里先 reset 掉，避免上升过程中持续输出 vel_cmd。 */
                if (g_position.active) position_reset(&g_position);
                g_position_lock_pending = true;
#endif
                g_takeoff_pending = false;
            }

            /* --- 高度环 PID --- */
            g_alt_out = 0.0f;
            if (alt_mode && tof_mm >= 40 && tof_mm <= 4000) {
                /* 世界系垂直加速度（扣除重力，向上为正），用于 vz 互补滤波。
                 * 小角度悬停用 cos 倾斜补偿近似垂直分量即可。 */
                float cr = cosf(roll  * 0.0174532925f);
                float cp = cosf(pitch * 0.0174532925f);
                float az_up = imu.accel_z * cr * cp - 9.81f;
                g_alt_out = altitude_update(&g_alt, tof_mm * 0.001f, az_up, dt);

#if FLOW_ENABLED
                /* takeoff 达到目标高度且稳定后启动位置环。
                 * 触发条件：高度误差 < 10cm（相对 final，ramp 完成）+ |vz| < 0.15m/s（不上不下）
                 *           + flow.qual > 30（光流可信）。任一不满足保持 pending，下次主循环再判。 */
                if (g_position_lock_pending) {
                    float current_m = tof_mm * 0.001f;
                    bool reached = fabsf(current_m - g_alt.target_final_m) < 0.10f;
                    bool steady  = fabsf(g_alt.vz) < 0.15f;
                    if (reached && steady && flow.qual > 30) {
                        position_hold_start(&g_position, g_flow_hold.pos_x_m, g_flow_hold.pos_y_m);
                        ESP_LOGW(TAG, "Reached target -> position lock @ (%.2f, %.2f) m",
                                 g_flow_hold.pos_x_m, g_flow_hold.pos_y_m);
                        g_position_lock_pending = false;
                    }
                }
#endif
            }

            /* --- 姿态角控制 --- */
            /* 所有辅助模式（STABILIZE / ALT_HOLD / POS_HOLD）都使用角度外环 */
            float target_roll_rate, target_pitch_rate;
            if (sp->mode != MODE_DISARMED) {
                /* 角度外环：摇杆映射到目标倾角，再换算为目标角速度 */
                float target_roll_angle  = sp->roll  * MAX_ANGLE_DEG;
                float target_pitch_angle = sp->pitch * MAX_ANGLE_DEG;

                /* 光流漂移修正：仅在定高/定点模式或有手动速度指令时叠加。
                 * STABILIZE 纯手动：无 vel 指令时不能叠加，否则速度环
                 * (setpoint=0) 会以 ±8° 权限对抗飞手摇杆。 */
                bool flow_corr_en = (sp->mode == MODE_ALT_HOLD || sp->mode == MODE_POS_HOLD)
                                  || (fabsf(sp->vel_x) > 0.01f)
                                  || (fabsf(sp->vel_y) > 0.01f);
                if (flow_corr_en && flow_hold_is_active(&g_flow_hold)) {
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

        /* 遥测 20Hz（每 5 拍一次）+ http_server 内部异步发送：全速 100Hz
         * 同步发送在 WiFi 拥塞时会阻塞控制循环 —— 电机保持旧 PWM，等效失控 */
        if (++telem_div >= 5) {
            telem_div = 0;
            build_telemetry(json, sizeof(json), &imu, roll, pitch, yaw,
                            tof_mm, &flow, pid_r, pid_p, pid_y);
            http_server_broadcast(json);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
