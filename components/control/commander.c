#include "commander.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "commander";

static setpoint_t g_sp = {
    .throttle     = 0.0f,
    .roll         = 0.0f,
    .pitch        = 0.0f,
    .yaw          = 0.0f,
    .vel_x        = 0.0f,
    .vel_y        = 0.0f,
    .mode         = MODE_DISARMED,
    .motor        = {0.0f, 0.0f, 0.0f, 0.0f},
    .motor_active = false,
    .mtrim        = {0.0f, 0.0f, 0.0f, 0.0f},
    .calib_motor  = 0,
    .move_to_x    = 0.0f,
    .move_to_y    = 0.0f,
    .pending_cmd  = CMD_NONE,
};

static int64_t g_last_command_us = 0;
static commander_cmd_cb_t g_cmd_cb = NULL;

static flight_mode_t parse_mode(const char *s)
{
    if (!s) return MODE_DISARMED;
    if (strcmp(s, "stabilize") == 0) return MODE_STABILIZE;
    if (strcmp(s, "alt_hold") == 0)  return MODE_ALT_HOLD;
    if (strcmp(s, "pos_hold") == 0)  return MODE_POS_HOLD;
    return MODE_DISARMED;
}

static float clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void commander_parse(const char *json, int len)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (!root) {
        ESP_LOGD(TAG, "invalid JSON: %.*s", len, json);
        return;
    }

    /* 先复制当前 g_sp 到局部变量，在此基础上修改 */
    setpoint_t sp = g_sp;
    sp.pending_cmd = CMD_NONE;  /* 每次解析重置，防止旧命令残留 */

    cJSON *item;

    item = cJSON_GetObjectItem(root, "throttle");
    if (cJSON_IsNumber(item))
        sp.throttle = clamp((float)item->valuedouble, 0.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "roll");
    if (cJSON_IsNumber(item))
        sp.roll = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "pitch");
    if (cJSON_IsNumber(item))
        sp.pitch = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "yaw");
    if (cJSON_IsNumber(item))
        sp.yaw = clamp((float)item->valuedouble, -1.0f, 1.0f);

    /* 水平速度指令 */
    item = cJSON_GetObjectItem(root, "vel_x");
    if (cJSON_IsNumber(item))
        sp.vel_x = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "vel_y");
    if (cJSON_IsNumber(item))
        sp.vel_y = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsString(item))
        sp.mode = parse_mode(item->valuestring);

    /* Per-motor manual control */
    item = cJSON_GetObjectItem(root, "motor");
    if (cJSON_IsArray(item) && cJSON_GetArraySize(item) >= 4) {
        bool any_nonzero = false;
        for (int i = 0; i < 4; i++) {
            cJSON *elem = cJSON_GetArrayItem(item, i);
            if (cJSON_IsNumber(elem)) {
                sp.motor[i] = clamp((float)elem->valuedouble, 0.0f, 1.0f);
                if (sp.motor[i] > 0.0f) any_nonzero = true;
            }
        }
        sp.motor_active = any_nonzero;
    } else {
        sp.motor_active = false;
    }

    /* Special commands (calibrate, etc.) — deferred to main loop */
    item = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(item)) {
        if (strcmp(item->valuestring, "calibrate") == 0)
            sp.pending_cmd = CMD_CALIBRATE;
        else if (strcmp(item->valuestring, "gyro_calib") == 0)
            sp.pending_cmd = CMD_GYRO_CALIB;
        else if (strcmp(item->valuestring, "level_trim") == 0)
            sp.pending_cmd = CMD_LEVEL_TRIM;
        else if (strcmp(item->valuestring, "reset_trim") == 0)
            sp.pending_cmd = CMD_RESET_TRIM;
        else if (strcmp(item->valuestring, "calibrate_motor") == 0) {
            cJSON *idx = cJSON_GetObjectItem(root, "motor_index");
            if (cJSON_IsNumber(idx)) {
                sp.calib_motor = (int)idx->valuedouble;
                sp.pending_cmd = CMD_CALIBRATE_MOTOR;
            }
        }
        else if (strcmp(item->valuestring, "move_to") == 0) {
            cJSON *x = cJSON_GetObjectItem(root, "x");
            cJSON *y = cJSON_GetObjectItem(root, "y");
            if (cJSON_IsNumber(x)) sp.move_to_x = (float)x->valuedouble;
            if (cJSON_IsNumber(y)) sp.move_to_y = (float)y->valuedouble;
            sp.pending_cmd = CMD_MOVE_TO;
        }
        else if (strcmp(item->valuestring, "move_stop") == 0) {
            sp.pending_cmd = CMD_MOVE_STOP;
        }
        else if (strcmp(item->valuestring, "takeoff") == 0) {
            cJSON *h = cJSON_GetObjectItem(root, "height");
            cJSON *t = cJSON_GetObjectItem(root, "base_throttle");
            if (cJSON_IsNumber(h))
                sp.takeoff_height = clamp((float)h->valuedouble, 0.2f, 2.0f);
            if (cJSON_IsNumber(t))
                sp.takeoff_throttle = clamp((float)t->valuedouble, 0.25f, 0.6f);
            sp.pending_cmd = CMD_TAKEOFF;
        }
    }

    /* Per-motor trim */
    item = cJSON_GetObjectItem(root, "mtrim");
    if (cJSON_IsArray(item) && cJSON_GetArraySize(item) >= 4) {
        for (int i = 0; i < 4; i++) {
            cJSON *elem = cJSON_GetArrayItem(item, i);
            if (cJSON_IsNumber(elem))
                sp.mtrim[i] = clamp((float)elem->valuedouble, -0.15f, 0.15f);
        }
    }

    cJSON_Delete(root);

    /* 原子写入：一次性 struct 赋值，最小化竞态窗口 */
    g_sp = sp;
    g_last_command_us = esp_timer_get_time();

    ESP_LOGD(TAG, "cmd: thr=%.2f r=%.2f p=%.2f y=%.2f vx=%.2f vy=%.2f mode=%s",
             g_sp.throttle, g_sp.roll, g_sp.pitch, g_sp.yaw,
             g_sp.vel_x, g_sp.vel_y,
             commander_mode_name(g_sp.mode));
}

const setpoint_t *commander_get_setpoint(void)
{
    return &g_sp;
}

void commander_reset_setpoint(void)
{
    setpoint_t safe = {
        .throttle     = 0.0f,
        .roll         = 0.0f,
        .pitch        = 0.0f,
        .yaw          = 0.0f,
        .vel_x        = 0.0f,
        .vel_y        = 0.0f,
        .mode         = MODE_DISARMED,
        .motor        = {0.0f, 0.0f, 0.0f, 0.0f},
        .motor_active = false,
        .mtrim        = {0.0f, 0.0f, 0.0f, 0.0f},
        .calib_motor  = 0,
        .move_to_x    = 0.0f,
        .move_to_y    = 0.0f,
        .takeoff_height   = 0.5f,
        .takeoff_throttle = 0.4f,
        .pending_cmd  = CMD_NONE,
    };
    g_sp = safe;
    g_last_command_us = 0;  /* 重置时间戳，避免循环触发超时 */
    ESP_LOGW(TAG, "setpoint reset to DISARMED (safety)");
}

bool commander_is_command_timeout(void)
{
    if (g_last_command_us == 0) return false;  /* 还没收到过命令，不算超时 */
    return (esp_timer_get_time() - g_last_command_us) > COMMAND_TIMEOUT_US;
}

const char *commander_mode_name(flight_mode_t mode)
{
    switch (mode) {
    case MODE_DISARMED:  return "disarmed";
    case MODE_STABILIZE: return "stabilize";
    case MODE_ALT_HOLD:  return "alt_hold";
    case MODE_POS_HOLD:  return "pos_hold";
    default:             return "unknown";
    }
}

void commander_set_cmd_callback(commander_cmd_cb_t cb)
{
    g_cmd_cb = cb;
}

void commander_clear_pending_cmd(void)
{
    g_sp.pending_cmd = CMD_NONE;
}
