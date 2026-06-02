#include "commander.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "commander";

static setpoint_t g_sp = {
    .throttle     = 0.0f,
    .roll         = 0.0f,
    .pitch        = 0.0f,
    .yaw          = 0.0f,
    .mode         = MODE_DISARMED,
    .motor        = {0.0f, 0.0f, 0.0f, 0.0f},
    .motor_active = false,
    .mtrim        = {0.0f, 0.0f, 0.0f, 0.0f},
};

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

    cJSON *item;

    item = cJSON_GetObjectItem(root, "throttle");
    if (cJSON_IsNumber(item))
        g_sp.throttle = clamp((float)item->valuedouble, 0.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "roll");
    if (cJSON_IsNumber(item))
        g_sp.roll = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "pitch");
    if (cJSON_IsNumber(item))
        g_sp.pitch = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "yaw");
    if (cJSON_IsNumber(item))
        g_sp.yaw = clamp((float)item->valuedouble, -1.0f, 1.0f);

    item = cJSON_GetObjectItem(root, "mode");
    if (cJSON_IsString(item))
        g_sp.mode = parse_mode(item->valuestring);

    /* Per-motor manual control */
    item = cJSON_GetObjectItem(root, "motor");
    if (cJSON_IsArray(item) && cJSON_GetArraySize(item) >= 4) {
        bool any_nonzero = false;
        for (int i = 0; i < 4; i++) {
            cJSON *elem = cJSON_GetArrayItem(item, i);
            if (cJSON_IsNumber(elem)) {
                g_sp.motor[i] = clamp((float)elem->valuedouble, 0.0f, 1.0f);
                if (g_sp.motor[i] > 0.0f) any_nonzero = true;
            }
        }
        g_sp.motor_active = any_nonzero;
    } else {
        g_sp.motor_active = false;
    }

    /* Special commands (calibrate, etc.) — deferred to main loop */
    item = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(item)) {
        if (strcmp(item->valuestring, "calibrate") == 0)
            g_sp.pending_cmd = CMD_CALIBRATE;
        else if (strcmp(item->valuestring, "gyro_calib") == 0)
            g_sp.pending_cmd = CMD_GYRO_CALIB;
        else if (strcmp(item->valuestring, "level_trim") == 0)
            g_sp.pending_cmd = CMD_LEVEL_TRIM;
        else if (strcmp(item->valuestring, "reset_trim") == 0)
            g_sp.pending_cmd = CMD_RESET_TRIM;
        else if (strcmp(item->valuestring, "calibrate_motor") == 0) {
            cJSON *idx = cJSON_GetObjectItem(root, "motor_index");
            if (cJSON_IsNumber(idx)) {
                g_sp.calib_motor = (int)idx->valuedouble;
                g_sp.pending_cmd = CMD_CALIBRATE_MOTOR;
            }
        }
    }

    /* Per-motor trim */
    item = cJSON_GetObjectItem(root, "mtrim");
    if (cJSON_IsArray(item) && cJSON_GetArraySize(item) >= 4) {
        for (int i = 0; i < 4; i++) {
            cJSON *elem = cJSON_GetArrayItem(item, i);
            if (cJSON_IsNumber(elem))
                g_sp.mtrim[i] = clamp((float)elem->valuedouble, -0.15f, 0.15f);
        }
    }

    cJSON_Delete(root);

    ESP_LOGD(TAG, "cmd: thr=%.2f r=%.2f p=%.2f y=%.2f mode=%s",
             g_sp.throttle, g_sp.roll, g_sp.pitch, g_sp.yaw,
             commander_mode_name(g_sp.mode));
}

const setpoint_t *commander_get_setpoint(void)
{
    return &g_sp;
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
