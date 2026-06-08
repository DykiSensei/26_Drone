#include "params.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

static const char *TAG = "params";
static const char *NS  = "drone";
static const char *KEY_HOVER      = "hover_thr";
static const char *KEY_FLOW_SCALE = "flow_scale";

/* 学习率：100Hz × α=0.001 → 时间常数 ~10s，避免瞬时扰动污染学习 */
#define HOVER_LEARN_ALPHA  0.001f
/* NVS 写入差距阈值：避免每次小变化都写 flash */
#define HOVER_SAVE_THRESHOLD       0.01f
#define FLOW_SCALE_SAVE_THRESHOLD  0.05f  /* IMU scale 是用户手动调的，差 0.05 就值得存 */

static float g_hover_throttle  = HOVER_THROTTLE_DEFAULT;
static float g_hover_saved     = HOVER_THROTTLE_DEFAULT;
static float g_flow_imu_scale  = FLOW_IMU_SCALE_DEFAULT;
static float g_flow_scale_saved = FLOW_IMU_SCALE_DEFAULT;

/* float ↔ uint32_t bit-cast（NVS 没有 float API，用 u32 存最简洁） */
static uint32_t f2u(float f) { uint32_t u; memcpy(&u, &f, sizeof(u)); return u; }
static float    u2f(uint32_t u) { float f; memcpy(&f, &u, sizeof(f)); return f; }

int params_init(void)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NS, NVS_READONLY, &h);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open '%s' failed: %s — using defaults", NS, esp_err_to_name(r));
        return 0;
    }

    uint32_t v;
    if (nvs_get_u32(h, KEY_HOVER, &v) == ESP_OK) {
        float f = u2f(v);
        if (isfinite(f) && f >= HOVER_THROTTLE_MIN && f <= HOVER_THROTTLE_MAX) {
            g_hover_throttle = f;
            g_hover_saved    = f;
            ESP_LOGW(TAG, "loaded hover_throttle=%.3f from NVS", f);
        } else {
            ESP_LOGW(TAG, "NVS hover_throttle out of range (%.3f), using default", f);
        }
    } else {
        ESP_LOGI(TAG, "no hover_throttle in NVS, using default %.2f", HOVER_THROTTLE_DEFAULT);
    }

    if (nvs_get_u32(h, KEY_FLOW_SCALE, &v) == ESP_OK) {
        float f = u2f(v);
        if (isfinite(f) && f >= FLOW_IMU_SCALE_MIN && f <= FLOW_IMU_SCALE_MAX) {
            g_flow_imu_scale  = f;
            g_flow_scale_saved = f;
            ESP_LOGW(TAG, "loaded flow_imu_scale=%.3f from NVS", f);
        } else {
            ESP_LOGW(TAG, "NVS flow_imu_scale out of range (%.3f), using default", f);
        }
    } else {
        ESP_LOGI(TAG, "no flow_imu_scale in NVS, using default %.2f", FLOW_IMU_SCALE_DEFAULT);
    }

    nvs_close(h);
    return 0;
}

float params_get_hover_throttle(void)
{
    return g_hover_throttle;
}

void params_update_hover_throttle(float actual)
{
    if (!isfinite(actual)) return;
    if (actual < HOVER_THROTTLE_MIN || actual > HOVER_THROTTLE_MAX) return;
    g_hover_throttle += HOVER_LEARN_ALPHA * (actual - g_hover_throttle);
}

float params_get_flow_imu_scale(void)
{
    return g_flow_imu_scale;
}

void params_set_flow_imu_scale(float s)
{
    if (!isfinite(s)) return;
    if (s < FLOW_IMU_SCALE_MIN) s = FLOW_IMU_SCALE_MIN;
    if (s > FLOW_IMU_SCALE_MAX) s = FLOW_IMU_SCALE_MAX;
    g_flow_imu_scale = s;
}

void params_save(void)
{
    bool hover_dirty = (fabsf(g_hover_throttle  - g_hover_saved)      >= HOVER_SAVE_THRESHOLD);
    bool scale_dirty = (fabsf(g_flow_imu_scale  - g_flow_scale_saved) >= FLOW_SCALE_SAVE_THRESHOLD);
    if (!hover_dirty && !scale_dirty) return;

    nvs_handle_t h;
    esp_err_t r = nvs_open(NS, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open RW failed: %s", esp_err_to_name(r));
        return;
    }

    bool ok = true;
    if (hover_dirty) {
        if (nvs_set_u32(h, KEY_HOVER, f2u(g_hover_throttle)) != ESP_OK) ok = false;
    }
    if (scale_dirty) {
        if (nvs_set_u32(h, KEY_FLOW_SCALE, f2u(g_flow_imu_scale)) != ESP_OK) ok = false;
    }
    if (ok && nvs_commit(h) == ESP_OK) {
        if (hover_dirty) {
            g_hover_saved = g_hover_throttle;
            ESP_LOGW(TAG, "saved hover_throttle=%.3f to NVS", g_hover_throttle);
        }
        if (scale_dirty) {
            g_flow_scale_saved = g_flow_imu_scale;
            ESP_LOGW(TAG, "saved flow_imu_scale=%.3f to NVS", g_flow_imu_scale);
        }
    } else {
        ESP_LOGE(TAG, "nvs save failed");
    }
    nvs_close(h);
}
