#include "motor.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor";

/* ── PWM timing ── */
#define PWM_FREQ_HZ      400          /* Standard ESC PWM frequency */
#define PWM_MIN_US       1000         /* 0% throttle — motor stopped */
#define PWM_MAX_US       2000         /* 100% throttle */
#define PWM_PERIOD_US    (1000000 / PWM_FREQ_HZ)  /* 2500 us */

/* LEDC resolution: enough for ~0.3 us per step */
#define LEDC_RESOLUTION  LEDC_TIMER_13_BIT
#define LEDC_MAX_DUTY    ((1 << 13) - 1)  /* 8191 */

/* GPIOs for 4 motors (X-quad config) */
static const int MOTOR_GPIO[MOTOR_COUNT] = {
    14,   /* Motor 0: Front-Right */
    11,   /* Motor 1: Front-Left  */
    13,   /* Motor 2: Rear-Left   */
    12,   /* Motor 3: Rear-Right  */
};

static bool g_initialized = false;

/* ── Convert 0.0–1.0 throttle to LEDC duty ── */
static uint32_t throttle_to_duty(float t)
{
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    uint32_t pulse_us = (uint32_t)(PWM_MIN_US + t * (PWM_MAX_US - PWM_MIN_US));
    return (uint32_t)((uint64_t)LEDC_MAX_DUTY * pulse_us / PWM_PERIOD_US);
}

/* ── Set single motor ── */
static void motor_write(int index, float throttle)
{
    uint32_t duty = throttle_to_duty(throttle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)index, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)index);
}

int motor_init(void)
{
    /* Timer config: 400 Hz, 13-bit resolution */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "timer config fail: %s", esp_err_to_name(ret));
        return -1;
    }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = MOTOR_GPIO[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0,
        };
        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ch%d GPIO%d fail: %s", i, MOTOR_GPIO[i],
                     esp_err_to_name(ret));
            return -1;
        }
    }

    /* Output MIN pulse to all motors for safe startup */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_write(i, 0.0f);
    }

    g_initialized = true;
    ESP_LOGI(TAG, "init ok: %d motors, %dHz, [%d-%d]us, GPIOs %d,%d,%d,%d",
             MOTOR_COUNT, PWM_FREQ_HZ, PWM_MIN_US, PWM_MAX_US,
             MOTOR_GPIO[0], MOTOR_GPIO[1], MOTOR_GPIO[2], MOTOR_GPIO[3]);
    return 0;
}

void motor_set(const float motor[MOTOR_COUNT])
{
    if (!g_initialized) return;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_write(i, motor[i]);
    }
}

void motor_stop(void)
{
    if (!g_initialized) return;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_write(i, 0.0f);
    }
    ESP_LOGD(TAG, "all motors stopped");
}

void motor_calibrate(void)
{
    if (!g_initialized) return;

    ESP_LOGW(TAG, "=== ESC CALIBRATION ===");
    ESP_LOGW(TAG, "1. REMOVE propellers now!");
    ESP_LOGW(TAG, "2. DISCONNECT ESC battery");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Output MAX to enter calibration mode */
    ESP_LOGW(TAG, "Sending MAX throttle (%d us)...", PWM_MAX_US);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_write(i, 1.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGW(TAG, ">>> Connect ESC battery NOW <<<");
    ESP_LOGW(TAG, "Waiting 6s for ESC beeps...");
    vTaskDelay(pdMS_TO_TICKS(6000));

    /* After beeps, set MIN to confirm calibration */
    ESP_LOGW(TAG, "Sending MIN throttle (%d us)...", PWM_MIN_US);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_write(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGW(TAG, "=== Calibration done ===");
    ESP_LOGW(TAG, "ESCs should confirm with long beep");
}

void motor_calibrate_single(int index)
{
    if (!g_initialized) return;
    if (index < 0 || index >= MOTOR_COUNT) return;

    ESP_LOGW(TAG, "=== ESC CALIBRATION: Motor %d (GPIO%d) ===", index, MOTOR_GPIO[index]);
    ESP_LOGW(TAG, "1. REMOVE propeller from motor %d!", index);
    ESP_LOGW(TAG, "2. Other motors will stay at MIN");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* All motors to MIN first */
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_write(i, 0.0f);
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Target motor to MAX */
    ESP_LOGW(TAG, "Motor %d → MAX (%d us)...", index, PWM_MAX_US);
    motor_write(index, 1.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGW(TAG, ">>> Connect ESC battery for motor %d NOW <<<", index);
    ESP_LOGW(TAG, "Waiting 6s for ESC beeps...");
    vTaskDelay(pdMS_TO_TICKS(6000));

    /* Target motor to MIN to confirm */
    ESP_LOGW(TAG, "Motor %d → MIN (%d us)...", index, PWM_MIN_US);
    motor_write(index, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGW(TAG, "=== Motor %d calibration done ===", index);
    ESP_LOGW(TAG, "ESC should confirm with long beep");
}
