#include "servo_grip.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "servo_grip";

/* ── 硬件参数 ── */
#define GRIP_GPIO         10
#define GRIP_LEDC_TIMER   LEDC_TIMER_1     /* TIMER_0 被电机 400Hz 占用，舵机 50Hz 独立定时器 */
#define GRIP_LEDC_CH      LEDC_CHANNEL_4   /* 通道 0–3 被电机占用 */
#define GRIP_FREQ_HZ      50               /* 标准舵机 PWM 周期 20ms */
#define GRIP_PERIOD_US    (1000000 / GRIP_FREQ_HZ)
#define GRIP_RESOLUTION   LEDC_TIMER_14_BIT
#define GRIP_MAX_DUTY     ((1 << 14) - 1)  /* 16383 → ~1.22us/步，远小于 MG995 死区 */

/* ── 脉宽 ↔ 角度（沿用 esp32s3-mg995-servo-web 库已验证参数）──
 * 映射保持全量程 0–180°↔500–2500µs；可用范围由 SERVO_GRIP_MAX_DEG 硬限位 */
#define GRIP_MIN_PULSE_US 500              /* 0° */
#define GRIP_MAX_PULSE_US 2500             /* 180° */
#define GRIP_ANGLE_FULL   180.0f           /* 脉宽映射满量程（勿用于钳位） */

/* 限速逼近速度：90° 行程约 0.75s。太快 = 电流尖峰 + 飞行中反扭矩 */
#define GRIP_SPEED_DPS    120.0f

static bool  g_initialized  = false;
static float g_angle_now    = SERVO_GRIP_OPEN_DEG;  /* 当前（限速后）输出角 */
static float g_angle_target = SERVO_GRIP_OPEN_DEG;

static void grip_write(float deg)
{
    float pulse_us = GRIP_MIN_PULSE_US
                   + deg / GRIP_ANGLE_FULL * (float)(GRIP_MAX_PULSE_US - GRIP_MIN_PULSE_US);
    uint32_t duty = (uint32_t)((float)GRIP_MAX_DUTY * pulse_us / (float)GRIP_PERIOD_US);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, GRIP_LEDC_CH, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, GRIP_LEDC_CH);
}

static float clamp_angle(float deg)
{
    if (deg < 0.0f) return 0.0f;
    /* 硬限位 90°：齿条行程极限，超过舵机空转打滑（2026-07-05 台架实测）。
     * 这里是最后一道防线——无论 commander/前端/未来 P4 状态机发什么都钳住 */
    if (deg > SERVO_GRIP_MAX_DEG) return SERVO_GRIP_MAX_DEG;
    return deg;
}

int servo_grip_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = GRIP_RESOLUTION,
        .timer_num       = GRIP_LEDC_TIMER,
        .freq_hz         = GRIP_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "timer config fail: %s", esp_err_to_name(ret));
        return -1;
    }

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = GRIP_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = GRIP_LEDC_CH,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = GRIP_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "channel config GPIO%d fail: %s", GRIP_GPIO, esp_err_to_name(ret));
        return -1;
    }

    g_angle_now = g_angle_target = SERVO_GRIP_OPEN_DEG;
    grip_write(g_angle_now);
    g_initialized = true;
    ESP_LOGI(TAG, "init ok: GPIO%d, %dHz, [%d-%d]us, open=%.0f° close=%.0f°",
             GRIP_GPIO, GRIP_FREQ_HZ, GRIP_MIN_PULSE_US, GRIP_MAX_PULSE_US,
             SERVO_GRIP_OPEN_DEG, SERVO_GRIP_CLOSE_DEG);
    return 0;
}

void servo_grip_set_angle(float deg)
{
    g_angle_target = clamp_angle(deg);
}

void servo_grip_open(void)
{
    g_angle_target = SERVO_GRIP_OPEN_DEG;
}

void servo_grip_close(void)
{
    g_angle_target = SERVO_GRIP_CLOSE_DEG;
}

void servo_grip_update(float dt)
{
    if (!g_initialized) return;
    float err = g_angle_target - g_angle_now;
    if (fabsf(err) < 0.01f) return;   /* 到位后不再重写占空比 */
    float step = GRIP_SPEED_DPS * dt;
    if (err >  step) err =  step;
    if (err < -step) err = -step;
    g_angle_now += err;
    grip_write(g_angle_now);
}

float servo_grip_get_angle(void)
{
    return g_angle_now;
}

bool servo_grip_is_moving(void)
{
    return g_initialized && fabsf(g_angle_target - g_angle_now) >= 0.01f;
}
