#include "tof400f.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "tof400f";

/* 缓存过期时间：传感器正常 ~10Hz 出数，超过 500ms 没有新样本说明它死了。
 * 过期后必须报告失效，不能永远返回旧高度 —— 定高环拿着假高度会一直悬在
 * 错误的油门上（甚至持续爬升/下坠）。 */
#define TOF_STALE_US  500000

static i2c_master_dev_handle_t g_dev_handle;
static uint16_t g_last_valid_mm = 0;
static int64_t  g_last_fresh_us = 0;

/* 返回缓存值；缓存过期或从未有效则报失效（*out=0），上层据此跳过定高环 */
static int cached_result(uint16_t *out)
{
    if (g_last_valid_mm == 0 ||
        (esp_timer_get_time() - g_last_fresh_us) > TOF_STALE_US) {
        *out = 0;
        return -1;
    }
    *out = g_last_valid_mm;
    return 0;
}

/* ── VL53L1X register addresses from ST ULD ── */
#define VL53L1_SYSTEM__MODE_START                        0x0087
#define VL53L1_SYSTEM__INTERRUPT_CLEAR                   0x0086
#define VL53L1_RESULT__RANGE_STATUS                      0x0089
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE    0x0096
#define VL53L1_GPIO__TIO_HV_STATUS                       0x0031
#define VL53L1_GPIO_HV_MUX__CTRL                         0x0030
#define VL53L1_IDENTIFICATION__MODEL_ID                   0x010F
#define VL53L1_FIRMWARE__SYSTEM_STATUS                    0x00E5
#define VL53L1_RESULT__OSC_CALIBRATE_VAL                  0x00DE
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND      0x0008
#define VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP             0x004B
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A              0x005E
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B              0x0061
#define VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A                0x0060
#define VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B                0x0063
#define VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH              0x0069
#define VL53L1_RANGE_CONFIG__SIGMA_THRESH                  0x0064
#define VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066
#define VL53L1_SD_CONFIG__WOI_SD0                          0x0078
#define VL53L1_SD_CONFIG__INITIAL_PHASE_SD0                0x007A
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD             0x006C

/* ── Default configuration blob (registers 0x2D–0x87) from ST ULD ── */
static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00
};

/* ── I2C platform layer ── */

static int vl53l1_write_multi(uint16_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[2 + len];
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (size_t i = 0; i < len; i++) buf[2 + i] = data[i];
    esp_err_t ret = i2c_master_transmit(g_dev_handle, buf, 2 + len, 10);
    return (ret == ESP_OK) ? 0 : -1;
}

static int vl53l1_read_multi(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t reg_buf[2];
    reg_buf[0] = (uint8_t)(reg >> 8);
    reg_buf[1] = (uint8_t)(reg & 0xFF);
    esp_err_t ret = i2c_master_transmit_receive(g_dev_handle, reg_buf, 2, data, len, 10);
    return (ret == ESP_OK) ? 0 : -1;
}

static int vl53l1_wr_byte(uint16_t reg, uint8_t val)
{
    return vl53l1_write_multi(reg, &val, 1);
}

static int vl53l1_wr_word(uint16_t reg, uint16_t val)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
    return vl53l1_write_multi(reg, buf, 2);
}

static int vl53l1_rd_byte(uint16_t reg, uint8_t *val)
{
    return vl53l1_read_multi(reg, val, 1);
}

static int vl53l1_rd_word(uint16_t reg, uint16_t *val)
{
    uint8_t buf[2];
    if (vl53l1_read_multi(reg, buf, 2) != 0) return -1;
    *val = ((uint16_t)buf[0] << 8) | buf[1];
    return 0;
}

/* ── VL53L1X API functions (from ST ULD) ── */

static int vl53l1x_start_ranging(void)
{
    return vl53l1_wr_byte(VL53L1_SYSTEM__MODE_START, 0x40);
}

static int vl53l1x_stop_ranging(void)
{
    return vl53l1_wr_byte(VL53L1_SYSTEM__MODE_START, 0x00);
}

static int vl53l1x_clear_interrupt(void)
{
    return vl53l1_wr_byte(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);
}

static int vl53l1x_get_interrupt_polarity(uint8_t *polarity)
{
    uint8_t temp;
    if (vl53l1_rd_byte(VL53L1_GPIO_HV_MUX__CTRL, &temp) != 0) return -1;
    temp &= 0x10;
    *polarity = !(temp >> 4);
    return 0;
}

static int vl53l1x_check_data_ready(bool *ready)
{
    uint8_t int_pol, temp;
    if (vl53l1x_get_interrupt_polarity(&int_pol) != 0) return -1;
    if (vl53l1_rd_byte(VL53L1_GPIO__TIO_HV_STATUS, &temp) != 0) return -1;
    *ready = ((temp & 1) == int_pol);
    return 0;
}

static int vl53l1x_get_distance(uint16_t *dist)
{
    return vl53l1_rd_word(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE, dist);
}

static int vl53l1x_set_timing_budget_ms(uint16_t budget_ms)
{
    /*
     * First need to know distance mode. Default is LONG (2).
     * Read back PHASECAL_CONFIG__TIMEOUT_MACROP to determine.
     */
    uint8_t dm;
    if (vl53l1_rd_byte(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, &dm) != 0) return -1;

    if (dm == 0x14) { /* Short distance mode */
        switch (budget_ms) {
        case 15:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x001D);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x0027); break;
        case 20:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x0051);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x006E); break;
        case 33:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x00D6);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x006E); break;
        case 50:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x01AE);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x01E8); break;
        case 100: vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x02E1);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x0388); break;
        case 200: vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x03E1);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x0496); break;
        case 500: vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x0591);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x05C1); break;
        default: return -1;
        }
    } else { /* Long distance mode (default) */
        switch (budget_ms) {
        case 20:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x001E);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x0022); break;
        case 33:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x0060);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x006E); break;
        case 50:  vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x00AD);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x00C6); break;
        case 100: vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x01CC);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x01EA); break;
        case 200: vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x02D9);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x02F8); break;
        case 500: vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, 0x048F);
                   vl53l1_wr_word(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, 0x04A4); break;
        default: return -1;
        }
    }
    return 0;
}

static int vl53l1x_set_distance_mode_short(void)
{
    vl53l1_wr_byte(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
    vl53l1_wr_byte(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
    vl53l1_wr_byte(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
    vl53l1_wr_byte(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
    vl53l1_wr_word(VL53L1_SD_CONFIG__WOI_SD0, 0x0705);
    vl53l1_wr_word(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
    return vl53l1x_set_timing_budget_ms(50);
}

static int vl53l1x_set_distance_mode_long(void)
{
    vl53l1_wr_byte(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
    vl53l1_wr_byte(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
    vl53l1_wr_byte(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
    vl53l1_wr_byte(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
    vl53l1_wr_word(VL53L1_SD_CONFIG__WOI_SD0, 0x0F0D);
    vl53l1_wr_word(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
    return vl53l1x_set_timing_budget_ms(50);
}

static int vl53l1x_sensor_init(void)
{
    int rc;

    /* Write default configuration to registers 0x2D–0x87 */
    for (uint16_t addr = 0x2D; addr <= 0x87; addr++) {
        rc = vl53l1_wr_byte(addr, VL53L1X_DEFAULT_CONFIG[addr - 0x2D]);
        if (rc != 0) {
            i2c_master_bus_reset(g_i2c0_bus);
            ESP_LOGW(TAG, "config write retry at 0x%04X", addr);
            vTaskDelay(pdMS_TO_TICKS(1));
            rc = vl53l1_wr_byte(addr, VL53L1X_DEFAULT_CONFIG[addr - 0x2D]);
        }
        if (rc != 0) {
            ESP_LOGE(TAG, "config write fail at 0x%04X", addr);
            return -1;
        }
    }

    /* Start ranging to get first measurement, then stop */
    rc = vl53l1x_start_ranging();
    if (rc != 0) return -1;

    uint8_t ready = 0;
    int timeout = 100;
    while (ready == 0 && timeout-- > 0) {
        if (vl53l1x_check_data_ready((bool *)&ready) != 0) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vl53l1x_clear_interrupt();
    vl53l1x_stop_ranging();

    /* VHV config */
    vl53l1_wr_byte(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    vl53l1_wr_byte(0x0B, 0x00);

    return 0;
}

/* ── Public API ── */

int tof400f_init(void)
{
    esp_err_t ret;

    ret = i2c_master_probe(g_i2c0_bus, TOF400F_ADDR, 50);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "device not found at 0x%02X", TOF400F_ADDR);
        return -1;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = TOF400F_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(g_i2c0_bus, &dev_cfg, &g_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Verify VL53L1X by reading Model ID */
    uint16_t model_id = 0;
    if (vl53l1_rd_word(VL53L1_IDENTIFICATION__MODEL_ID, &model_id) == 0) {
        ESP_LOGI(TAG, "Model ID: 0x%04X (expect 0xEACC)", model_id);
        if (model_id != 0xEACC) {
            ESP_LOGW(TAG, "unexpected Model ID, continuing anyway");
        }
    }

    /* Full sensor init: load config, calibrate VHV */
    if (vl53l1x_sensor_init() != 0) {
        ESP_LOGE(TAG, "sensor init failed");
        return -1;
    }

    /* Set long distance mode (up to 4m), 100ms timing budget */
    vl53l1x_set_distance_mode_long();
    vl53l1x_set_timing_budget_ms(100);

    /* Start continuous ranging */
    if (vl53l1x_start_ranging() != 0) {
        ESP_LOGE(TAG, "start ranging failed");
        return -1;
    }

    g_last_valid_mm = 0;
    ESP_LOGI(TAG, "init ok, ranging started");
    return 0;
}

int tof400f_get_distance(uint16_t *distance_mm)
{
    if (!distance_mm) return -1;

    /*
     * Sensor produces new data every ~100ms (timing budget).
     * Main loop runs at 100Hz, so skip I2C checks most of the time.
     */
    static uint8_t skip_counter = 0;
    if (++skip_counter < 10) {
        return cached_result(distance_mm);
    }
    skip_counter = 0;

    bool ready = false;
    if (vl53l1x_check_data_ready(&ready) != 0 || !ready) {
        return cached_result(distance_mm);
    }

    vl53l1x_clear_interrupt();

    uint16_t dist;
    if (vl53l1x_get_distance(&dist) != 0) {
        return cached_result(distance_mm);
    }

    if (dist >= 40 && dist <= 4000) {
        g_last_valid_mm = dist;
        g_last_fresh_us = esp_timer_get_time();
        *distance_mm = dist;
        return 0;
    }

    return cached_result(distance_mm);
}
