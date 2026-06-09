#include "mpu6050.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "mpu6050";

/* ---- MPU6050 Registers ---- */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* ---- Conversion factors ---- */
#define GYRO_SCALE_2000     (1.0f / 16.4f)   /* LSB to °/s */
#define ACCEL_SCALE_16G     (1.0f / 2048.0f)  /* LSB to g */
#define DEG2RAD             0.017453293f       /* PI / 180 */
#define GRAVITY             9.80665f           /* m/s^2 per g */

/* ---- State ---- */
static i2c_master_dev_handle_t g_dev;
static float g_gyro_offs[3];    /* rad/s */
static float g_accel_offs[3];   /* m/s^2, z offset NOT removed */

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(g_dev, buf, 2, 10);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(g_dev, &reg, 1, buf, len, 10);
}

/* ---- Calibration ---- */
static void mpu6050_calibrate(void)
{
    const int samples = 500;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[14];
        if (read_reg(REG_ACCEL_XOUT_H, buf, 14) == ESP_OK) {
            sum_ax += (int16_t)((buf[0]  << 8) | buf[1]);
            sum_ay += (int16_t)((buf[2]  << 8) | buf[3]);
            sum_az += (int16_t)((buf[4]  << 8) | buf[5]);
            sum_gx += (int16_t)((buf[8]  << 8) | buf[9]);
            sum_gy += (int16_t)((buf[10] << 8) | buf[11]);
            sum_gz += (int16_t)((buf[12] << 8) | buf[13]);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    /* Gyro: average = offset (should be ~0 at rest) */
    g_gyro_offs[0] = (float)(sum_gx / samples) * GYRO_SCALE_2000 * DEG2RAD;
    g_gyro_offs[1] = (float)(sum_gy / samples) * GYRO_SCALE_2000 * DEG2RAD;
    g_gyro_offs[2] = (float)(sum_gz / samples) * GYRO_SCALE_2000 * DEG2RAD;

    /* Accel: x/y ~0, z ~1g. Only zero x/y; keep z for gravity reference */
    g_accel_offs[0] = (float)(sum_ax / samples) * ACCEL_SCALE_16G * GRAVITY;
    g_accel_offs[1] = (float)(sum_ay / samples) * ACCEL_SCALE_16G * GRAVITY;
    g_accel_offs[2] = 0.0f;

    ESP_LOGI(TAG, "calib done: gyro_offs=(%.4f,%.4f,%.4f) rad/s, "
             "accel_offs=(%.3f,%.3f) m/s^2",
             g_gyro_offs[0], g_gyro_offs[1], g_gyro_offs[2],
             g_accel_offs[0], g_accel_offs[1]);
}

/* ---- Public API ---- */

int mpu6050_init(void)
{
    /* Probe device first */
    esp_err_t ret = i2c_master_probe(g_i2c0_bus, MPU6050_ADDR, 50);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "device not found at 0x%02X (%s), check AD0/wiring",
                 MPU6050_ADDR, esp_err_to_name(ret));
        return -1;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = 400000,
    };
    ret = i2c_master_bus_add_device(g_i2c0_bus, &dev_cfg, &g_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* Verify WHO_AM_I with retries */
    uint8_t whoami = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        if (read_reg(REG_WHO_AM_I, &whoami, 1) == ESP_OK && whoami == 0x68) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (whoami != 0x68) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x68 got 0x%02X", whoami);
        return -1;
    }

    /* Wake up: clear sleep bit */
    write_reg(REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Sample rate: 0 = max (8kHz gyro, 1kHz accel) */
    write_reg(REG_SMPLRT_DIV, 0x00);

    /* DLPF: 4 = ~21Hz accel BW, ~20Hz gyro BW — kills motor vibration */
    write_reg(REG_CONFIG, 0x04);

    /* Gyro full scale: ±2000°/s */
    write_reg(REG_GYRO_CONFIG, 0x18);  /* FS_SEL = 3 */

    /* Accel full scale: ±16g */
    write_reg(REG_ACCEL_CONFIG, 0x18); /* AFS_SEL = 3 */

    /* Calibrate offsets (drone must be stationary!) */
    mpu6050_calibrate();

    ESP_LOGI(TAG, "init ok: addr=0x%02X, gyro=±2000dps accel=±16g",
             MPU6050_ADDR);
    return 0;
}

int mpu6050_recalibrate_gyro(void)
{
    ESP_LOGW(TAG, "=== GYRO + ACCEL RECALIBRATION ===");
    ESP_LOGW(TAG, "Keep drone COMPLETELY STILL and LEVEL for 1 second...");

    const int samples = 100;   /* 100 samples @ 100Hz = 1s */
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int64_t sum_ax = 0, sum_ay = 0;
    int valid = 0;

    for (int i = 0; i < samples; i++) {
        uint8_t buf[14];
        if (read_reg(REG_ACCEL_XOUT_H, buf, 14) == ESP_OK) {
            sum_ax += (int16_t)((buf[0]  << 8) | buf[1]);
            sum_ay += (int16_t)((buf[2]  << 8) | buf[3]);
            sum_gx += (int16_t)((buf[8]  << 8) | buf[9]);
            sum_gy += (int16_t)((buf[10] << 8) | buf[11]);
            sum_gz += (int16_t)((buf[12] << 8) | buf[13]);
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid < samples / 2) {
        ESP_LOGE(TAG, "recalibration failed: too many read errors");
        return -1;
    }

    /* 更新陀螺仪零偏 */
    g_gyro_offs[0] = (float)(sum_gx / valid) * GYRO_SCALE_2000 * DEG2RAD;
    g_gyro_offs[1] = (float)(sum_gy / valid) * GYRO_SCALE_2000 * DEG2RAD;
    g_gyro_offs[2] = (float)(sum_gz / valid) * GYRO_SCALE_2000 * DEG2RAD;

    /* 同步更新加速度计零偏 — 以当前放置面为"水平"基准，
     * 消除上电面与起飞面不一致导致的姿态偏差 */
    g_accel_offs[0] = (float)(sum_ax / valid) * ACCEL_SCALE_16G * GRAVITY;
    g_accel_offs[1] = (float)(sum_ay / valid) * ACCEL_SCALE_16G * GRAVITY;

    ESP_LOGI(TAG, "recal done: gyro=(%.4f,%.4f,%.4f) rad/s, accel=(%.4f,%.4f) m/s^2",
             g_gyro_offs[0], g_gyro_offs[1], g_gyro_offs[2],
             g_accel_offs[0], g_accel_offs[1]);
    return 0;
}

int mpu6050_read(mpu6050_data_t *out)
{
    if (!out) return -1;

    uint8_t buf[14];
    if (read_reg(REG_ACCEL_XOUT_H, buf, 14) != ESP_OK) {
        memset(out, 0, sizeof(*out));
        return -1;
    }

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature */
    int16_t temp = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    out->accel_x = (float)ax * ACCEL_SCALE_16G * GRAVITY - g_accel_offs[0];
    out->accel_y = (float)ay * ACCEL_SCALE_16G * GRAVITY - g_accel_offs[1];
    out->accel_z = (float)az * ACCEL_SCALE_16G * GRAVITY;
    out->gyro_x  = (float)gx * GYRO_SCALE_2000 * DEG2RAD - g_gyro_offs[0];
    out->gyro_y  = (float)gy * GYRO_SCALE_2000 * DEG2RAD - g_gyro_offs[1];
    out->gyro_z  = (float)gz * GYRO_SCALE_2000 * DEG2RAD - g_gyro_offs[2];
    out->temp_c  = (float)temp / 340.0f + 36.53f;
    out->data_ready = true;

    return 0;
}
