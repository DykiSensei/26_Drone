#include "bn880_mag.h"
#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "bn880_mag";

typedef enum {
    MAG_CHIP_NONE = 0,
    MAG_CHIP_QMC5883L,
    MAG_CHIP_HMC5883L,
    MAG_CHIP_IST8310,
} mag_chip_t;

static mag_chip_t g_chip = MAG_CHIP_NONE;
static i2c_master_dev_handle_t g_dev;

/* ---- QMC5883L 寄存器 ---- */
#define QMC_REG_DATA      0x00  /* X LSB..Z MSB，小端 s16 ×3 */
#define QMC_REG_STATUS    0x06  /* bit0 = DRDY */
#define QMC_REG_CTRL1     0x09
#define QMC_REG_SET_RESET 0x0B
#define QMC_REG_CHIP_ID   0x0D  /* 恒为 0xFF */
/* CTRL1: OSR[7:6]=00(512) RNG[5:4]=00(±2G) ODR[3:2]=01(50Hz) MODE[1:0]=01(连续) */
#define QMC_CTRL1_CONFIG  0x05
#define QMC_LSB_PER_GAUSS 12000.0f   /* ±2G 量程 */

/* ---- IST8310 寄存器 ---- */
#define IST_REG_WHOAMI    0x00  /* 恒为 0x10 */
#define IST_REG_DATA      0x03  /* X,Y,Z 小端 s16 */
#define IST_REG_CNTL1     0x0A  /* 0x01 = 单次测量（该芯片无连续模式） */
#define IST_REG_CNTL2     0x0B  /* bit0 = 软复位 */
#define IST_REG_AVGCNTL   0x41  /* 采样平均 */
#define IST_REG_PDCNTL    0x42  /* 脉宽控制（手册推荐 0xC0） */
#define IST_WHOAMI_VAL    0x10
#define IST_LSB_PER_GAUSS 330.0f /* 0.3 µT/LSB 手册标称 —— 若前端模长明显
                                  * 偏离地磁 0.3~0.7G 区间需修正此值 */

/* ---- HMC5883L 寄存器 ---- */
#define HMC_REG_CONFIG_A  0x00
#define HMC_REG_CONFIG_B  0x01
#define HMC_REG_MODE      0x02
#define HMC_REG_DATA      0x03  /* 顺序 X,Z,Y（不是 XYZ！），大端 s16 */
#define HMC_REG_ID_A      0x0A  /* 'H' '4' '3' */
#define HMC_CONFIG_A_VAL  0x78  /* 8 次平均, 75Hz, 正常测量 */
#define HMC_CONFIG_B_VAL  0x20  /* 增益 ±1.3Ga */
#define HMC_LSB_PER_GAUSS 1090.0f

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(g_dev, buf, 2, 10);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(g_dev, &reg, 1, buf, len, 10);
}

static int attach_device(uint8_t addr)
{
    if (i2c_master_probe(g_i2c0_bus, addr, 50) != ESP_OK) return -1;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 400000,
    };
    return (i2c_master_bus_add_device(g_i2c0_bus, &cfg, &g_dev) == ESP_OK) ? 0 : -1;
}

static int try_qmc5883l(void)
{
    if (attach_device(QMC5883L_ADDR) != 0) return -1;

    uint8_t id = 0;
    if (read_regs(QMC_REG_CHIP_ID, &id, 1) != ESP_OK || id != 0xFF) {
        ESP_LOGW(TAG, "0x0D 应答但 chip id=0x%02X (期望 0xFF)", id);
        i2c_master_bus_rm_device(g_dev);
        return -1;
    }
    /* 手册要求：SET/RESET 周期寄存器必须写 0x01 */
    if (write_reg(QMC_REG_SET_RESET, 0x01) != ESP_OK) return -1;
    if (write_reg(QMC_REG_CTRL1, QMC_CTRL1_CONFIG) != ESP_OK) return -1;
    g_chip = MAG_CHIP_QMC5883L;
    return 0;
}

static int try_hmc5883l(void)
{
    if (attach_device(HMC5883L_ADDR) != 0) return -1;

    uint8_t id[3] = {0};
    if (read_regs(HMC_REG_ID_A, id, 3) != ESP_OK ||
        id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        ESP_LOGW(TAG, "0x1E 应答但 id=%02X %02X %02X (期望 'H43')", id[0], id[1], id[2]);
        i2c_master_bus_rm_device(g_dev);
        return -1;
    }
    if (write_reg(HMC_REG_CONFIG_A, HMC_CONFIG_A_VAL) != ESP_OK) return -1;
    if (write_reg(HMC_REG_CONFIG_B, HMC_CONFIG_B_VAL) != ESP_OK) return -1;
    if (write_reg(HMC_REG_MODE, 0x00) != ESP_OK) return -1;   /* 连续测量 */
    g_chip = MAG_CHIP_HMC5883L;
    return 0;
}

static int try_ist8310(void)
{
    if (attach_device(IST8310_ADDR) != 0) return -1;

    uint8_t id = 0;
    if (read_regs(IST_REG_WHOAMI, &id, 1) != ESP_OK || id != IST_WHOAMI_VAL) {
        ESP_LOGW(TAG, "0x0E 应答但 whoami=0x%02X (期望 0x10)", id);
        i2c_master_bus_rm_device(g_dev);
        return -1;
    }
    write_reg(IST_REG_CNTL2, 0x01);              /* 软复位 */
    vTaskDelay(pdMS_TO_TICKS(10));
    write_reg(IST_REG_AVGCNTL, 0x24);            /* XYZ 16 次平均 */
    write_reg(IST_REG_PDCNTL, 0xC0);
    write_reg(IST_REG_CNTL1, 0x01);              /* 触发首次单次测量 */
    g_chip = MAG_CHIP_IST8310;
    return 0;
}

/* 全总线扫描：三个已知地址都失败时的排查工具。
 * 只有 0x29/0x68 → 罗盘没上电或没进总线（接线/焊点问题）；
 * 出现其他地址 → 又一种未知罗盘变体，把地址报给开发者加驱动分支。 */
static void scan_bus(void)
{
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_master_probe(g_i2c0_bus, a, 10) == ESP_OK) {
            const char *hint = (a == 0x68) ? " (MPU6050)"
                             : (a == 0x29) ? " (TOF400F)"
                             : " (未知设备 — 疑似罗盘变体)";
            ESP_LOGW(TAG, "  I2C 应答: 0x%02X%s", a, hint);
            found++;
        }
    }
    ESP_LOGW(TAG, "总线扫描完成: %d 个设备。仅 0x29/0x68 = 罗盘未上电/未接入", found);
}

int bn880_mag_init(void)
{
    if (try_qmc5883l() == 0) {
        ESP_LOGI(TAG, "init ok: QMC5883L @0x0D, ±2G, 50Hz 连续");
        return 0;
    }
    if (try_hmc5883l() == 0) {
        ESP_LOGI(TAG, "init ok: HMC5883L @0x1E, ±1.3Ga, 75Hz 连续");
        return 0;
    }
    if (try_ist8310() == 0) {
        ESP_LOGI(TAG, "init ok: IST8310 @0x0E, 单次测量模式（逐读触发）");
        return 0;
    }
    ESP_LOGW(TAG, "0x0D/0x1E/0x0E 均未探测到磁力计，开始总线扫描:");
    scan_bus();
    g_chip = MAG_CHIP_NONE;
    return -1;
}

int bn880_mag_read(bn880_mag_data_t *out)
{
    if (!out) return -1;
    out->valid = false;

    if (g_chip == MAG_CHIP_QMC5883L) {
        uint8_t buf[6];
        if (read_regs(QMC_REG_DATA, buf, 6) != ESP_OK) return -1;
        int16_t x = (int16_t)((buf[1] << 8) | buf[0]);   /* 小端 */
        int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
        out->x = (float)x / QMC_LSB_PER_GAUSS;
        out->y = (float)y / QMC_LSB_PER_GAUSS;
        out->z = (float)z / QMC_LSB_PER_GAUSS;
        out->valid = true;
        return 0;
    }
    if (g_chip == MAG_CHIP_HMC5883L) {
        uint8_t buf[6];
        if (read_regs(HMC_REG_DATA, buf, 6) != ESP_OK) return -1;
        /* HMC 数据顺序是 X, Z, Y（大端）—— 这是与 QMC 最容易搞混的坑 */
        int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t z = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t y = (int16_t)((buf[4] << 8) | buf[5]);
        out->x = (float)x / HMC_LSB_PER_GAUSS;
        out->y = (float)y / HMC_LSB_PER_GAUSS;
        out->z = (float)z / HMC_LSB_PER_GAUSS;
        out->valid = true;
        return 0;
    }
    if (g_chip == MAG_CHIP_IST8310) {
        /* 单次测量模式：读上一次结果，再触发下一次。20Hz 读取节奏下数据
         * 最多 50ms 旧 —— 显示/融合都够用 */
        uint8_t buf[6];
        if (read_regs(IST_REG_DATA, buf, 6) != ESP_OK) return -1;
        int16_t x = (int16_t)((buf[1] << 8) | buf[0]);   /* 小端 */
        int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
        write_reg(IST_REG_CNTL1, 0x01);                  /* 触发下一次 */
        out->x = (float)x / IST_LSB_PER_GAUSS;
        out->y = (float)y / IST_LSB_PER_GAUSS;
        out->z = (float)z / IST_LSB_PER_GAUSS;
        out->valid = true;
        return 0;
    }
    return -1;
}

const char *bn880_mag_chip_name(void)
{
    switch (g_chip) {
    case MAG_CHIP_QMC5883L: return "QMC5883L";
    case MAG_CHIP_HMC5883L: return "HMC5883L";
    case MAG_CHIP_IST8310:  return "IST8310";
    default:                return "none";
    }
}
