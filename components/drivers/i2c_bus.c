#include "i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

i2c_master_bus_handle_t g_i2c0_bus = NULL;

int i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port    = I2C_NUM_0,
        .sda_io_num  = I2C0_SDA,
        .scl_io_num  = I2C0_SCL,
        .clk_source  = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&cfg, &g_i2c0_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(ret));
        return -1;
    }
    ESP_LOGI(TAG, "I2C0 init ok: SDA=%d SCL=%d", I2C0_SDA, I2C0_SCL);
    return 0;
}
