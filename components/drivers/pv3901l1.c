#include "pv3901l1.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "pv3901l1";

/* ---- 内部共享数据 ---- */
static pv3901l1_data_t g_flow;
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- UART 接收状态机 ---- */
static uint8_t  rx_buf[16];
static uint8_t  data_cnt;
static uint8_t  state;

static void parse_byte(uint8_t byte)
{
    switch (state) {
    case 0: /* 等待帧头1: 0xFE */
        if (byte == 0xFE) {
            rx_buf[data_cnt++] = byte;
            state = 1;
        }
        break;

    case 1: /* 等待帧头2: 0x04 */
        if (byte == 0x04) {
            rx_buf[data_cnt++] = byte;
            state = 2;
        } else {
            state = 0;
            data_cnt = 0;
        }
        break;

    case 2: /* 接收数据 */
        rx_buf[data_cnt++] = byte;
        if (data_cnt >= 9) {
            /* 协议（手册）：
             *   [0]=0xFE  [1]=0x04  [2..5]=flow_x/y (小端 s16)
             *   [6]=和校验  [7]=qual  [8]=结束符(0xAA YAW高/0xBB YAW低)
             * 双重校验：sum + 结束符。结束符错位通常意味着串口丢字节
             * 或半个包，仅靠 sum 容易把错位数据当成有效帧 → flow_x_i 跳变。 */
            uint8_t sum = rx_buf[2] + rx_buf[3] + rx_buf[4] + rx_buf[5];
            bool sum_ok  = (sum == rx_buf[6]);
            bool tail_ok = (rx_buf[8] == 0xAA || rx_buf[8] == 0xBB);
            if (sum_ok && tail_ok) {
                /* 模块原始轴 → 机体系映射（2026-07-04 实测本机安装转了 90°：
                 * 前移→raw_y 正、左移→raw_x 正）。机体系约定：flow_x=前正、
                 * flow_y=右正 —— 下游陀螺补偿配对/米制换算/控制符号全部依赖
                 * 此约定。换装或旋转模块后只改这两行，并重验陀螺补偿符号。 */
                int16_t raw_x = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
                int16_t raw_y = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
                portENTER_CRITICAL(&g_lock);
                g_flow.flow_x = raw_y;              /* 机体前正 = 模块 +y */
                g_flow.flow_y = (int16_t)(-raw_x);  /* 机体右正 = 模块 −x */
                g_flow.qual   = rx_buf[7];
                g_flow.flow_x_i += (float)g_flow.flow_x;
                g_flow.flow_y_i += (float)g_flow.flow_y;
                g_flow.frame_count++;
                g_flow.data_ready = true;
                portEXIT_CRITICAL(&g_lock);
            } else {
                portENTER_CRITICAL(&g_lock);
                g_flow.error_count++;
                portEXIT_CRITICAL(&g_lock);
            }
            state = 0;
            data_cnt = 0;
        }
        break;

    default:
        state = 0;
        data_cnt = 0;
        break;
    }
}

static void uart_rx_task(void *arg)
{
    uint8_t buf[PV3901L1_UART_BUF_SIZE];
    uint32_t raw_total = 0;
    while (1) {
        int len = uart_read_bytes(PV3901L1_UART_PORT, buf,
                                  sizeof(buf), pdMS_TO_TICKS(100));
        if (len > 0) {
            raw_total += len;
            for (int i = 0; i < len; i++) {
                parse_byte(buf[i]);
            }
        }
        /* 每5秒打印诊断 */
        static uint32_t last_tick;
        if (xTaskGetTickCount() - last_tick > pdMS_TO_TICKS(5000)) {
            last_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "raw_bytes=%lu frames=%lu errors=%lu",
                     raw_total, g_flow.frame_count, g_flow.error_count);
        }
    }
}

/* ---- 公开接口 ---- */

int pv3901l1_init(void)
{
    /* --- YAW_MODE GPIO --- */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PV3901L1_YAW_MODE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PV3901L1_YAW_MODE_PIN, 0);

    /* --- UART --- */
    uart_config_t uart_conf = {
        .baud_rate  = PV3901L1_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PV3901L1_UART_PORT,
                                         PV3901L1_UART_BUF_SIZE,  /* RX buf */
                                         0,                        /* TX buf unused */
                                         0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PV3901L1_UART_PORT, &uart_conf));
    ESP_ERROR_CHECK(uart_set_pin(PV3901L1_UART_PORT,
                                  UART_PIN_NO_CHANGE,             /* TX unused */
                                  PV3901L1_RX_PIN,                /* RX */
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* 清零初始状态 */
    memset(&g_flow, 0, sizeof(g_flow));
    data_cnt = 0;
    state    = 0;

    /* 启动接收任务（绑定到 Core 1，避免抢占 stabilizer） */
    xTaskCreatePinnedToCore(uart_rx_task, "flow_rx", 3072,
                            NULL, 10, NULL, 1);

    /* 等待一小段时间后检查是否有数据流入 */
    vTaskDelay(pdMS_TO_TICKS(500));
    size_t buffered = 0;
    uart_get_buffered_data_len(PV3901L1_UART_PORT, &buffered);
    ESP_LOGI(TAG, "init ok: RX=GPIO%d, YAW=GPIO%d, baud=%d, buffered=%d",
             PV3901L1_RX_PIN, PV3901L1_YAW_MODE_PIN, PV3901L1_BAUDRATE,
             (int)buffered);
    return 0;
}

int pv3901l1_get_data(pv3901l1_data_t *out)
{
    if (!out) return -1;
    portENTER_CRITICAL(&g_lock);
    bool ready = g_flow.data_ready;
    if (ready) {
        memcpy(out, &g_flow, sizeof(pv3901l1_data_t));
        g_flow.data_ready = false;
    }
    portEXIT_CRITICAL(&g_lock);
    return ready ? 0 : -1;
}

void pv3901l1_reset_integral(void)
{
    portENTER_CRITICAL(&g_lock);
    g_flow.flow_x_i = 0.0f;
    g_flow.flow_y_i = 0.0f;
    portEXIT_CRITICAL(&g_lock);
}

void pv3901l1_set_yaw_mode(flow_yaw_mode_t mode)
{
    gpio_set_level(PV3901L1_YAW_MODE_PIN, mode);
}
