#include "p4link.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "p4link";

#define P4_UART      UART_NUM_2
#define P4_TX_GPIO   17
#define P4_RX_GPIO   18
#define P4_RX_BUF    256          /* UART 驱动接收环形缓冲 */
#define P4_ALIVE_US  ((int64_t)P4LINK_LINK_TIMEOUT_MS * 1000)

static bool g_initialized = false;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

/* rx 任务写、主循环读——都在 g_mux 临界区内 */
static p4link_target_t g_target;
static int64_t  g_target_rx_us = 0;
static bool     g_target_fresh = false;
static int64_t  g_last_valid_us = 0;
static uint32_t g_rx_ok = 0, g_crc_err = 0;

/* ── 解析状态机（仅 rx 任务访问）──
 * fbuf 累积 type+len+payload，CRC 一次算完（与协议头 p4link_crc16 覆盖一致） */
typedef enum { PS_MAGIC0, PS_MAGIC1, PS_TYPE, PS_LEN,
               PS_PAYLOAD, PS_CRC_LO, PS_CRC_HI } parse_state_t;

static parse_state_t g_ps = PS_MAGIC0;
static uint8_t g_fbuf[2 + P4LINK_MAX_PAYLOAD];   /* [0]=type [1]=len [2..]=payload */
static uint8_t g_plen = 0, g_pidx = 0, g_crc_lo = 0;

static void deliver_frame(void)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&g_mux);
    g_last_valid_us = now;
    g_rx_ok++;
    if (g_fbuf[0] == P4LINK_MSG_TARGET && g_plen == sizeof(p4link_target_t)) {
        memcpy(&g_target, &g_fbuf[2], sizeof(g_target));
        g_target_rx_us = now;
        g_target_fresh = true;
    }
    portEXIT_CRITICAL(&g_mux);
}

static void feed_byte(uint8_t b)
{
    switch (g_ps) {
    case PS_MAGIC0:
        if (b == P4LINK_MAGIC0) g_ps = PS_MAGIC1;
        break;
    case PS_MAGIC1:
        /* 0xAA 0xAA 0x55 也要能同步上：非 0x55 但是 0xAA 则停留本态 */
        g_ps = (b == P4LINK_MAGIC1) ? PS_TYPE
             : (b == P4LINK_MAGIC0) ? PS_MAGIC1 : PS_MAGIC0;
        break;
    case PS_TYPE:
        g_fbuf[0] = b;
        g_ps = PS_LEN;
        break;
    case PS_LEN:
        if (b > P4LINK_MAX_PAYLOAD) { g_ps = PS_MAGIC0; break; }
        g_fbuf[1] = b;
        g_plen = b;
        g_pidx = 0;
        g_ps = (b == 0) ? PS_CRC_LO : PS_PAYLOAD;
        break;
    case PS_PAYLOAD:
        g_fbuf[2 + g_pidx++] = b;
        if (g_pidx >= g_plen) g_ps = PS_CRC_LO;
        break;
    case PS_CRC_LO:
        g_crc_lo = b;
        g_ps = PS_CRC_HI;
        break;
    case PS_CRC_HI: {
        uint16_t rx_crc = (uint16_t)g_crc_lo | ((uint16_t)b << 8);
        if (p4link_crc16(g_fbuf, 2 + g_plen) == rx_crc) {
            deliver_frame();
        } else {
            portENTER_CRITICAL(&g_mux);
            g_crc_err++;
            portEXIT_CRITICAL(&g_mux);
        }
        g_ps = PS_MAGIC0;
        break; }
    default:
        g_ps = PS_MAGIC0;
        break;
    }
}

static void p4link_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    while (1) {
        int n = uart_read_bytes(P4_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) feed_byte(buf[i]);
    }
}

int p4link_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = P4LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    /* TX 环形缓冲 = 0：发送用 uart_tx_chars 直写 FIFO（非阻塞，见 send_state） */
    if (uart_driver_install(P4_UART, P4_RX_BUF, 0, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart driver install fail");
        return -1;
    }
    if (uart_param_config(P4_UART, &cfg) != ESP_OK ||
        uart_set_pin(P4_UART, P4_TX_GPIO, P4_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart config/pin fail");
        uart_driver_delete(P4_UART);
        return -1;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(p4link_rx_task, "p4link_rx",
                                            3072, NULL, 9, NULL, 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "rx task create fail");
        uart_driver_delete(P4_UART);
        return -1;
    }

    g_initialized = true;
    ESP_LOGI(TAG, "init ok: UART2 TX=GPIO%d RX=GPIO%d @%d (proto v%d)",
             P4_TX_GPIO, P4_RX_GPIO, P4LINK_BAUD, P4LINK_PROTO_VER);
    return 0;
}

void p4link_send_state(const p4link_state_t *st)
{
    if (!g_initialized || !st) return;
    uint8_t frame[P4LINK_FRAME_OVERHEAD + sizeof(*st)];
    frame[0] = P4LINK_MAGIC0;
    frame[1] = P4LINK_MAGIC1;
    frame[2] = P4LINK_MSG_STATE;
    frame[3] = (uint8_t)sizeof(*st);
    memcpy(&frame[4], st, sizeof(*st));
    uint16_t crc = p4link_crc16(&frame[2], 2 + sizeof(*st));
    frame[4 + sizeof(*st)] = (uint8_t)(crc & 0xFF);
    frame[5 + sizeof(*st)] = (uint8_t)(crc >> 8);
    /* 非阻塞：只写 FIFO 能装下的部分。24B@50Hz vs FIFO 128B@115200(排空 2ms)
     * 正常永远整帧；万一半帧，P4 侧 CRC 丢弃 + 重同步（协议 §4） */
    uart_tx_chars(P4_UART, (const char *)frame, sizeof(frame));
}

bool p4link_take_target(p4link_target_t *out, int64_t *rx_time_us)
{
    if (!g_initialized || !out) return false;
    bool fresh;
    portENTER_CRITICAL(&g_mux);
    fresh = g_target_fresh;
    if (fresh) {
        *out = g_target;
        if (rx_time_us) *rx_time_us = g_target_rx_us;
        g_target_fresh = false;
    }
    portEXIT_CRITICAL(&g_mux);
    return fresh;
}

bool p4link_peek_target(p4link_target_t *out, int64_t *rx_time_us)
{
    if (!g_initialized || !out) return false;
    bool ever;
    portENTER_CRITICAL(&g_mux);
    ever = (g_target_rx_us != 0);   /* rx 时刻恒 >0，用作"曾收到过"标志 */
    if (ever) {
        *out = g_target;
        if (rx_time_us) *rx_time_us = g_target_rx_us;
    }
    portEXIT_CRITICAL(&g_mux);
    return ever;
}

bool p4link_alive(void)
{
    if (!g_initialized) return false;
    int64_t last;
    portENTER_CRITICAL(&g_mux);
    last = g_last_valid_us;
    portEXIT_CRITICAL(&g_mux);
    return last != 0 && (esp_timer_get_time() - last) < P4_ALIVE_US;
}

void p4link_get_stats(uint32_t *rx_ok, uint32_t *crc_err)
{
    portENTER_CRITICAL(&g_mux);
    if (rx_ok)   *rx_ok   = g_rx_ok;
    if (crc_err) *crc_err = g_crc_err;
    portEXIT_CRITICAL(&g_mux);
}
