#include "http_server.h"
#include "web_page.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "httpd";

/* Max WebSocket clients */
#define WS_MAX_CLIENTS  4

static httpd_handle_t g_server = NULL;
static int g_ws_fds[WS_MAX_CLIENTS];
static int g_ws_count = 0;
static ws_command_cb_t g_cmd_cb = NULL;
static ws_disconnect_cb_t g_disconnect_cb = NULL;

void http_server_set_command_cb(ws_command_cb_t cb)
{
    g_cmd_cb = cb;
}

void http_server_set_disconnect_cb(ws_disconnect_cb_t cb)
{
    g_disconnect_cb = cb;
}

static void check_all_disconnected(void)
{
    if (g_ws_count == 0 && g_disconnect_cb) {
        g_disconnect_cb();
    }
}

/* ---- HTTP GET / handler ---- */
static esp_err_t root_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET / from fd=%d", httpd_req_to_sockfd(req));
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, WEB_PAGE_HTML, strlen(WEB_PAGE_HTML));
    return ESP_OK;
}

/* ---- WebSocket handler ---- */
static esp_err_t ws_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "ws_handler: method=%d uri=%s", req->method, req->uri);
    if (req->method == HTTP_GET) {
        /* New WS connection — store fd. 满员必须拒绝（返回错误关闭连接）：
         * 不入列表的客户端能发控制命令却不参与"全断开→DISARM"安全统计 */
        if (g_ws_count >= WS_MAX_CLIENTS) {
            ESP_LOGW(TAG, "WS client limit (%d) reached — rejecting fd=%d",
                     WS_MAX_CLIENTS, httpd_req_to_sockfd(req));
            return ESP_FAIL;
        }
        g_ws_fds[g_ws_count++] = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WS client connected (fd=%d, total=%d)",
                 httpd_req_to_sockfd(req), g_ws_count);
        return ESP_OK;
    }

    /* WS data frame received */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));

    /* Get frame info (len=0 means peek only) */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    /* Handle non-text frames */
    if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
        /* Respond with PONG (same payload) */
        uint8_t pong_buf[128] = {0};
        httpd_ws_frame_t pong = {
            .type = HTTPD_WS_TYPE_PONG,
            .payload = pong_buf,
            .len = 0,
        };
        if (ws_pkt.len > 0 && ws_pkt.len <= sizeof(pong_buf)) {
            ws_pkt.payload = pong_buf;
            httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            pong.len = ws_pkt.len;
        }
        httpd_ws_send_frame(req, &pong);
        return ESP_OK;
    }
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "WS close frame received");
        return ESP_OK;
    }
    if (ws_pkt.type != HTTPD_WS_TYPE_TEXT) {
        /* PONG or unknown — ignore */
        return ESP_OK;
    }
    if (ws_pkt.len == 0) return ESP_OK;
    if (ws_pkt.len > 256) ws_pkt.len = 256;

    /* Read TEXT payload */
    uint8_t buf[257] = {0};
    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) return ret;

    /* Forward JSON command to registered callback */
    if (g_cmd_cb) g_cmd_cb((const char *)buf, ws_pkt.len);
    return ESP_OK;
}

/* Called on WS disconnection (registered via on_close) */
static void ws_close_handler(httpd_handle_t hd, int sockfd)
{
    for (int i = 0; i < g_ws_count; i++) {
        if (g_ws_fds[i] == sockfd) {
            g_ws_fds[i] = g_ws_fds[--g_ws_count];
            ESP_LOGI(TAG, "WS client disconnected (fd=%d, total=%d)",
                     sockfd, g_ws_count);
            check_all_disconnected();
            return;
        }
    }
}

/* ---- Public API ---- */

int http_server_init(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_open_sockets = WS_MAX_CLIENTS + 2;
    cfg.close_fn = ws_close_handler;
    cfg.send_wait_timeout = 1;   /* 卡死的客户端最多拖住 httpd 任务 1s（默认 5s） */

    esp_err_t ret = httpd_start(&g_server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return -1;
    }

    /* GET / → web page */
    httpd_uri_t uri_root = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(g_server, &uri_root);

    /* WS /ws → WebSocket */
    httpd_uri_t uri_ws = {
        .uri      = "/ws",
        .method   = HTTP_GET,
        .handler  = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };
    httpd_register_uri_handler(g_server, &uri_ws);

    memset(g_ws_fds, 0, sizeof(g_ws_fds));
    g_ws_count = 0;

    ESP_LOGI(TAG, "HTTP+WS server started on port 80");
    return 0;
}

/* ---- 异步广播 ----
 * 发送必须在 httpd 任务里做，不能在主循环（调用方）上下文直接发：
 * httpd_ws_send_frame_async 实际是同步写 socket，WiFi 拥塞/客户端卡顿时
 * 会阻塞调用者 —— 控制循环停摆期间电机保持旧 PWM，等效失控。
 * 顺带消除 g_ws_fds 竞态：连接/断开/发送失败移除现在都发生在 httpd 任务。 */
static char          g_bcast_buf[768];
static volatile bool g_bcast_inflight = false;

static void bcast_work(void *arg)
{
    httpd_ws_frame_t ws_pkt = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)g_bcast_buf,
        .len     = strlen(g_bcast_buf),
    };

    int i = 0;
    while (i < g_ws_count) {
        esp_err_t ret = httpd_ws_send_frame_async(g_server, g_ws_fds[i], &ws_pkt);
        if (ret != ESP_OK) {
            /* Client likely disconnected — remove from list */
            g_ws_fds[i] = g_ws_fds[--g_ws_count];
        } else {
            i++;
        }
    }
    check_all_disconnected();
    g_bcast_inflight = false;
}

void http_server_broadcast(const char *json_str)
{
    if (!g_server || !json_str) return;
    if (g_ws_count == 0) return;
    if (g_bcast_inflight) return;   /* 上一帧还没发完 — 丢弃本帧，绝不等待 */

    size_t n = strlen(json_str);
    if (n >= sizeof(g_bcast_buf)) n = sizeof(g_bcast_buf) - 1;
    memcpy(g_bcast_buf, json_str, n);
    g_bcast_buf[n] = '\0';

    g_bcast_inflight = true;
    if (httpd_queue_work(g_server, bcast_work, NULL) != ESP_OK) {
        g_bcast_inflight = false;
    }
}
