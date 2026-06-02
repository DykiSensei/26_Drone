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

void http_server_set_command_cb(ws_command_cb_t cb)
{
    g_cmd_cb = cb;
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
        /* New WS connection — store fd */
        if (g_ws_count < WS_MAX_CLIENTS) {
            g_ws_fds[g_ws_count++] = httpd_req_to_sockfd(req);
            ESP_LOGI(TAG, "WS client connected (fd=%d, total=%d)",
                     httpd_req_to_sockfd(req), g_ws_count);
        }
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

void http_server_broadcast(const char *json_str)
{
    if (!g_server || !json_str) return;
    if (g_ws_count == 0) return;

    httpd_ws_frame_t ws_pkt = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json_str,
        .len     = strlen(json_str),
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
}
