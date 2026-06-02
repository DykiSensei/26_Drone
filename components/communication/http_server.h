#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback type for received WS commands: void fn(const char *json, int len) */
typedef void (*ws_command_cb_t)(const char *json, int len);

/**
 * @brief 注册 WS 命令回调
 */
void http_server_set_command_cb(ws_command_cb_t cb);

/**
 * @brief 启动 HTTP + WebSocket 服务器 (port 80)
 * @return 0 成功, -1 失败
 */
int http_server_init(void);

/**
 * @brief 向所有已连接的 WebSocket 客户端广播遥测 JSON
 */
void http_server_broadcast(const char *json_str);

#ifdef __cplusplus
}
#endif
