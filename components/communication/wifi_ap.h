#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 WiFi AP 模式
 *        SSID: Drone-XXXX (MAC后4位), Password: 12345678
 *        IP: 192.168.4.1
 * @return 0 成功, -1 失败
 */
int wifi_ap_init(void);

#ifdef __cplusplus
}
#endif
