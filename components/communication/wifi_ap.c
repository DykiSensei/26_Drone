#include "wifi_ap.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "wifi_ap";

#define WIFI_AP_SSID_PREFIX  "Drone-"
#define WIFI_AP_PASSWORD     "12345678"
#define WIFI_AP_CHANNEL      1
#define WIFI_MAX_STA_CONN    2

int wifi_ap_init(void)
{
    /* Init NVS (required by WiFi) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Init TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    /* WiFi init */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Get MAC to build unique SSID */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ssid[32];
    snprintf(ssid, sizeof(ssid), WIFI_AP_SSID_PREFIX "%02X%02X",
             mac[4], mac[5]);

    /* Configure AP */
    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid_len = strlen(ssid),
            .channel  = WIFI_AP_CHANNEL,
            .max_connection = WIFI_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strcpy((char *)wifi_cfg.ap.ssid, ssid);
    strcpy((char *)wifi_cfg.ap.password, WIFI_AP_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP started: SSID=%s  Password=%s  IP=192.168.4.1",
             ssid, WIFI_AP_PASSWORD);
    return 0;
}
