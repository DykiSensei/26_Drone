idf_component_register(SRCS "wifi_ap.c" "http_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi esp_http_server esp_netif nvs_flash)
