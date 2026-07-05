idf_component_register(SRCS "p4link.c" "wifi_ap.c" "http_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi esp_http_server esp_netif nvs_flash esp_timer)
