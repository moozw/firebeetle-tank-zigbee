idf_component_register(
    SRCS "main.c" "lps2x.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer app_update esp_wifi esp_netif esp_event esp_http_server mqtt esp_https_ota esp_http_client espressif__esp-zigbee-lib
)
