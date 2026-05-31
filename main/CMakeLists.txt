idf_component_register(
    SRCS "main.c" "lps2x.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_timer app_update espressif__esp-zigbee-lib
)
