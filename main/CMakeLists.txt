idf_component_register(
    SRCS
        "main.c" "wifi_connectivity.c" "goalpost_mechanism.c"
    INCLUDE_DIRS
        "../include"
    REQUIRES
        esp_wifi
        esp_event
        nvs_flash
        driver
        spi_flash
    PRIV_REQUIRES
        esp_driver_gpio
        esp_driver_ledc
)