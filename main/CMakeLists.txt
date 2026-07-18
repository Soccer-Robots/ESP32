idf_component_register(
    SRCS
        "main.c"
        "wifi_connectivity.c"
        "goalpost_mechanism.c"

    INCLUDE_DIRS
        "../include"

    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        lwip
        esp_driver_gpio
        esp_driver_ledc
        esp_driver_gptimer
        spi_flash
)