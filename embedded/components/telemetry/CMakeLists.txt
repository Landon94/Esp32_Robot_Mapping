idf_component_register(
    SRCS "telemetry.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_timer freertos udp
)