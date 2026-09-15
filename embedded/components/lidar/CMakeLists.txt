idf_component_register(
    SRCS "lidar_protocal.c" "lidar.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_uart esp_timer telemetry esp_driver_gpio log
)