idf_component_register(
    SRCS mmc56x3.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c esp_common esp_timer driver log
)
