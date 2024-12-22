idf_component_register(
    SRCS ak8975.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common esp_timer driver log
)
