idf_component_register(
    SRCS ccs811.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common esp_event esp_timer driver log
)
