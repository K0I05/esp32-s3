idf_component_register(
    SRCS bmp390.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext log esp_common esp_timer
)
