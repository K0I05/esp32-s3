idf_component_register(
    SRCS hdc1080.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common driver log
)
