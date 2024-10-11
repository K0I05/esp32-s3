idf_component_register(
    SRCS veml7700.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common driver log
)
