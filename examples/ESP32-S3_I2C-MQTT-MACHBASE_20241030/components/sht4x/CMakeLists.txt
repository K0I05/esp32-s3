idf_component_register(
    SRCS sht4x.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common driver log
)
