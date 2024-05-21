idf_component_register(
    SRCS sht4x.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c esp_common driver log
)
