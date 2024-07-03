idf_component_register(
    SRCS ak8975.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c esp_common driver log
)
