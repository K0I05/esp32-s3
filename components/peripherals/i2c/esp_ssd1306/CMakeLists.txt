idf_component_register(
    SRCS ssd1306.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common driver log
)
