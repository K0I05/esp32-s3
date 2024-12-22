idf_component_register(
    SRCS tlv493d.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext esp_common driver log esp_timer
)
