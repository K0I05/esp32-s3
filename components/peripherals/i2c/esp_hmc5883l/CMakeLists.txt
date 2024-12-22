idf_component_register(
    SRCS hmc5883l.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext driver log esp_common esp_timer
)
