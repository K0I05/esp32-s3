idf_component_register(
    SRCS bh1750.c
    INCLUDE_DIRS .
    REQUIRES esp_driver_i2c_ext driver log
)
