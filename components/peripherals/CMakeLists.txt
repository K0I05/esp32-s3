set (srcs
    "esp_i2c/esp_driver_i2c_ext/i2c_master_ext.c"
    "esp_i2c/esp_bmp280/bmp280.c"
    "esp_i2c/esp_ssd1306/ssd1306.c"
)

set (include_dir
    "peripherals/esp_i2c/esp_driver_i2c_ext"
    "peripherals/esp_i2c/esp_bmp280"
    "peripherals/esp_i2c/esp_ssd1306"
)

#add_prefix(srcs "${CMAKE_CURRENT_LIST_DIR}/" ${srcs})

#message(STATUS "DEBUG: Use esp_peripherals component folder: ${CMAKE_CURRENT_LIST_DIR}.")

#idf_component_register(
#    SRCS "${srcs}" 
#    INCLUDE_DIRS "${include_dir}"
#    REQUIRES log
#)
