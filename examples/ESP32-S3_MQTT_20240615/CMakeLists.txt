cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-S3_MQTT_20240615)

#target_add_binary_data(ESP32-S3_MQTT_20240615.elf "src/mqtt_eclipseprojects_io.pem" TEXT)

#target_add_binary_data(ESP32-S3_MQTT_20240615.elf "src/mqtt_broker_emqx_io.pem" TEXT)

#target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "src/mqtt_eclipseprojects_io.pem" TEXT)
#target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "src/mqtt_broker_emqx_io.pem" TEXT)