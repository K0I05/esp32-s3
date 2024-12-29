cmake_minimum_required(VERSION 3.16.0)
list(APPEND EXTRA_COMPONENT_DIRS
                    "components/engineering/kalman_motion" 
                    "components/engineering/sensirion_gas_index_algorithm" 

                    "components/peripherals/adc/esp_s12sd"

                    "components/peripherals/i2c/esp_driver_i2c_ext"
                    "components/peripherals/i2c/esp_ahtxx"
                    "components/peripherals/i2c/esp_ak8975"
                    "components/peripherals/i2c/esp_as7341"
                    "components/peripherals/i2c/esp_bh1750"
                    "components/peripherals/i2c/esp_bmp280"
                    "components/peripherals/i2c/esp_bmp390"
                    "components/peripherals/i2c/esp_ccs811"
                    "components/peripherals/i2c/esp_ens160"
                    "components/peripherals/i2c/esp_hdc1080"
                    "components/peripherals/i2c/esp_hmc5883l"
                    "components/peripherals/i2c/esp_mlx90614"
                    "components/peripherals/i2c/esp_mmc56x3"
                    "components/peripherals/i2c/esp_mpu6050"
                    "components/peripherals/i2c/esp_sgp4x"
                    "components/peripherals/i2c/esp_sht4x"
                    "components/peripherals/i2c/esp_ssd1306"
                    "components/peripherals/i2c/esp_tlv493d"
                    "components/peripherals/i2c/esp_veml7700"

                    "components/peripherals/owb/onewire_bus"
                    "components/peripherals/owb/esp_ds18b20"

                    "components/peripherals/spi/esp_max31865"

                    "components/schedule/esp_time_into_interval"

                    "components/storage/esp_datalogger"
                    "components/storage/esp_nvs_ext"
                    )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-S3_ESP-IDF_COMPONENTS)
