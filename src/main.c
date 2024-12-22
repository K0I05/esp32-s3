/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file main.c
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE

 * @brief ESP-IDF consolidated component repository with basic examples for each component.  Components were developed
 * with an ESP32-S3 development board under Visual Studio Code and Platform IO and may not work with older development
 * board releases.
 * 
 * See `app_config.h` within the include folder to change default I2C master bus and I2C GPIO ping configuration settings.
 * 
 * See `[component-name]_task.h` within the include folder to change default configuration settings by component.
 *
 * 
 * 
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * C:\Users\lavco\.platformio\penv\Scripts\platformio.exe system prune
 * 
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>

#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* components */
#include <i2c_master_ext.h>
#include <nvs_ext.h>
/* component tasks */
#include <ahtxx_task.h>
#include <as7341_task.h>
#include <bh1750_task.h>
#include <bmp280_task.h>
#include <bmp390_task.h>
#include <ccs811_task.h>
#include <ens160_task.h>
#include <hdc1080_task.h>
#include <hmc5883l_task.h>
#include <mlx90614_task.h>
#include <mpu6050_task.h>
#include <sgp4x_task.h>
#include <sht4x_task.h>
#include <ssd1306_task.h>
#include <tlv493d_task.h>
#include <veml7700_task.h>

/**
 * @brief I2C component examples enumerator.
 */
typedef enum i2c_components_tag {
    I2C_COMPONENT_AHTXX,
    I2C_COMPONENT_AS7341,
    I2C_COMPONENT_BH1750,
    I2C_COMPONENT_BMP280,
    I2C_COMPONENT_BMP390,
    I2C_COMPONENT_CCS811,
    I2C_COMPONENT_ENS160,
    I2C_COMPONENT_HDC1080,
    I2C_COMPONENT_HMC5883L,
    I2C_COMPONENT_MLX90614,
    I2C_COMPONENT_MPU6050,
    I2C_COMPONENT_SGP4X,
    I2C_COMPONENT_SHT4X,
    I2C_COMPONENT_SSD1306,
    I2C_COMPONENT_TLV493D,
    I2C_COMPONENT_VEML7700,
} i2c_components_t;

// initialize master i2c 0 bus configuration
i2c_master_bus_config_t i2c0_bus_cfg = I2C0_MASTER_CONFIG_DEFAULT;
i2c_master_bus_handle_t i2c0_bus_hdl;
bool                    i2c0_component_tasked = false;

/**
 * @brief Creates a task pinned to the application core (1) by task function
 * and task name to run an I2C component example on I2C master bus (0).
 * 
 * @note Only one I2C component example can run on I2C master bus 0.
 * 
 * @param task Task function reference.
 * @param name Task reference name.
 */
static inline void i2c0_task_create(TaskFunction_t task, const char* name) {
    /*  
        note: only one task on the i2c master bus 0 can run at a time
     */

    /* validate i2c0 component flag */
    if(i2c0_component_tasked == true) {
        ESP_LOGE(APP_TAG, "An I2C component sample is already running on I2C master bus 0, failed to create i2c0 task");
        return;
    }
    /* create task pinned to the app core */
    xTaskCreatePinnedToCore( 
        task,
        name, 
        I2C0_TASK_STACK_SIZE, 
        NULL, 
        I2C0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    /* set i2c0 component flag */
    i2c0_component_tasked = true;
}

/**
 * @brief I2C example to run in the application by component name.
 * 
 * @note Only one I2C component example can run on I2C master bus 0.
 * 
 * @param component I2C component example to run in the application
 */
static inline void i2c0_component_example_start(const i2c_components_t component) {
    /*  
        device task functions use a common naming nomenclature
        for i2c and adc peripherals, as an example: i2c_0_[device]_task

        note: only one i2c 0 task can run at a time
     */
    switch(component) {
        case I2C_COMPONENT_AHTXX:
            i2c0_task_create(i2c0_ahtxx_task, I2C0_AHTXX_TASK_NAME);
            break;
        case I2C_COMPONENT_AS7341:
            i2c0_task_create(i2c0_as7341_task, I2C0_AS7341_TASK_NAME);
            break;
        case I2C_COMPONENT_BH1750:
            i2c0_task_create(i2c0_bh1750_task, I2C0_BH1750_TASK_NAME);
            break;
        case I2C_COMPONENT_BMP280:
            i2c0_task_create(i2c0_bmp280_task, I2C0_BMP280_TASK_NAME);
            break;
        case I2C_COMPONENT_BMP390:
            i2c0_task_create(i2c0_bmp390_task, I2C0_BMP390_TASK_NAME);
            break;
        case I2C_COMPONENT_CCS811:
            i2c0_task_create(i2c0_ccs811_task, I2C0_CCS811_TASK_NAME);
            break;
        case I2C_COMPONENT_ENS160:
            i2c0_task_create(i2c0_ens160_task, I2C0_ENS160_TASK_NAME);
            break;
        case I2C_COMPONENT_HDC1080:
            i2c0_task_create(i2c0_hdc1080_task, I2C0_HDC1080_TASK_NAME);
            break;
        case I2C_COMPONENT_HMC5883L:
            i2c0_task_create(i2c0_hmc5883l_task, I2C0_HMC5883L_TASK_NAME);
            break;
        case I2C_COMPONENT_MLX90614:
            i2c0_task_create(i2c0_mlx90614_task, I2C0_MLX90614_TASK_NAME);
            break;
        case I2C_COMPONENT_MPU6050:
            i2c0_task_create(i2c0_mpu6050_task, I2C0_MPU6050_TASK_NAME);
            break;
        case I2C_COMPONENT_SGP4X:
            i2c0_task_create(i2c0_sgp4x_task, I2C0_SGP4X_TASK_NAME);
            break;
        case I2C_COMPONENT_SHT4X:
            i2c0_task_create(i2c0_sht4x_task, I2C0_SHT4X_TASK_NAME);
            break;
        case I2C_COMPONENT_SSD1306:
            i2c0_task_create(i2c0_ssd1306_task, I2C0_SSD1306_TASK_NAME);
            break;
        case I2C_COMPONENT_TLV493D:
            i2c0_task_create(i2c0_tlv493d_task, I2C0_TLV493D_TASK_NAME);
            break;
        case I2C_COMPONENT_VEML7700:
            i2c0_task_create(i2c0_veml7700_task, I2C0_VEML7700_TASK_NAME);
            break;
    }
}

/**
 * @brief Scans I2C master bus 0 for i2c devices and prints each device address when detected.
 */
static inline esp_err_t i2c0_device_scan(void) {
    return i2c_master_bus_detect_devices(i2c0_bus_hdl);
}

/**
 * @brief Main application entry point.
 */
void app_main( void ) {
    ESP_LOGI(APP_TAG, "Startup..");
    ESP_LOGI(APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(APP_TAG, "IDF version: %s", esp_get_idf_version());

    /* set log levels */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(APP_TAG, ESP_LOG_VERBOSE);

    /* instantiate nvs storage */
    ESP_ERROR_CHECK( nvs_init() );

    /* instantiate i2c master bus 0 */
    ESP_ERROR_CHECK( i2c_new_master_bus(&i2c0_bus_cfg, &i2c0_bus_hdl) );

    /* scan i2c devices on i2c master bus 0 */
    ESP_LOGI(APP_TAG, "Scanning I2C master bus 0 for I2C devices..");
    ESP_ERROR_CHECK( i2c0_device_scan() );

    /* delay task before starting component example */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* start a component example */
    /* note: only one component can run at a time */
    
    //i2c0_component_example_start(I2C_COMPONENT_AHTXX);
    //i2c0_component_example_start(I2C_COMPONENT_AS7341);
    //i2c0_component_example_start(I2C_COMPONENT_BH1750);
    //i2c0_component_example_start(I2C_COMPONENT_BMP280);
    i2c0_component_example_start(I2C_COMPONENT_BMP390);
    //i2c0_component_example_start(I2C_COMPONENT_CCS811);
    //i2c0_component_example_start(I2C_COMPONENT_ENS160);
    //i2c0_component_example_start(I2C_COMPONENT_HDC1080);
    //i2c0_component_example_start(I2C_COMPONENT_HMC5883L);
    //i2c0_component_example_start(I2C_COMPONENT_MLX90614);
    //i2c0_component_example_start(I2C_COMPONENT_MPU6050);
    //i2c0_component_example_start(I2C_COMPONENT_SGP4X);
    //i2c0_component_example_start(I2C_COMPONENT_SHT4X);
    //i2c0_component_example_start(I2C_COMPONENT_SSD1306);
    //i2c0_component_example_start(I2C_COMPONENT_TLV493D);
    //i2c0_component_example_start(I2C_COMPONENT_VEML7700);
}