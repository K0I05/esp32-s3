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
 * @file ens160_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <ens160_task.h>
#include <ens160.h>


void i2c0_ens160_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_ens160_config_t dev_cfg          = I2C_ENS160_CONFIG_DEFAULT;
    i2c_ens160_handle_t dev_hdl;
    //
    // init device
    i2c_ens160_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ens160 handle init failed");
        assert(dev_hdl);
    }
    //
    uint16_t startup_time = 0; // seconds
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## ENS160 - START #########################");
        //
        // handle sensor
        i2c_ens160_validity_flags_t dev_flag;
        if(i2c_ens160_get_validity_status(dev_hdl, &dev_flag) == ESP_OK) {
            // validate device status
            if(dev_flag == I2C_ENS160_VALFLAG_NORMAL) {
                i2c_ens160_air_quality_data_t aq_data;
                esp_err_t result = i2c_ens160_get_measurement(dev_hdl, &aq_data);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "ens160 device read failed (%s)", esp_err_to_name(result));
                } else {
                    i2c_ens160_aqi_uba_row_t uba_aqi = i2c_ens160_aqi_index_to_definition(aq_data.uba_aqi);

                    ESP_LOGW(APP_TAG, "index    %1x (%s)", aq_data.uba_aqi, uba_aqi.rating);
                    ESP_LOGW(APP_TAG, "tvco     %d (0x%04x)", aq_data.tvoc, aq_data.tvoc);
                    ESP_LOGW(APP_TAG, "eco2     %d (0x%04x)", aq_data.eco2, aq_data.eco2);
                }
                //
                i2c_ens160_air_quality_raw_data_t aq_raw_data;
                result = i2c_ens160_get_raw_measurement(dev_hdl, &aq_raw_data);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "ens160 device read failed (%s)", esp_err_to_name(result));
                } else {
                    ESP_LOGW(APP_TAG, "ri-res 0 %lu", aq_raw_data.hp0_rs);
                    ESP_LOGW(APP_TAG, "ri-res 1 %lu", aq_raw_data.hp1_rs);
                    ESP_LOGW(APP_TAG, "ri-res 2 %lu", aq_raw_data.hp2_rs);
                    ESP_LOGW(APP_TAG, "ri-res 3 %lu", aq_raw_data.hp3_rs);

                    ESP_LOGW(APP_TAG, "bl-res 0 %lu", aq_raw_data.hp0_bl);
                    ESP_LOGW(APP_TAG, "bl-res 1 %lu", aq_raw_data.hp1_bl);
                    ESP_LOGW(APP_TAG, "bl-res 2 %lu", aq_raw_data.hp2_bl);
                    ESP_LOGW(APP_TAG, "bl-res 3 %lu", aq_raw_data.hp3_bl);
                }
            } else if(dev_flag == I2C_ENS160_VALFLAG_WARMUP) {
                ESP_LOGW(APP_TAG, "ens160 device is warming up (180-sec wait [%u-sec])", startup_time);
                startup_time = startup_time + I2C0_TASK_SAMPLING_RATE;
            } else if(dev_flag == I2C_ENS160_VALFLAG_INITIAL_STARTUP) {
                ESP_LOGW(APP_TAG, "ens160 device is undrgoing initial starting up (3600-sec wait [%u-sec])", startup_time);
                startup_time = startup_time + I2C0_TASK_SAMPLING_RATE;
            } else if(dev_flag == I2C_ENS160_VALFLAG_INVALID_OUTPUT) {
                ESP_LOGW(APP_TAG, "ens160 device signals are giving unexpected values");
            }
        }
        //
        ESP_LOGI(APP_TAG, "######################## ENS160 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_ens160_delete( dev_hdl );
    vTaskDelete( NULL );
}