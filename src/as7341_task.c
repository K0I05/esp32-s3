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
 * @file as7341_task.c
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

#include <as7341_task.h>
#include <as7341.h>


void i2c0_as7341_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_as7341_config_t dev_cfg          = I2C_AS7341_CONFIG_DEFAULT;
    i2c_as7341_handle_t dev_hdl;
    bool                flicker_completed = false;
    uint8_t             flicker_cycles = 0;
    //
    // init device
    i2c_as7341_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "as7341 handle init failed");
        assert(dev_hdl);
    }
    //
    //
    ESP_LOGI(APP_TAG, "Enable Register:  0x%02x (0b%s)", dev_hdl->enable_reg.reg, uint8_to_binary(dev_hdl->enable_reg.reg));
    ESP_LOGI(APP_TAG, "Aux ID Register:  0x%02x (0b%s)", dev_hdl->aux_id_reg.reg, uint8_to_binary(dev_hdl->aux_id_reg.reg));
    ESP_LOGI(APP_TAG, "Rev ID Register:  0x%02x (0b%s)", dev_hdl->revision_id_reg.reg, uint8_to_binary(dev_hdl->revision_id_reg.reg));
    ESP_LOGI(APP_TAG, "Part ID Register: 0x%02x (0b%s)", dev_hdl->part_id_reg.reg, uint8_to_binary(dev_hdl->part_id_reg.reg));

    //i2c_as7341_disable_led(dev_hdl);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AS7341 - START #########################");
        //
        if(flicker_completed == true) {
            // handle sensor
            i2c_as7341_channels_spectral_data_t adc_data;
            esp_err_t result = i2c_as7341_get_spectral_measurements(dev_hdl, &adc_data);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "spectral measurement failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGW(APP_TAG, "F1    %d", adc_data.f1);
                ESP_LOGW(APP_TAG, "F2    %d", adc_data.f2);
                ESP_LOGW(APP_TAG, "F3    %d", adc_data.f3);
                ESP_LOGW(APP_TAG, "F4    %d", adc_data.f4);
                ESP_LOGW(APP_TAG, "F5    %d", adc_data.f5);
                ESP_LOGW(APP_TAG, "F6    %d", adc_data.f6);
                ESP_LOGW(APP_TAG, "F7    %d", adc_data.f7);
                ESP_LOGW(APP_TAG, "F8    %d", adc_data.f8);
                ESP_LOGW(APP_TAG, "NIR   %d", adc_data.nir);
                ESP_LOGW(APP_TAG, "CLEAR %d", adc_data.clear);
            }

            i2c_as7341_channels_basic_counts_data_t basic_counts_data;
            result = i2c_as7341_get_basic_counts(dev_hdl, adc_data, &basic_counts_data);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "basic counts conversion failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGW(APP_TAG, "F1    %f", basic_counts_data.f1);
                ESP_LOGW(APP_TAG, "F2    %f", basic_counts_data.f2);
                ESP_LOGW(APP_TAG, "F3    %f", basic_counts_data.f3);
                ESP_LOGW(APP_TAG, "F4    %f", basic_counts_data.f4);
                ESP_LOGW(APP_TAG, "F5    %f", basic_counts_data.f5);
                ESP_LOGW(APP_TAG, "F6    %f", basic_counts_data.f6);
                ESP_LOGW(APP_TAG, "F7    %f", basic_counts_data.f7);
                ESP_LOGW(APP_TAG, "F8    %f", basic_counts_data.f8);
                ESP_LOGW(APP_TAG, "NIR   %f", basic_counts_data.nir);
                ESP_LOGW(APP_TAG, "CLEAR %f", basic_counts_data.clear);
            }
        } else {
            if(flicker_cycles < 5) {
                i2c_as7341_flicker_detection_states_t flicker_state;
                esp_err_t result = i2c_as7341_get_flicker_detection_status(dev_hdl, &flicker_state);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "flicker detection failed (%s)", esp_err_to_name(result));
                } else {
                    switch(flicker_state) {
                        case I2C_AS7341_FLICKER_DETECTION_INVALID:
                            ESP_LOGW(APP_TAG, "Flicker Detection: Invalid");
                            break;
                        case I2C_AS7341_FLICKER_DETECTION_UNKNOWN:
                            ESP_LOGW(APP_TAG, "Flicker Detection: Unknown");
                            break;
                        case I2C_AS7341_FLICKER_DETECTION_SATURATED:
                            ESP_LOGW(APP_TAG, "Flicker Detection: Saturated");
                            break;
                        case I2C_AS7341_FLICKER_DETECTION_100HZ:
                            ESP_LOGW(APP_TAG, "Flicker Detection: 100 Hz");
                            break;
                        case I2C_AS7341_FLICKER_DETECTION_120HZ:
                            ESP_LOGW(APP_TAG, "Flicker Detection: 120 Hz");
                            break;
                    }
                }
                i2c_as7341_clear_flicker_detection_status_register(dev_hdl);
                ++flicker_cycles;
            } else {
                flicker_completed = true;
            }
        }
        //
        ESP_LOGI(APP_TAG, "######################## AS7341 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_as7341_delete( dev_hdl );
    vTaskDelete( NULL );
}