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
 * @file tlv493d_task.c
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

#include <tlv493d_task.h>
#include <tlv493d.h>


void i2c0_tlv493d_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t           last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_tlv493d_config_t dev_cfg          = I2C_TLV493D_CONFIG_DEFAULT;
    i2c_tlv493d_handle_t dev_hdl;
    //
    // init device
    i2c_tlv493d_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "tlv493d handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## TLV493D - START #########################");
        //
        // handle sensor
        i2c_tlv493d_data_t data;
        esp_err_t result = i2c_tlv493d_get_data(dev_hdl, &data);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "tlv493d device read failed (%s)", esp_err_to_name(result));
        } else {
            //ESP_LOGI(APP_TAG, "air temperature:     %.2f C", temperature);
            //ESP_LOGI(APP_TAG, "relative humidity:   %.2f C", humidity);
        }
        //
        ESP_LOGI(APP_TAG, "######################## TLV493D - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_tlv493d_del( dev_hdl );
    vTaskDelete( NULL );
}