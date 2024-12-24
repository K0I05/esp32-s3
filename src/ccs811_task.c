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
 * @file ccs811_task.c
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

#include <ccs811_task.h>
#include <ccs811.h>


void i2c0_ccs811_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_ccs811_config_t dev_cfg          = I2C_CCS811_CONFIG_DEFAULT;
    i2c_ccs811_handle_t dev_hdl;
    //
    // init device
    i2c_ccs811_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ccs811 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## CCS811 - START #########################");
        //
        // handle sensor
        uint16_t eco2; uint16_t etvoc;
        esp_err_t result = i2c_ccs811_get_measurement(dev_hdl, &eco2, &etvoc);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "i2c0 i2c_ccs811_get_measurement failed: %s", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "eCO2  value: %d ppm", eco2);
            ESP_LOGI(APP_TAG, "eTVOC value: %d ppb", etvoc); 
        }
        //
        ESP_LOGI(APP_TAG, "######################## CCS811 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_ccs811_delete( dev_hdl );
    vTaskDelete( NULL );
}