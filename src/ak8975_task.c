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
 * @file ak8975_task.c
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

#include <ak8975_task.h>
#include <ak8975.h>


void i2c0_ak8975_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_ak8975_config_t dev_cfg         = I2C_AK8975_CONFIG_DEFAULT;
    i2c_ak8975_handle_t dev_hdl;
    //
    // init device
    i2c_ak8975_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ak8975 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AK8975 - START #########################");
        //
        // handle sensor
        i2c_ak8975_magnetic_axes_data_t magnetic_axes;
        esp_err_t result = i2c_ak8975_get_magnetic_axes(dev_hdl, &magnetic_axes);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ak8975 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "Compass X-Axis:  %f mG", magnetic_axes.x_axis);
            ESP_LOGI(APP_TAG, "Compass Y-Axis:  %f mG", magnetic_axes.y_axis);
            ESP_LOGI(APP_TAG, "Compass Z-Axis:  %f mG", magnetic_axes.z_axis);
            ESP_LOGI(APP_TAG, "Compass Heading: %f °", i2c_ak8975_convert_to_heading(magnetic_axes));
            //ESP_LOGI(APP_TAG, "True Heading:    %f °", i2c_mmc56x3_convert_to_true_heading(dev_hdl->declination, magnetic_axes));
        }
        //
        ESP_LOGI(APP_TAG, "######################## AK8975 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_ak8975_delete( dev_hdl );
    vTaskDelete( NULL );
}