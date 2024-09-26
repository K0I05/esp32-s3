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
 * @file time_into_interval.c
 *
 * ESP-IDF FreeRTOS task extension
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "time_into_interval.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "time_into_interval";


esp_err_t time_into_interval_new(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, time_into_interval_handle_t *time_into_interval_handle) {
    esp_err_t                   ret = ESP_OK;
    time_into_interval_handle_t out_handle;
    
    /* validate arguments */
    ESP_ARG_CHECK( interval_period );

    /* validate memory availability for time into interval handle */
    out_handle = (time_into_interval_handle_t)calloc(1, sizeof(time_into_interval_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for time into interval handle, new time into interval failed");

    /* validate memory availability for time into interval handle parameters */
    out_handle->params = (time_into_interval_params_t*)calloc(1, sizeof(time_into_interval_params_t));
    ESP_GOTO_ON_FALSE(out_handle->params, ESP_ERR_NO_MEM, err, TAG, "no memory for time into interval handle configuration parameters, new time into interval failed");

    /* initialize task schedule state object parameters */
    out_handle->params->epoch_time      = 0;
    out_handle->params->interval_type   = interval_type;
    out_handle->params->interval_period = interval_period;
    out_handle->params->interval_offset = interval_offset;

    /* set epoch time of the next scheduled task */
    datalogger_set_epoch_time_event(out_handle->params->interval_type, out_handle->params->interval_period, &out_handle->params->epoch_time);

    /* set output handle */
    *time_into_interval_handle = out_handle;

    return ESP_OK;

    err:
        free(out_handle);
        return ret;
}

bool time_into_interval(time_into_interval_handle_t time_into_interval_handle) {
    bool            status = false;
    struct timeval  tv_now;
    uint64_t        now_msec;
    uint64_t        next_msec;
    int64_t         delta_msec;

    /* validate arguments */
    if(!(time_into_interval_handle)) {
        return status;
    }

    // get system time
    gettimeofday(&tv_now, NULL);

    // convert system time to msec
    now_msec = (uint64_t)tv_now.tv_sec * 1000U + (uint64_t)tv_now.tv_usec / 1000U;

    // convert next scan event epoch time to msec
    next_msec = (uint64_t)time_into_interval_handle->params->epoch_time * 1000U;

    // compute time delta until next time into interval condition
    delta_msec = next_msec - now_msec;

    if(delta_msec <= 0) {

        status = true;

        datalogger_set_epoch_time_event(time_into_interval_handle->params->interval_type, time_into_interval_handle->params->interval_period, &time_into_interval_handle->params->epoch_time);
    }
    
    return status;
}

esp_err_t time_into_interval_del(time_into_interval_handle_t time_into_interval_handle) {
    /* free resource */
    free(time_into_interval_handle);

    return ESP_OK;
}