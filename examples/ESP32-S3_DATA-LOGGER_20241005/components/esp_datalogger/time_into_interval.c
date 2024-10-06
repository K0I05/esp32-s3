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


esp_err_t time_into_interval_init(const time_into_interval_config_t *time_into_interval_config, 
                                 time_into_interval_handle_t *time_into_interval_handle) {
    esp_err_t                   ret = ESP_OK;
    time_into_interval_handle_t out_handle;
    
    /* validate task-schedule arguments */
    ESP_GOTO_ON_FALSE( (strlen(time_into_interval_config->name) <= TIME_INTO_INTERVAL_NAME_MAX_SIZE), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval name cannot exceed 20-characters, time-into-interval handle initialization failed" );
    ESP_GOTO_ON_FALSE( (time_into_interval_config->interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval interval period cannot be 0, time-into-interval handle initialization failed" );

    /* validate period and offset intervals */
    int64_t interval_delta = datalogger_normalize_interval_to_sec(time_into_interval_config->interval_type, time_into_interval_config->interval_period) - 
                             datalogger_normalize_interval_to_sec(time_into_interval_config->interval_type, time_into_interval_config->interval_offset); 
    ESP_GOTO_ON_FALSE( (interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval interval period must be larger than the interval offset, time-into-interval handle initialization failed" );
    
    /* validate memory availability for time into interval handle */
    out_handle = (time_into_interval_handle_t)calloc(1, sizeof(time_into_interval_t)); 
    ESP_GOTO_ON_FALSE( out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for time-into-interval handle, time-into-interval handle initialization failed" );

    /* initialize task schedule state object parameters */
    strcpy(out_handle->name, time_into_interval_config->name);
    out_handle->epoch_timestamp = 0;
    out_handle->interval_type   = time_into_interval_config->interval_type;
    out_handle->interval_period = time_into_interval_config->interval_period;
    out_handle->interval_offset = time_into_interval_config->interval_offset;
    out_handle->hash_code       = datalogger_get_hash_code();

    /* set epoch timestamp of the next scheduled time-into-interval event */
    ESP_GOTO_ON_ERROR( datalogger_set_epoch_timestamp_event(out_handle->interval_type, 
                                                            out_handle->interval_period, 
                                                            out_handle->interval_offset, 
                                                            &out_handle->epoch_timestamp), 
                                                            err_out_handle, TAG, "unable to set epoch timestamp, time-into-interval handle initialization failed" );

    /* set output handle */
    *time_into_interval_handle = out_handle;

    return ESP_OK;

    err_out_handle:
        free(out_handle);
    err:
        return ret;
}

bool time_into_interval(time_into_interval_handle_t time_into_interval_handle) {
    bool state = false;

    /* validate arguments */
    if(!(time_into_interval_handle)) {
        return state;
    }

    // get system unix epoch timestamp (UTC)
    uint64_t now_unix_msec = datalogger_get_epoch_timestamp_msec();

    // compute time delta until next time into interval condition
    int64_t delta_msec = time_into_interval_handle->epoch_timestamp - now_unix_msec;

    // validate time delta, when delta is <= 0, time has elapsed
    if(delta_msec <= 0) {
        // set time-into-interval state to true - intervale has lapsed
        state = true;

        /* set next event timestamp (UTC) */
        datalogger_set_epoch_timestamp_event(time_into_interval_handle->interval_type, 
                                            time_into_interval_handle->interval_period, 
                                            time_into_interval_handle->interval_offset, 
                                            &time_into_interval_handle->epoch_timestamp);
    }
    
    return state;
}

esp_err_t time_into_interval_delay(time_into_interval_handle_t time_into_interval_handle) {
    // validate arguments
    ESP_ARG_CHECK( time_into_interval_handle );

    // get system unix epoch timestamp (UTC)
    uint64_t now_unix_msec = datalogger_get_epoch_timestamp_msec();

    // compute time delta until next scan event
    int64_t delta_msec = time_into_interval_handle->epoch_timestamp - now_unix_msec;

    // validate time is into the future, otherwise, reset next epoch time
    if(delta_msec < 0) {
        // reset epoch time of the next schedule task
        time_into_interval_handle->epoch_timestamp = 0;

        // set epoch timestamp of the next scheduled task
        datalogger_set_epoch_timestamp_event(time_into_interval_handle->interval_type, 
                                            time_into_interval_handle->interval_period, 
                                            time_into_interval_handle->interval_offset, 
                                            &time_into_interval_handle->epoch_timestamp);

        // compute time delta for next event
        delta_msec = time_into_interval_handle->epoch_timestamp - now_unix_msec;
    }

    // compute ticks delay from time delta
    TickType_t delay = (delta_msec / portTICK_PERIOD_MS);

    // delay the task per ticks delay
    vTaskDelay( delay );

    // set epoch timestamp of the next scheduled task
    datalogger_set_epoch_timestamp_event(time_into_interval_handle->interval_type, 
                                        time_into_interval_handle->interval_period, 
                                        time_into_interval_handle->interval_offset, 
                                        &time_into_interval_handle->epoch_timestamp);

    return ESP_OK;
}

esp_err_t time_into_interval_del(time_into_interval_handle_t time_into_interval_handle) {
    /* free resource */
    if(time_into_interval_handle) {
        free(time_into_interval_handle);
    }

    return ESP_OK;
}