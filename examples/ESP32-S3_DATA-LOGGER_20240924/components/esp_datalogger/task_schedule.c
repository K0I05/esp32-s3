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
 * @file task_schedule.c
 *
 * ESP-IDF FreeRTOS task extension
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "task_schedule.h"
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
static const char *TAG = "task_schedule";



esp_err_t task_schedule_new(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, task_schedule_handle_t *task_schedule_handle) {
    esp_err_t               ret = ESP_OK;
    task_schedule_handle_t  out_handle;
    
    /* validate arguments */
    ESP_ARG_CHECK( interval_period );

    /* validate memory availability for task schedule handle */
    out_handle = (task_schedule_handle_t)calloc(1, sizeof(task_schedule_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for task schedule handle, new task schedule failed");

    /* validate memory availability for task schedule handle parameters */
    out_handle->params = (task_schedule_params_t*)calloc(1, sizeof(task_schedule_params_t));
    ESP_GOTO_ON_FALSE(out_handle->params, ESP_ERR_NO_MEM, err, TAG, "no memory for task schedule handle configuration parameters, new task schedule failed");

    /* initialize task schedule state object parameters */
    out_handle->params->epoch_time      = 0;
    out_handle->params->interval_type   = interval_type;
    out_handle->params->interval_period = interval_period;
    out_handle->params->interval_offset = interval_offset;

    /* set epoch time of the next scheduled task */
    datalogger_set_epoch_time_event(out_handle->params->interval_type, out_handle->params->interval_period, &out_handle->params->epoch_time);

    /* set output handle */
    *task_schedule_handle = out_handle;

    return ESP_OK;

    err:
        free(out_handle);
        return ret;
}

esp_err_t task_schedule_delay(task_schedule_handle_t task_schedule_handle) {
    TickType_t      delay;
    struct timeval  tv_now;
    uint64_t        now_msec;
    uint64_t        next_msec;
    int64_t         delta_msec;

    /* validate arguments */
    ESP_ARG_CHECK( task_schedule_handle );

    // get system time
    gettimeofday(&tv_now, NULL);

    // convert system time to msec
    now_msec = (uint64_t)tv_now.tv_sec * 1000U + (uint64_t)tv_now.tv_usec / 1000U;

    // convert next scan event epoch time to msec
    next_msec = (uint64_t)task_schedule_handle->params->epoch_time * 1000U;

    // compute time delta until next scan event
    delta_msec = next_msec - now_msec;

    // validate time is into the future, otherwise, reset next epoch time
    if(delta_msec < 0) {
        // reset epoch time of the next schedule task
        task_schedule_handle->params->epoch_time = 0;

        // set epoch time of the next scheduled task
        datalogger_set_epoch_time_event(task_schedule_handle->params->interval_type, task_schedule_handle->params->interval_period, &task_schedule_handle->params->epoch_time);


        // convert next scan event epoch time to msec
        next_msec = (uint64_t)task_schedule_handle->params->epoch_time * 1000U;

        // compute time delta until next scan event
        delta_msec = next_msec - now_msec;
    }

    // compute ticks delay from time delta
    delay = (delta_msec / portTICK_PERIOD_MS);

    // delay the task per ticks delay
    vTaskDelay( delay );

    /* set epoch time of the next scheduled task */
    datalogger_set_epoch_time_event(task_schedule_handle->params->interval_type, task_schedule_handle->params->interval_period, &task_schedule_handle->params->epoch_time);

    return ESP_OK;
}

esp_err_t task_schedule_del(task_schedule_handle_t task_schedule_handle) {
    /* free resource */
    free(task_schedule_handle);

    return ESP_OK;
}




