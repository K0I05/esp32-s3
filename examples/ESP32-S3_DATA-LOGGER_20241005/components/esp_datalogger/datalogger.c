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
 * @file datalogger.c
 *
 * ESP-IDF library for DATA-LOGGER
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "datalogger.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <systemtable.h>
#include <datatable.h>

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "datalogger";



static inline void datalogger_event_handler(void *handle, datalogger_event_t event) {
    
    switch(event.source) {
        case DATALOGGER_EVENT_DL: {
            datalogger_handle_t datalogger_handle = (datalogger_handle_t)handle;
            if(datalogger_handle != NULL) {
                ESP_LOGW(TAG, "datalogger_event_handler:datalogger_handle-> %s", event.message);
            }
            break;
        }
        case DATALOGGER_EVENT_DT: {
            datatable_handle_t datatable_handle = (datatable_handle_t)handle;
            if(datatable_handle != NULL) {
                ESP_LOGW(TAG, "datalogger_event_handler:datatable_handle-> %s", event.message);
            }
            break;
        }
        case DATALOGGER_EVENT_TII: {
            time_into_interval_handle_t time_into_interval_handle = (time_into_interval_handle_t)handle;
            if(time_into_interval_handle != NULL) {
                ESP_LOGW(TAG, "datalogger_event_handler:time_into_interval_handle-> %s", event.message);
            }
            break;
        }
    }
}

esp_err_t datalogger_init(const char *name, datalogger_handle_t *datalogger_handle) {
    esp_err_t           ret = ESP_OK;
    datalogger_handle_t out_handle;

    /* validate memory availability for data-logger handle */
    out_handle = (datalogger_handle_t)calloc(1, sizeof(datalogger_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger handle, data-logger handle initialization failed");

    /* validate memory availability for data-logger data-table handles */
    out_handle->datatable_handles = (datalogger_datatable_handle_t*)calloc(DATALOGGER_DT_HANDLES_MAXIMUM, sizeof(datalogger_datatable_handle_t));
    ESP_GOTO_ON_FALSE( out_handle->datatable_handles, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger data-table handles, data-logger handle initialization failed" );

    /* attempt to initialize data-logger system-table handle */
    ESP_GOTO_ON_ERROR( systemtable_init(&out_handle->systemtable_handle), err, TAG, "unable to initialize system-table, data-logger handle initialization failed" );

    /* initialize data-logger state object */
    strcpy(out_handle->name, name);
    out_handle->datatable_handles_count = 0;

   // out_handle->event_handler = datalogger_event_handler;

    datalogger_event_t dl_event = {
        .source  = DATALOGGER_EVENT_DL,
        .type    = DATALOGGER_EVENT_DL_INIT,
        .message = "data-logger initialized successfully"
    };

    out_handle->event_handler(out_handle, dl_event);

    
    /* set output handle */
    *datalogger_handle = out_handle;

    return ESP_OK;

    err:
        free(out_handle);
        return ret;
}


esp_err_t datalogger_new_datatable(datalogger_handle_t datalogger_handle, const datatable_config_t *datatable_config, datatable_handle_t *datatable_handle) {
    datatable_handle_t out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle && datatable_config );

    /* validate data-table handles count before attempting to add one */
    ESP_RETURN_ON_FALSE( (datalogger_handle->datatable_handles_count + 1 < DATALOGGER_DT_HANDLES_MAXIMUM + 1), ESP_ERR_INVALID_SIZE, TAG, "number of data-table handles exceeds the maximum allowed, data-logger new data-table failed" );
    
    /* attempt to create a new data-table handle */
    ESP_RETURN_ON_ERROR( datatable_init(datatable_config, &out_handle), TAG, "unable to create new data-table, data-logger new data-table failed." );

    //out_handle->event_handler = datalogger_event_handler;

    /* append data-logger data-table reference */
    datalogger_handle->datatable_handles[datalogger_handle->datatable_handles_count].datatable_handle = out_handle;

    /* set out handle */
    *datatable_handle = datalogger_handle->datatable_handles[datalogger_handle->datatable_handles_count].datatable_handle;

    /* increment data-table handles count */
    datalogger_handle->datatable_handles_count += 1;

    datalogger_event_t dl_event = {
        .source  = DATALOGGER_EVENT_DL,
        .type    = DATALOGGER_EVENT_DT_INIT,
        .message = "data-table initialized successfully"
    };

    datalogger_handle->event_handler(datalogger_handle, dl_event);

    return ESP_OK;
}



esp_err_t datalogger_get_datatable_count(datalogger_handle_t datalogger_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* set data-table handles count output parameter */
    *count = datalogger_handle->datatable_handles_count;

    return ESP_OK;
}