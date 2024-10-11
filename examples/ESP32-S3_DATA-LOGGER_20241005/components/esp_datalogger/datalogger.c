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


//static inline void datalogger_event_handler(void *handle, datalogger_event_t event) {
extern void datalogger_event_handler(void *handle, datalogger_event_t event) {
    if(handle == NULL) return;

    switch(event.source) {
        case DATALOGGER_EVENT_DL: {
            datalogger_handle_t datalogger_handle = (datalogger_handle_t)handle;
            if(datalogger_handle != NULL) {
                ESP_LOGW(TAG, "datalogger_event_handler:datalogger_handle->%s %s", datalogger_handle->name, event.message);
            }
            break;
        }
        case DATALOGGER_EVENT_DT: {
            datatable_handle_t datatable_handle = (datatable_handle_t)handle;
            if(datatable_handle != NULL) {
                ESP_LOGW(TAG, "datalogger_event_handler:datatable_handle->%s %s", datatable_handle->name, event.message);
            }
            break;
        }
        case DATALOGGER_EVENT_TII: {
            time_into_interval_handle_t time_into_interval_handle = (time_into_interval_handle_t)handle;
            if(time_into_interval_handle != NULL) {
                ESP_LOGW(TAG, "datalogger_event_handler:time_into_interval_handle->%s %s", time_into_interval_handle->name, event.message);
            }
            break;
        }
    }
}

static inline esp_err_t datalogger_invoke_event(datalogger_handle_t datalogger_handle, datalogger_event_types_t event_type, const char* message) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* validate event handler */
    if(!datalogger_handle->event_handler) return ESP_ERR_INVALID_STATE;

    /* initialize event structure */
    datalogger_event_t dl_event = {
        .source  = DATALOGGER_EVENT_DL,
        .type    = event_type,
        .message = message
    };

    /* invoke event */
    datalogger_handle->event_handler(datalogger_handle, dl_event);

    return ESP_OK;
}



esp_err_t datalogger_init(const char *name, datalogger_handle_t *datalogger_handle) {
    esp_err_t           ret = ESP_OK;
    datalogger_handle_t out_handle;

    /* validate memory availability for data-logger handle */
    out_handle = (datalogger_handle_t)calloc(1, sizeof(datalogger_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger handle, data-logger handle initialization failed");

    /* validate memory availability for data-logger data-table handles */
    out_handle->datatable_handles = (datalogger_datatable_handle_t**)calloc(DATALOGGER_DT_HANDLES_MAXIMUM, sizeof(datalogger_datatable_handle_t*));
    ESP_GOTO_ON_FALSE( out_handle->datatable_handles, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger data-table handles, data-logger handle initialization failed" );

    /* attempt to initialize data-logger system-table handle */
    ESP_GOTO_ON_ERROR( systemtable_init(&out_handle->systemtable_handle), err, TAG, "unable to initialize system-table, data-logger handle initialization failed" );

    /* initialize data-logger state object */
    strcpy(out_handle->name, name);
    out_handle->datatable_handles_count = 0;
    out_handle->event_handler = datalogger_event_handler;

    /* invoke data-logger event */
    datalogger_invoke_event(out_handle, DATALOGGER_EVENT_DL_INIT, "data-logger initialized successfully");

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
    
    /* copy data-table config */
    datatable_config_t dt_config = {
        .columns_size       = datatable_config->columns_size,
        .rows_size          = datatable_config->rows_size,
        .data_storage_type  = datatable_config->data_storage_type,
        .sampling_config    = datatable_config->sampling_config,
        .processing_config  = datatable_config->processing_config,
        .event_handler      = datalogger_event_handler
    };
    strcpy(dt_config.name, datatable_config->name);

    /* attempt to create a new data-table handle */
    ESP_RETURN_ON_ERROR( datatable_init(&dt_config, &out_handle), TAG, "unable to create new data-table, data-logger new data-table failed." );

    datalogger_datatable_handle_t* dl_dt_hdl = (datalogger_datatable_handle_t*)calloc(1, sizeof(datalogger_datatable_handle_t));
    ESP_RETURN_ON_FALSE( dl_dt_hdl, ESP_ERR_NO_MEM, TAG, "no memory for data-logger data-table handle, data-table handle initialization failed" );

    /* set data-logger data-table handle */
    dl_dt_hdl->datatable_handle = out_handle;

    /* set data-logger data-table handle */
    datalogger_handle->datatable_handles[datalogger_handle->datatable_handles_count] = dl_dt_hdl;

    /* increment data-table handles count */
    datalogger_handle->datatable_handles_count += 1;

    /* set out handle */
    *datatable_handle = dl_dt_hdl->datatable_handle;

    return ESP_OK;
}

esp_err_t datalogger_get_datatable_count(datalogger_handle_t datalogger_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* set data-table handles count output parameter */
    *count = datalogger_handle->datatable_handles_count;

    return ESP_OK;
}