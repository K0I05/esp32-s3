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


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "datalogger";


/**
 * @brief Checks if the data-table handle exist by index and referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the data-table handle of interest.
 * @param[in] index Data-table handle index to check if it exist agaist the referenced data-logger handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG when the index is out of range and data-table handle does not exist, 
 * ESP_ERR_INVALID_STATE when the index returns an invalid data-table handle state.
 */
static inline esp_err_t datalogger_datatable_exist(datalogger_handle_t datalogger_handle, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE((index < datalogger_handle->datatable_handles_count), ESP_ERR_INVALID_ARG, TAG, "index is out of range, data-table handle exist failed");

    /* validate data-table handle state */
    ESP_RETURN_ON_FALSE((datalogger_handle->datatable_handles[index] != NULL), ESP_ERR_INVALID_STATE, TAG, "index returned an invalid data-table handle state, data-table handle exist failed");

    return ESP_OK;
}

/**
 * @brief Checks if the time-into-interval handle exist by index and referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the time-into-interval handle of interest.
 * @param[in] index Time-into-interval handle index to check if it exist agaist the referenced data-logger handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG when the index is out of range and time-into-interval handle does not exist, 
 * ESP_ERR_INVALID_STATE when the index returns an invalid time-into-interval handle state.
 */
static inline esp_err_t datalogger_time_into_interval_exist(datalogger_handle_t datalogger_handle, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE((index < datalogger_handle->time_into_interval_handles_count), ESP_ERR_INVALID_ARG, TAG, "index is out of range, gtime-into-interval handle exist failed");

    /* validate time-into-interval handle state */
    ESP_RETURN_ON_FALSE((datalogger_handle->time_into_interval_handles[index] != NULL), ESP_ERR_INVALID_STATE, TAG, "index returned an invalid time-into-interval handle state, time-into-interval handle exist failed");

    return ESP_OK;
}

/**
 * @brief Checks if the task-schedule handle exist by index and referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the task-schedule handle of interest.
 * @param[in] index Task-schedule handle index to check if it exist agaist the referenced data-logger handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG when the index is out of range and task-schedule handle does not exist, 
 * ESP_ERR_INVALID_STATE when the index returns an invalid task-schedule handle state.
 */
static inline esp_err_t datalogger_task_schedule_exist(datalogger_handle_t datalogger_handle, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE((index < datalogger_handle->task_schedule_handles_count), ESP_ERR_INVALID_ARG, TAG, "index is out of range, task-schedule handle exist failed");

    /* validate task-schedule handle state */
    ESP_RETURN_ON_FALSE((datalogger_handle->task_schedule_handles[index] != NULL), ESP_ERR_INVALID_STATE, TAG, "index returned an invalid task-schedule handle state, task-schedule handle exist failed");

    return ESP_OK;
}




esp_err_t datalogger_new(const char *name, datalogger_handle_t *datalogger_handle) {
    esp_err_t           ret = ESP_OK;
    datalogger_handle_t out_handle;

    /* validate memory availability for data-logger handle */
    out_handle = (datalogger_handle_t)calloc(1, sizeof(datalogger_handle_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger handle, new data-logger handle failed");

    /* validate memory availability for data-logger data-table handles */
    //out_handle->datatable_handles = (datatable_handle_t*)malloc(DATALOGGER_DT_HANDLES_MAXIMUM * sizeof(datatable_handle_t));
    //ESP_GOTO_ON_FALSE( out_handle->datatable_handles, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger data-table handles, new data-logger handle failed" );

    /* validate memory availability for data-logger time-into-inteval handles */
    //out_handle->time_into_interval_handles = (time_into_interval_handle_t*)malloc(DATALOGGER_TTI_HANDLES_MAXIMUM * sizeof(time_into_interval_handle_t));
    //ESP_GOTO_ON_FALSE( out_handle->time_into_interval_handles, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger time-into-interval handles, new data-logger handle failed" );

    /* validate memory availability for data-logger task-schedule handles */
    //out_handle->task_schedule_handles = (task_schedule_handle_t*)malloc(DATALOGGER_TS_HANDLES_MAXIMUM * sizeof(task_schedule_handle_t));
    //ESP_GOTO_ON_FALSE( out_handle->task_schedule_handles, ESP_ERR_NO_MEM, err, TAG, "no memory for data-logger task-schedule handles, new data-logger handle failed" );

    /* initialize data-logger state object */
    strcpy(out_handle->name, name);
    out_handle->datatable_handles_count             = 0;
    out_handle->time_into_interval_handles_count    = 0;
    out_handle->task_schedule_handles_count         = 0;
    
    /* set output handle */
    *datalogger_handle = out_handle;

    return ESP_OK;

    err:
        free(out_handle);
        return ret;
}


esp_err_t datalogger_new_datatable(const char *name, const uint8_t columns_size, const uint16_t rows_size, 
                    const datalogger_time_interval_types_t sampling_interval_type, const uint16_t sampling_interval_period, const uint16_t sampling_interval_offset,
                    const datalogger_time_interval_types_t processing_interval_type, const uint16_t processing_interval_period, const uint16_t processing_interval_offset, 
                    const datatable_data_storage_types_t data_storage_type, datalogger_handle_t datalogger_handle, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* validate data-table handles count before attempting to add one */
    ESP_RETURN_ON_FALSE( (datalogger_handle->datatable_handles_count + 1 < DATALOGGER_DT_HANDLES_MAXIMUM + 1), ESP_ERR_INVALID_SIZE, TAG, "number of data-table handles exceeds the maximum allowed, data-logger new data-table failed" );
    
    /* attempt to create a new data-table handle */
    //ESP_RETURN_ON_ERROR( datatable_new(name, columns_size, rows_size, sampling_interval_type, sampling_interval_period, sampling_interval_offset,
    //                                processing_interval_type, processing_interval_period, processing_interval_offset,
    //                                data_storage_type, &datalogger_handle->datatable_handles[datalogger_handle->datatable_handles_count]), 
    //                    TAG, "unable to create new data-table, data-logger new data-table failed." );
    
    /* set data-table handle index output parameter for the new data-table handle */
    *index = datalogger_handle->datatable_handles_count;
    
    /* increment data-table handles count */
    datalogger_handle->datatable_handles_count += 1;

    return ESP_OK;
}

esp_err_t datalogger_get_datatable(datalogger_handle_t datalogger_handle, uint8_t index, datatable_handle_t *datatable_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* validate data-table handle exist and state */
    ESP_RETURN_ON_ERROR( datalogger_datatable_exist(datalogger_handle, index), TAG, "data-table does not appear to exist or it is in an invalid state, data-logger get data-table failed" );

    /* set data-table handle output parameter */
    *datatable_handle = datalogger_handle->datatable_handles[index];

    return ESP_OK;
}

esp_err_t datalogger_get_datatables_count(datalogger_handle_t datalogger_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( datalogger_handle );

    /* set data-table handles count output parameter */
    *count = datalogger_handle->datatable_handles_count;

    return ESP_OK;
}