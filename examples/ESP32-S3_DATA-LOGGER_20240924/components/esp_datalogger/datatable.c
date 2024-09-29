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
 * @file datatable.c
 *
 * ESP-IDF library for DATA-TABLE
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "datatable.h"
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
static const char *TAG = "data-table";


/**
 * @brief Converts a data-table column process-type to a short abbreviated textual string.  The shortened and abbreviated
 * textual string of the process-type uses a julian convention e.g. `DATATABLE_COLUMN_PROCESS_SMP` represents a 
 * data-table column process-type as a sample and will output `Smp` for the short abbreviated textual string.
 * 
 * For full textual representation of the data-table column process-type, see `datatable_process_type_to_string` function.
 * 
 * @param process_type Data-table column process-type to convert to a short abbreviated textual string.
 * @return char* Data-table column process-type as a short abbreviated textual string.
 */
static inline const char* datatable_process_type_to_short_string(const datatable_column_process_types_t process_type) {
    static char process[8] = "-";

    /* normalize */
    switch(process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            strcpy(process, "Smp");
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            strcpy(process, "Avg");
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            strcpy(process, "Min");
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            strcpy(process, "Max");
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            strcpy(process, "Min_TS");
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            strcpy(process, "Max_TS");
            break;
    }

    return process;
}

/**
 * @brief Converts a data-table column process-type to a textual string.  The textual string of the process-type 
 * uses a julian convention e.g. `DATATABLE_COLUMN_PROCESS_SMP` represents a data-table column process-type 
 * as a sample and will output `Sample` for the textual string.
 * 
 * For a short abbreviated textual representation of the data-table column process-type, see `datatable_process_type_to_short_string` 
 * function.
 * 
 * @param process_type Data-table column process-type to convert to a textual string.
 * @return char* Data-table column process-type as a textual string.
 */
static inline const char* datatable_process_type_to_string(const datatable_column_process_types_t process_type) {
    static char process[20] = "-";

    /* normalize */
    switch(process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            strcpy(process, "Sample");
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            strcpy(process, "Average");
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            strcpy(process, "Minimum");
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            strcpy(process, "Maximum");
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            strcpy(process, "Minimum_TimeStamp");
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            strcpy(process, "Maximum_TimeStamp");
            break;
    }

    return process;
}

/**
 * @brief Serializes data-logger time interval type to a textual string for json formating.
 * 
 * @param interval_type Data-logger time interval type to serialize.
 * @return char* Serialized data-logger time interval type as a textual string.
 */
static inline const char* datatable_json_serialize_interval_type(const datalogger_time_interval_types_t interval_type) {
    static char type[8] = "-";

    /* normalize  */
    switch(interval_type) {
        case DATALOGGER_TIME_INTERVAL_SEC:
            strcpy(type, "second");
            break;
        case DATALOGGER_TIME_INTERVAL_MIN:
            strcpy(type, "minute");
            break;
        case DATALOGGER_TIME_INTERVAL_HR:
            strcpy(type, "hour");
            break;
    }

    return type;
}

/**
 * @brief Serializes data-table column data-type to a textual string for json formating.
 * 
 * @param data_type Data-table column data-type to serialize.
 * @return char* Serialized data-table column data-type as a textual string.
 */
static inline const char* datatable_json_serialize_column_data_type(const datatable_column_data_types_t data_type) {
    static char data[8] = "-";

    /* normalize  */
    switch(data_type) {
        case DATATABLE_COLUMN_DATA_ID:
            strcpy(data, "id");
            break;
        case DATATABLE_COLUMN_DATA_TS:
            strcpy(data, "ts");
            break;
        case DATATABLE_COLUMN_DATA_VECTOR:
            strcpy(data, "vector");
            break;
        case DATATABLE_COLUMN_DATA_BOOL:
            strcpy(data, "bool");
            break;
        case DATATABLE_COLUMN_DATA_FLOAT:
            strcpy(data, "float");
            break;
        case DATATABLE_COLUMN_DATA_INT16:
            strcpy(data, "int16");
            break;
    }

    return data;
}

/**
 * @brief Serializes data-table column process-type to a textual string for json formating.
 * 
 * @param process_type Data-table column process-type to serialize.
 * @return char* Serialized data-table column process-type as a textual string.
 */
static inline const char* datatable_json_serialize_process_type(const datatable_column_process_types_t process_type) {
    static char process[20] = "-";

    /* normalize */
    switch(process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            strcpy(process, "sample");
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            strcpy(process, "average");
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            strcpy(process, "minimum");
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            strcpy(process, "maximum");
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            strcpy(process, "minimum-timestamp");
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            strcpy(process, "maximum-timestamp");
            break;
    }

    return process;
}

/**
 * @brief Gets the data-table column data buffer size.  The data-table column buffer size is a whole number that
 * represents the total number of samples that can be pushed onto the stack for analytical processing which is 
 * defined by the process-type setting for each column configured in the data-table.
 * 
 * As an example, if the data-table sampling task schedule event occurs every 10-seconds and the data-table 
 * processing event occurs every minute, data-table column data buffer created for analytical processing will be
 * sized to store a maximum of 6 samples.  If the data-table column data buffer created for analytical processing 
 * is proccesed with less than 6 samples, the data-table record is skipped, and logged as a skipped record  
 * processing event for the data-table.
 * 
 * @param datatable_handle Data-table handle.
 * @param size Data-table column data buffer size of maximum samples that can be stored in the data buffer.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_get_column_data_buffer_size(datatable_handle_t datatable_handle, uint16_t *size) {
    uint64_t sampling_interval;
    uint64_t processing_interval;
    int64_t  interval_delta;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* normalize sampling and processing periods to seconds, and set delta interval */
    sampling_interval   = datalogger_normalize_interval_to_sec(datatable_handle->sampling_ts_handle->params->interval_type, datatable_handle->sampling_ts_handle->params->interval_period);
    processing_interval = datalogger_normalize_interval_to_sec(datatable_handle->processing_interval_type, datatable_handle->processing_interval_period);
    interval_delta      = processing_interval - sampling_interval;

    //ESP_LOGD(TAG, "datatable_get_column_data_buffer_size (delta %lli): processing_interval(%llu) / sampling_interval(%llu)", interval_delta, processing_interval, sampling_interval);

    /* validate sampling and processing intervals to avoid dividing by 0 */

    /* validate sampling interval */
    ESP_RETURN_ON_FALSE((sampling_interval > 0), ESP_ERR_INVALID_SIZE, TAG, "sampling interval is out of range, sampling interval cannot be 0, get column data buffer size failed");

    /* validate processing interval */
    ESP_RETURN_ON_FALSE((processing_interval > 0), ESP_ERR_INVALID_SIZE, TAG, "processing interval is out of range, processing interval cannot be 0, get column data buffer size failed");

    /* validate sampling and processing interval delta - processing interval must be greater than the sampling interval */
    ESP_RETURN_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_SIZE, TAG, "sampling and processing intervals are out of range, sampling interval cannot be greater than the processing interval, get column data buffer size failed");

    /* calculate data-table column data buffer sample size */
    *size = (uint16_t)(processing_interval / sampling_interval);

    return ESP_OK;
}

/**
 * @brief Checks if the data-table column exist by column index.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] index Data-table column index to check if it exist.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG when the index is out of range and column does not exist.
 */
static inline esp_err_t datatable_column_exist(datatable_handle_t datatable_handle, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE((index < datatable_handle->columns_size), ESP_ERR_INVALID_ARG, TAG, "index is out of range, get column failed");

    return ESP_OK;
}

/**
 * @brief Checks if the data-table is full (i.e. number of rows matches configured rows size).
 * 
 * @param datatable_handle Data-table handle.
 * @param full True when data-table is full, otherwise, false.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_is_full(datatable_handle_t datatable_handle, bool *full) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate if the data-table is full and set output parameter */
    if(datatable_handle->rows_count >= datatable_handle->rows_size) {
        *full = true;
    } else {
        *full = false;
    }

    return ESP_OK;
}

/**
 * @brief Pops the top data-table row and shifts the index of remaining rows up by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_fifo_rows(datatable_handle_t datatable_handle) {
    datatable_row_t* tmp_rows;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate memory availability for default data-table tmp rows */
    tmp_rows = (datatable_row_t*)malloc(datatable_handle->rows_size * sizeof(datatable_row_t));
    ESP_RETURN_ON_FALSE(tmp_rows, ESP_ERR_NO_MEM, TAG, "no memory for data-table temporary rows, data-table fifo rows failed");

    /* copy working rows to tmp rows, skip the first row */
    for(int r = 1; r < datatable_handle->rows_count; r++) {
        tmp_rows[r - 1] = datatable_handle->rows[r];
    }

    /* adjust rows index and count attributes */
    datatable_handle->rows_index -= 1;
    datatable_handle->rows_count -= 1;

    /* copy tmp rows to working rows */
    for(int r = 0; r < datatable_handle->rows_count; r++) {
        datatable_handle->rows[r] = tmp_rows[r];
    }

    /* free up memory used by the tmp rows */
    if(tmp_rows) {
        free(tmp_rows);
    }

    return ESP_OK;
}

/**
 * @brief Resets data-table rows, this is full reset, all data is deleted and data-table row array is re-initialized per configured row size.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_reset_rows(datatable_handle_t datatable_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* reset rows by freeing up memory */
    if(datatable_handle->rows) {
        free(datatable_handle->rows);
    }

    /* reset row attributes */
    datatable_handle->rows_index = 0;
    datatable_handle->rows_count = 0;

    /* validate memory availability for default data-table rows */
    datatable_handle->rows = (datatable_row_t*)malloc(datatable_handle->rows_size * sizeof(datatable_row_t));
    ESP_RETURN_ON_FALSE(datatable_handle->rows, ESP_ERR_NO_MEM, TAG, "no memory for data-table rows, data-table reset rows failed");

    return ESP_OK;
}

/**
 * @brief Resets data-table column data buffer by column index.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Index of data-table column to reset column data buffer.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_reset_data_buffer(datatable_handle_t datatable_handle, const uint8_t index) {
    uint16_t dt_data_buffer_size = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range, data-table reset column data buffer failed" );

    /* validate data-table column data buffer size by processing type */
    if(datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size for data-table column data buffer */
        dt_data_buffer_size = 1;
    } else {
        /* determine size of samples for data-table column data buffer */
        ESP_RETURN_ON_ERROR( datatable_get_column_data_buffer_size(datatable_handle, &dt_data_buffer_size),TAG, "get column data buffer size for data-table reset column data buffers failed" );
    }
    
    /* handle each column by data-type */
    switch(datatable_handle->columns[index].data_type) {
        case DATATABLE_COLUMN_DATA_ID:
            /* no data buffer to process */
            break;
        case DATATABLE_COLUMN_DATA_TS:
            /* no data buffer to process */
            break;
        case DATATABLE_COLUMN_DATA_VECTOR:
            /* free-up memory */
            if(datatable_handle->columns[index].data.buffer.vector_samples) {
                free(datatable_handle->columns[index].data.buffer.vector_samples);
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count = 0;

            /* validate memory availability for data-table tmp data buffer samples */
            datatable_handle->columns[index].data.buffer.vector_samples = (datatable_vector_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_vector_column_data_type_t));
            ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data.buffer.vector_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table vector column data buffer, data-table reset column data buffers failed");

            break;
        case DATATABLE_COLUMN_DATA_BOOL:
            /* free-up memory */
            if(datatable_handle->columns[index].data.buffer.bool_samples) {
                free(datatable_handle->columns[index].data.buffer.bool_samples);
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count = 0;
            
            /* validate memory availability for data-table tmp data buffer samples */
            datatable_handle->columns[index].data.buffer.bool_samples = (datatable_bool_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_bool_column_data_type_t));
            ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data.buffer.bool_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table bool column data buffer, data-table reset column data buffers failed");

            break;
        case DATATABLE_COLUMN_DATA_FLOAT:
            /* free-up memory */
            if(datatable_handle->columns[index].data.buffer.float_samples) {
                free(datatable_handle->columns[index].data.buffer.float_samples);
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count = 0;

            /* validate memory availability for data-table tmp data buffer samples */
            datatable_handle->columns[index].data.buffer.float_samples = (datatable_float_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_float_column_data_type_t));
            ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data.buffer.float_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table float column data buffer, data-table reset column data buffers failed");

            break;
        case DATATABLE_COLUMN_DATA_INT16:
            /* free-up memory */
            if(datatable_handle->columns[index].data.buffer.int16_samples) {
                free(datatable_handle->columns[index].data.buffer.int16_samples);
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count = 0;

            /* validate memory availability for data-table tmp data buffer samples */
            datatable_handle->columns[index].data.buffer.int16_samples = (datatable_int16_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_int16_column_data_type_t));
            ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data.buffer.int16_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table int16 column data buffer, data-table reset column data buffers failed");

            break;
    }

    return ESP_OK;
}

/**
 * @brief Resets data-table column data buffer for configured columns.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_reset_data_buffers(datatable_handle_t datatable_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* handle each data-table column */
    for(uint8_t ci = 0; ci <= datatable_handle->columns_index; ci++) {
        /* pop and shift data buffer stack */
        ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_handle, ci),TAG, "reset column data buffer for data-table reset column data buffers failed" );
    }

    /* reset sampling count */
    datatable_handle->sampling_count = 0;

    return ESP_OK;
}

/**
 * @brief Pops the top data-table data buffer by column index and shifts the index of remaining 
 * samples up by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param datatable_handle Data-table handle.
 * @param index Data-table column index to FIFO data buffer to pop.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_fifo_data_buffer(datatable_handle_t datatable_handle, const uint8_t index) {
    uint16_t dt_data_buffer_size = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range, data-table fifo column data buffer failed" );

    /* validate data-table column data buffer size by processing type */
    if(datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size for data-table column data buffer */
        dt_data_buffer_size = 1;
    } else {
        /* determine size of samples for data-table column data buffer */
        ESP_RETURN_ON_ERROR( datatable_get_column_data_buffer_size(datatable_handle, &dt_data_buffer_size),TAG, "get column data buffer size for data-table fifo column data buffer failed" );
    }
    
    /* handle each column by data-type */
    switch(datatable_handle->columns[index].data_type) {
        case DATATABLE_COLUMN_DATA_ID:
            /* no data buffer */
            break;
        case DATATABLE_COLUMN_DATA_TS:
            /* no data buffer */
            break;
        case DATATABLE_COLUMN_DATA_VECTOR:
            datatable_vector_column_data_type_t* tmp_vector_samples;

            /* validate memory availability for data-table tmp data buffer samples */
            tmp_vector_samples = (datatable_vector_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_vector_column_data_type_t));
            ESP_RETURN_ON_FALSE(tmp_vector_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table temporary vector column data buffer, data-table fifo column data buffer failed");

            /* copy working samples to tmp data buffer samples, skip the first data buffer sample */
            for(int dbs = 1; dbs < dt_data_buffer_size; dbs++) {
                tmp_vector_samples[dbs - 1] = datatable_handle->columns[index].data.buffer.vector_samples[dbs];
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count -= 1;

            /* copy tmp data buffer samples to working data buffer samples */
            for(int dbs = 0; dbs < datatable_handle->columns[index].data.samples_count; dbs++) {
                datatable_handle->columns[index].data.buffer.vector_samples[dbs] = tmp_vector_samples[dbs];
            }

            /* free-up memory */
            if(tmp_vector_samples) {
                free(tmp_vector_samples);
            }

            break;
        case DATATABLE_COLUMN_DATA_BOOL:
            datatable_bool_column_data_type_t* tmp_bool_samples;
            
            /* validate memory availability for data-table tmp data buffer samples */
            tmp_bool_samples = (datatable_bool_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_bool_column_data_type_t));
            ESP_RETURN_ON_FALSE(tmp_bool_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table temporary bool column data buffer, data-table fifo column data buffer failed");

            /* copy working samples to tmp data buffer samples, skip the first data buffer sample */
            for(int dbs = 1; dbs < dt_data_buffer_size; dbs++) {
                tmp_bool_samples[dbs - 1] = datatable_handle->columns[index].data.buffer.bool_samples[dbs];
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count -= 1;

            /* copy tmp data buffer samples to working data buffer samples */
            for(int dbs = 0; dbs < datatable_handle->columns[index].data.samples_count; dbs++) {
                datatable_handle->columns[index].data.buffer.bool_samples[dbs] = tmp_bool_samples[dbs];
            }

            /* free-up memory */
            if(tmp_bool_samples) {
                free(tmp_bool_samples);
            }

            break;
        case DATATABLE_COLUMN_DATA_FLOAT:
            datatable_float_column_data_type_t* tmp_float_samples;

            /* validate memory availability for data-table tmp data buffer samples */
            tmp_float_samples = (datatable_float_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_float_column_data_type_t));
            ESP_RETURN_ON_FALSE(tmp_float_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table temporary float column data buffer, data-table fifo column data buffer failed");

            /* copy working samples to tmp data buffer samples, skip the first data buffer sample */
            for(int dbs = 1; dbs < dt_data_buffer_size; dbs++) {
                tmp_float_samples[dbs - 1] = datatable_handle->columns[index].data.buffer.float_samples[dbs];
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count -= 1;

            /* copy tmp data buffer samples to working data buffer samples */
            for(int dbs = 0; dbs < datatable_handle->columns[index].data.samples_count; dbs++) {
                datatable_handle->columns[index].data.buffer.float_samples[dbs] = tmp_float_samples[dbs];
            }

            /* free-up memory */
            if(tmp_float_samples) {
                free(tmp_float_samples);
            }

            break;
        case DATATABLE_COLUMN_DATA_INT16:
            datatable_int16_column_data_type_t* tmp_int16_samples;

            /* validate memory availability for data-table tmp data buffer samples */
            tmp_int16_samples = (datatable_int16_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_int16_column_data_type_t));
            ESP_RETURN_ON_FALSE(tmp_int16_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table temporary int16 column data buffer, data-table fifo column data buffer failed");

            /* copy working samples to tmp data buffer samples, skip the first data buffer sample */
            for(int dbs = 1; dbs < dt_data_buffer_size; dbs++) {
                tmp_int16_samples[dbs - 1] = datatable_handle->columns[index].data.buffer.int16_samples[dbs];
            }

            /* adjust data buffer samples count attribute */
            datatable_handle->columns[index].data.samples_count -= 1;

            /* copy tmp data buffer samples to working data buffer samples */
            for(int dbs = 0; dbs < datatable_handle->columns[index].data.samples_count; dbs++) {
                datatable_handle->columns[index].data.buffer.int16_samples[dbs] = tmp_int16_samples[dbs];
            }

            /* free-up memory */
            if(tmp_int16_samples) {
                free(tmp_int16_samples);
            }

            break;
    }
    
    return ESP_OK;
}

/**
 * @brief Pops the first data-table data buffers for configured columns and shifts the index remaining 
 * samples by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param datatable_handle Data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_fifo_data_buffers(datatable_handle_t datatable_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* handle each data-table column */
    for(uint8_t ci = 0; ci <= datatable_handle->columns_index; ci++) {
        /* pop and shift data buffer stack */
        ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_handle, ci),TAG, "fifo column data buffer for data-table fifo column data buffers failed" );
    }

    return ESP_OK;
}

/**
 * @brief Processes data-table vector data-type data buffer samples on the stack by column based on the colunm index provided.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] index Data-table column index to process.
 * @param[out] value_uc Data-table column data buffer processed u-component value.
 * @param[out] value_vc Data-table column data buffer processed v-component value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_vector_data_buffer(datatable_handle_t datatable_handle, const uint8_t index, float *value_uc, float *value_vc, time_t *value_ts) {
    double tmp_ew_vector    = NAN;
    double tmp_ns_vector    = NAN;
    double tmp_ew_avg       = NAN;
    double tmp_ns_avg       = NAN;
    double tmp_atan2_uc     = NAN;
    double tmp_uc_avg       = NAN;
    double tmp_uc_value     = NAN;
    double tmp_vc_value     = NAN;
    time_t tmp_ts_value     = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range for process vector data buffer failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_VECTOR, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process vector data buffer failed" );

    // validate number of appended samples against expected number of samples
    if(datatable_handle->columns[index].data.samples_count != datatable_handle->columns[index].data.samples_size) {
        /* set default data */
        *value_uc = tmp_uc_value;
        *value_vc = tmp_vc_value;

        return ESP_ERR_INVALID_SIZE;
    }

    /* process data buffer by process type */
    switch(datatable_handle->columns[index].process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            *value_uc = datatable_handle->columns[index].data.buffer.vector_samples[0].value_uc;
            *value_vc = datatable_handle->columns[index].data.buffer.vector_samples[0].value_vc;
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            /*
            * http://www.webmet.com/met_monitoring/622.html
            * https://www.scadacore.com/2014/12/19/average-wind-direction-and-wind-speed/
            */
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_ew_vector = sin( datalogger_degrees_to_radians(datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc) ) * datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                    tmp_ns_vector = cos( datalogger_degrees_to_radians(datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc) ) * datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                } else {
                    tmp_ew_vector += sin( datalogger_degrees_to_radians(datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc) ) * datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                    tmp_ns_vector += cos( datalogger_degrees_to_radians(datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc) ) * datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                }
                //ESP_LOGW(TAG, "datatable_process_float_data_buffer(column-index: %u) data-sum: %.2f", column_index, tmp_data);
            }

            // average in radians
            tmp_ew_avg = (tmp_ew_vector / datatable_handle->columns[index].data.samples_count) * -1;
            tmp_ns_avg = (tmp_ns_vector / datatable_handle->columns[index].data.samples_count) * -1;

            // average u-component in degrees
            tmp_atan2_uc = atan2(tmp_ew_avg, tmp_ns_avg);
            tmp_uc_avg = datalogger_radians_to_degrees(tmp_atan2_uc);

            // apply correction as specified in webmet.com webpage http://www.webmet.com/met_monitoring/622.html
            if(tmp_uc_avg > 180.0) {
                tmp_uc_avg -= 180.0;
            } else if(tmp_uc_avg < 180.0) {
                tmp_uc_avg += 180.0;
            }
            *value_uc = tmp_uc_avg;

            // average v-component
            *value_vc = sqrt((tmp_ew_avg * tmp_ew_avg) + (tmp_ns_avg * tmp_ns_avg));

            //ESP_LOGW(TAG, "datatable_process_float_data_buffer(column-index: %u) data-count: %u data-avg: %.2f", index, datatable_handle->columns[index].data.samples_count, *value);
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                    tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                } else {
                    if(datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc < tmp_vc_value) {
                        tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                        tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                    tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                } else {
                    if(datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc > tmp_vc_value) {
                        tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                        tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_ts_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_ts;
                    tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                    tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                } else {
                    if(datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc < tmp_vc_value) {
                        tmp_ts_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_ts;
                        tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                        tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            *value_ts = tmp_ts_value;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_ts_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_ts;
                    tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                    tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                } else {
                    if(datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc > tmp_vc_value) {
                        tmp_ts_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_ts;
                        tmp_uc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_uc;
                        tmp_vc_value = datatable_handle->columns[index].data.buffer.vector_samples[s].value_vc;
                    }
                }
            }
            *value_uc = tmp_uc_value;
            *value_vc = tmp_vc_value;
            *value_ts = tmp_ts_value;
            break;
    }
 
    return ESP_OK;
}

/**
 * @brief Processes data-table boolean data-type data buffer samples on the stack by column based on the colunm index provided.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] index Data-table column index to process.
 * @param[out] value Data-table column data buffer processed value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_bool_data_buffer(datatable_handle_t datatable_handle, const uint8_t index, bool *value) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range for process bool data buffer failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_BOOL, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process bool data buffer failed" );

    /* validate column process-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP, ESP_ERR_INVALID_ARG, TAG, "column process-type is incorrect for process bool data buffer failed" );

    // validate number of appended samples against expected number of samples
    if(datatable_handle->columns[index].data.samples_count != datatable_handle->columns[index].data.samples_size) {
        /* set default data */
        *value = false;

        return ESP_ERR_INVALID_SIZE;
    }

    /* set value from sample */
    *value = datatable_handle->columns[index].data.buffer.float_samples[0].value;
 
    return ESP_OK;
}

/**
 * @brief Processes data-table float data-type data buffer samples on the stack by column based on the colunm index provided.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] index Data-table column index to process.
 * @param[out] value Data-table column data buffer processed value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_float_data_buffer(datatable_handle_t datatable_handle, const uint8_t index, float *value, time_t *value_ts) {
    float tmp_value = NAN;
    time_t tmp_ts = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range for process float data buffer failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_FLOAT, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process float data buffer failed" );

    // validate number of appended samples against expected number of samples
    if(datatable_handle->columns[index].data.samples_count != datatable_handle->columns[index].data.samples_size) {
        /* set default data */
        *value = tmp_value;
        *value_ts = tmp_ts;

        return ESP_ERR_INVALID_SIZE;
    }

    /* process data buffer by process type */
    switch(datatable_handle->columns[index].process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            *value = datatable_handle->columns[index].data.buffer.float_samples[0].value;
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                } else {
                    tmp_value += datatable_handle->columns[index].data.buffer.float_samples[s].value;
                }
                //ESP_LOGW(TAG, "datatable_process_float_data_buffer(column-index: %u) data-sum: %.2f", column_index, tmp_data);
            }
            *value = tmp_value / datatable_handle->columns[index].data.samples_count;
            *value_ts = tmp_ts;
            ESP_LOGW(TAG, "datatable_process_float_data_buffer(column-index: %u) data-count: %u data-avg: %.2f", index, datatable_handle->columns[index].data.samples_count, *value);
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                } else {
                    if(datatable_handle->columns[index].data.buffer.float_samples[s].value < tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                } else {
                    if(datatable_handle->columns[index].data.buffer.float_samples[s].value > tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                    tmp_ts = datatable_handle->columns[index].data.buffer.float_samples[s].value_ts;
                } else {
                    if(datatable_handle->columns[index].data.buffer.float_samples[s].value < tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                        tmp_ts = datatable_handle->columns[index].data.buffer.float_samples[s].value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                    tmp_ts = datatable_handle->columns[index].data.buffer.float_samples[s].value_ts;
                } else {
                    if(datatable_handle->columns[index].data.buffer.float_samples[s].value > tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.float_samples[s].value;
                        tmp_ts = datatable_handle->columns[index].data.buffer.float_samples[s].value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
    }
 
    return ESP_OK;
}

/**
 * @brief Processes data-table int16 data-type data buffer samples on the stack by column based on the colunm index provided.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] index Data-table column index to process.
 * @param[out] value Data-table column data buffer processed value.
 * @param[out] value_ts Data-table column data buffer processed timestamp for process value.  This parameter is for timestamp process types, otherwise it is NULL.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_process_int16_data_buffer(datatable_handle_t datatable_handle, const uint8_t index, int16_t *value, time_t *value_ts) {
    int16_t tmp_value = 0;
    time_t  tmp_ts    = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range for process int16 data buffer failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_INT16, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect for process int16 data buffer failed");

    // validate number of appended samples against expected number of samples
    if(datatable_handle->columns[index].data.samples_count != datatable_handle->columns[index].data.samples_size) {
        /* set default data */
        *value    = tmp_value;
        *value_ts = tmp_ts;

        return ESP_ERR_INVALID_SIZE;
    }

    /* process data buffer by process type */
    switch(datatable_handle->columns[index].process_type) {
        case DATATABLE_COLUMN_PROCESS_SMP:
            *value = datatable_handle->columns[index].data.buffer.int16_samples[0].value;
            break;
        case DATATABLE_COLUMN_PROCESS_AVG:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                } else {
                    tmp_value += datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                }
                //ESP_LOGW(TAG, "datatable_process_int16_data_buffer(column-index: %u) data-sum: %u", column_index, tmp_data);
            }
            *value = tmp_value / datatable_handle->columns[index].data.samples_count;
            *value_ts = tmp_ts;
            ESP_LOGW(TAG, "datatable_process_int16_data_buffer(column-index: %u) data-count: %u data-avg: %u", index, datatable_handle->columns[index].data.samples_count, *value);
            break;
        case DATATABLE_COLUMN_PROCESS_MIN:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                } else {
                    if(datatable_handle->columns[index].data.buffer.int16_samples[s].value < tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                } else {
                    if(datatable_handle->columns[index].data.buffer.int16_samples[s].value > tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MIN_TS:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                    tmp_ts = datatable_handle->columns[index].data.buffer.int16_samples[s].value_ts;
                } else {
                    if(datatable_handle->columns[index].data.buffer.int16_samples[s].value < tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                        tmp_ts = datatable_handle->columns[index].data.buffer.int16_samples[s].value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
            break;
        case DATATABLE_COLUMN_PROCESS_MAX_TS:
            for(uint16_t s = 0; s < datatable_handle->columns[index].data.samples_count; s++) {
                if(s == 0) {
                    tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                    tmp_ts = datatable_handle->columns[index].data.buffer.int16_samples[s].value_ts;
                } else {
                    if(datatable_handle->columns[index].data.buffer.int16_samples[s].value > tmp_value) {
                        tmp_value = datatable_handle->columns[index].data.buffer.int16_samples[s].value;
                        tmp_ts = datatable_handle->columns[index].data.buffer.int16_samples[s].value_ts;
                    }
                }
            }
            *value = tmp_value;
            *value_ts = tmp_ts;
    }
 
    return ESP_OK;
}

esp_err_t datatable_new(const datatable_config_t *datatable_config, datatable_handle_t *datatable_handle) {
    esp_err_t          ret = ESP_OK;
    int64_t            interval_delta = 0;
    datatable_column_t dt_id_column;
    datatable_column_t dt_ts_column;
    datatable_handle_t out_handle = NULL;

    /**
     * validate data-table sampling and processing type, period and offset arguments
     * 
     * the sampling rate must be lower than the processing interval. as an example,
     * a 5-sec sampling rate with a 1-min processing interval would trigger processing
     * of the row columns once every minute and would process 12 samples based on 
     * the desired processing type (i.e. avg, min, max).  if the processing type is
     * configured to smp a sample would be updated every sampling interval and latest
     * value would be recorded during the processing interval.
     */

    /* validate data-table arguments */
    ESP_GOTO_ON_FALSE( (datatable_config->columns_size > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table columns size cannot be 0, new data-table handle failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->rows_size > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table rows size cannot be 0, new data-table handle failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->sampling_config.interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table sampling interval period cannot be 0, new data-table failed" );
    ESP_GOTO_ON_FALSE( (datatable_config->sampling_config.interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period cannot be 0, new data-table failed" );

    /* validate sampling and processing interval periods */
    interval_delta = datalogger_normalize_interval_to_sec(datatable_config->processing_config.interval_type, datatable_config->processing_config.interval_period) - 
                     datalogger_normalize_interval_to_sec(datatable_config->sampling_config.interval_type, datatable_config->sampling_config.interval_period); 
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period must be larger than the sampling interval period, new data-table handle failed" );

    /* validate sampling period and offset intervals */
    interval_delta = datalogger_normalize_interval_to_sec(datatable_config->sampling_config.interval_type, datatable_config->sampling_config.interval_period) - 
                     datalogger_normalize_interval_to_sec(datatable_config->sampling_config.interval_type, datatable_config->sampling_config.interval_offset); 
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period must be larger than the sampling interval offset, new data-table handle failed" );
    
    /* validate processing period and offset intervals */
    interval_delta = datalogger_normalize_interval_to_sec(datatable_config->processing_config.interval_type, datatable_config->processing_config.interval_period) - 
                     datalogger_normalize_interval_to_sec(datatable_config->processing_config.interval_type, datatable_config->processing_config.interval_offset); 
    ESP_GOTO_ON_FALSE((interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "data-table processing interval period must be larger than the processing interval offset, new data-table handle failed" );

    /* validate memory availability for data-table handle */
    out_handle = (datatable_handle_t)calloc(1, sizeof(datatable_handle_t));
    ESP_GOTO_ON_FALSE( out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for data-table handle, new data-table handle" );

    /* initialize data-table state object */
    strcpy(out_handle->name, datatable_config->name);
    out_handle->columns_index               = 0;
    out_handle->columns_size                = datatable_config->columns_size + 2; // add record id and timestamp columns
    out_handle->rows_index                  = 0;
    out_handle->rows_count                  = 0;
    out_handle->rows_size                   = datatable_config->rows_size;
    out_handle->sampling_count              = 0;
    out_handle->sampling_interval_type      = datatable_config->sampling_config.interval_type;
    out_handle->sampling_interval_period    = datatable_config->sampling_config.interval_period;
    out_handle->sampling_interval_offset    = datatable_config->sampling_config.interval_offset;
    out_handle->processing_interval_type    = datatable_config->processing_config.interval_type;
    out_handle->processing_interval_period  = datatable_config->processing_config.interval_period;
    out_handle->processing_interval_offset  = datatable_config->processing_config.interval_offset;
    out_handle->data_storage_type           = datatable_config->data_storage_type;
    out_handle->record_id                   = 0;

    /* define default data-table columns (record id and timestamp) */
    dt_id_column.index                      = out_handle->columns_index;   // 0
    dt_id_column.data_type                  = DATATABLE_COLUMN_DATA_ID;
    dt_id_column.process_type               = DATATABLE_COLUMN_PROCESS_SMP;
    dt_id_column.data.samples_size          = 0;
    dt_id_column.data.samples_count         = 0;
    strcpy(dt_id_column.names[0].name, "Record ID");
    out_handle->columns_index               += 1; // increment column index
    dt_ts_column.index                      = out_handle->columns_index; // 1
    dt_ts_column.data_type                  = DATATABLE_COLUMN_DATA_TS;
    dt_ts_column.process_type               = DATATABLE_COLUMN_PROCESS_SMP;
    dt_ts_column.data.samples_size          = 0;
    dt_ts_column.data.samples_count         = 0;
    strcpy(dt_ts_column.names[0].name, "TS");

    // initialize task-schedule configuration - data-table sampling task system clock synchronization
    task_schedule_config_t   dt_ts_cfg = {
        .interval_type      = out_handle->sampling_interval_type,
        .interval_period    = out_handle->sampling_interval_period,
        .interval_offset    = out_handle->sampling_interval_offset
    };

    // create a new task-schedule handle - data-table sampling task system clock synchronization
    ESP_GOTO_ON_ERROR( task_schedule_new(&dt_ts_cfg, &out_handle->sampling_ts_handle), 
                        err_out_handle, TAG, "new task-schedule for new data-table handle failed" );

    // initialize time-into-interval configuration - data-table column data buffer processing task system clock synchronization
    time_into_interval_config_t dt_tii_cfg = {
        .interval_type      = out_handle->processing_interval_type,
        .interval_period    = out_handle->processing_interval_period,
        .interval_offset    = out_handle->processing_interval_offset
    };

    // create a new time-into-interval handle - data-table column data buffer processing task system clock synchronization
    ESP_GOTO_ON_ERROR( time_into_interval_new(&dt_tii_cfg, &out_handle->processing_tti_handle), 
                        err_out_handle, TAG, "new time-into-interval for new data-table handle failed" );

    /* validate memory availability for default data-table columns */
    out_handle->columns = (datatable_column_t*)malloc(out_handle->columns_size * sizeof(datatable_column_t));
    ESP_GOTO_ON_FALSE( out_handle->columns, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table columns, new data-table handle failed" );

    /* validate memory availability for default data-table rows */
    out_handle->rows = (datatable_row_t*)malloc(out_handle->rows_size * sizeof(datatable_row_t));
    ESP_GOTO_ON_FALSE( out_handle->rows, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for data-table rows, new data-table handle failed" );

    /* append default data-table columns (record id and timestamp) to state object */
    out_handle->columns[0] = dt_id_column;
    out_handle->columns[1] = dt_ts_column;

    /* set output handle */
    *datatable_handle = out_handle;

    return ESP_OK;

    err_out_handle:
        free(out_handle);
    err:
        return ret;
}

/**
 * @brief Appends a vector based data-type column to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name_uc Textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc Textual name of the data-table column to be added for vector v-component.
 * @param[in] process_type Data processing type of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_vector_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, const datatable_column_process_types_t process_type, uint8_t *index) {
    static char col_name_0[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";  // u-component
    static char col_name_1[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";  // v-component
    static char col_name_2[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";  // timestamp
    static char pt_str[10]          = "";
    static char us_str[2]           = "_";
    uint16_t    dt_data_buffer_size = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate column name length */
    ESP_RETURN_ON_FALSE( (strlen(name_uc) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, TAG, "column name_uc is too long, data-table add vector column failed" );
    ESP_RETURN_ON_FALSE( (strlen(name_vc) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, TAG, "column name_vc is too long, data-table add vector column failed" );

    /* validate columns size */
    ESP_RETURN_ON_FALSE((datatable_handle->columns_index + 1 <= datatable_handle->columns_size), ESP_ERR_INVALID_SIZE, TAG, "unable to add columns to data-table for add vector column");

    /* validate data-table column data buffer size by processing type */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size for data-table column data buffer */
        dt_data_buffer_size = 1;
    } else {
        /* determine size of samples for data-table column data buffer */
        ESP_RETURN_ON_ERROR( datatable_get_column_data_buffer_size(datatable_handle, &dt_data_buffer_size),TAG, "get column data buffer size for add vector column failed" );
    }

    /* validate memory availability for default data-table column data buffer */
    datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.vector_samples = (datatable_vector_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_vector_column_data_type_t));
    ESP_RETURN_ON_FALSE(datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.vector_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table column data buffer for add vector column");

    /* validate processing type and set column name(s) */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP || process_type == DATATABLE_COLUMN_PROCESS_AVG || 
       process_type == DATATABLE_COLUMN_PROCESS_MIN || process_type == DATATABLE_COLUMN_PROCESS_MAX) {
        /* set column base name */
        strcpy(col_name_0, name_uc);
        strcat(col_name_0, us_str);
        strcpy(col_name_1, name_vc);
        strcat(col_name_1, us_str);

        /* update column name by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_1, pt_str);

        /* set column names */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[1].name, col_name_1);
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
        /* set column base name */
        strcpy(col_name_0, name_uc);
        strcat(col_name_0, us_str);
        strcpy(col_name_1, name_vc);
        strcat(col_name_1, us_str);
        strcpy(col_name_2, name_vc);
        strcat(col_name_2, us_str);

        /* update column name by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_1, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_2, pt_str);

        /* set column names */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[1].name, col_name_1);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[2].name, col_name_2);
    } else {
        /* if we landed here, this data-type doesn't support the process-type provided in the arguments */
        ESP_RETURN_ON_FALSE( false, ESP_ERR_NOT_SUPPORTED, TAG, "data-table column process-type is not supported float data-type, data-table add float column failed");
    }

    /* initialize data-table column */
    datatable_handle->columns[datatable_handle->columns_index + 1].index                        = datatable_handle->columns_index + 1;
    datatable_handle->columns[datatable_handle->columns_index + 1].data_type                    = DATATABLE_COLUMN_DATA_VECTOR;
    datatable_handle->columns[datatable_handle->columns_index + 1].process_type                 = process_type;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_count           = 0;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_size            = dt_data_buffer_size;

    /* increment data-table columns index and set output parameter */
    datatable_handle->columns_index += 1;
    *index = datatable_handle->columns_index;

    return ESP_OK;
}

esp_err_t datatable_add_vector_smp_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append vector sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_handle, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_SMP, index), TAG, "add vector column for add vector sample process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_avg_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_handle, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_AVG, index), TAG, "add vector column for add vector average process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_min_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_handle, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MIN, index), TAG, "add vector column for add vector minimum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_max_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_handle, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MAX, index), TAG, "add vector column for add vector maximum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_min_ts_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_handle, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MIN_TS, index), TAG, "add vector column for add vector minimum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_vector_max_ts_column(datatable_handle_t datatable_handle, const char *name_uc, const char *name_vc, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append vector average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_vector_column(datatable_handle, name_uc, name_vc, DATATABLE_COLUMN_PROCESS_MAX_TS, index), TAG, "add vector column for add vector maximum with timestamp process-type column failed");

    return ESP_OK;
}

/**
 * @brief Appends a bool based data-type column to the data-table.  This column data-type supports sampling only.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_bool_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    static char col_name_0[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";
    static char pt_str[10]          = "";
    static char us_str[2]           = "_";
    uint16_t    dt_data_buffer_size = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate column name length */
    ESP_RETURN_ON_FALSE( (strlen(name) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, TAG, "column name is too long, data-table add bool column failed" );

    /* validate columns size */
    ESP_RETURN_ON_FALSE( (datatable_handle->columns_index + 1 < datatable_handle->columns_size), ESP_ERR_INVALID_SIZE, TAG, "unable to add columns to data-table for add bool column" );

    /* static size for data-table column data buffer with bool data-type */
    dt_data_buffer_size = 1;

    /* validate memory availability for default data-table column data buffer */
    datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.bool_samples = (datatable_bool_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_bool_column_data_type_t));
    ESP_RETURN_ON_FALSE( datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.bool_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table column data buffer for add bool column");

    /* set column base name */
    strcpy(col_name_0, name);
    strcat(col_name_0, us_str);

    /* update column name by appending processing type short string */
    strcpy(pt_str, datatable_process_type_to_short_string(DATATABLE_COLUMN_PROCESS_SMP));
    strcat(col_name_0, pt_str);
    
    /* initialize data-table column */
    strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
    datatable_handle->columns[datatable_handle->columns_index + 1].index                        = datatable_handle->columns_index + 1;
    datatable_handle->columns[datatable_handle->columns_index + 1].data_type                    = DATATABLE_COLUMN_DATA_BOOL;
    datatable_handle->columns[datatable_handle->columns_index + 1].process_type                 = DATATABLE_COLUMN_PROCESS_SMP;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_count           = 1;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_size            = dt_data_buffer_size;

    /* increment data-table columns index and set output parameter */
    datatable_handle->columns_index += 1;
    *index = datatable_handle->columns_index;

    return ESP_OK;
}

esp_err_t datatable_add_bool_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append bool sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_bool_column(datatable_handle, name, index), TAG, "add bool column for add bool sample process-type column failed");

    return ESP_OK;
}

/**
 * @brief Appends a float based data-type column to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[in] process_type Data processing type of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_float_column(datatable_handle_t datatable_handle, const char *name, const datatable_column_process_types_t process_type, uint8_t *index) {
    static char col_name_0[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";
    static char col_name_1[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";
    static char pt_str[10]       = "";
    static char us_str[2]        = "_";
    uint16_t dt_data_buffer_size = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate column name length */
    ESP_RETURN_ON_FALSE( (strlen(name) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, TAG, "column name is too long, data-table add float column failed" );

    /* validate columns size */
    ESP_RETURN_ON_FALSE( (datatable_handle->columns_index + 1 < datatable_handle->columns_size), ESP_ERR_INVALID_SIZE, TAG, "unable to add columns to data-table for add float column" );

    /* validate data-table column data buffer size by processing type */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size for data-table column data buffer */
        dt_data_buffer_size = 1;
    } else {
        /* determine size of samples for data-table column data buffer */
        ESP_RETURN_ON_ERROR( datatable_get_column_data_buffer_size(datatable_handle, &dt_data_buffer_size),TAG, "get column data buffer size for add float column failed" );
    }

    /* validate memory availability for default data-table column data buffer */
    datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.float_samples = (datatable_float_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_float_column_data_type_t));
    ESP_RETURN_ON_FALSE( datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.float_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table column data buffer for add float column");

    /* validate processing type and set column name(s) */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP || process_type == DATATABLE_COLUMN_PROCESS_AVG || 
       process_type == DATATABLE_COLUMN_PROCESS_MIN || process_type == DATATABLE_COLUMN_PROCESS_MAX) {
        /* set column base name */
        strcpy(col_name_0, name);
        strcat(col_name_0, us_str);

        /* update column name by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);

        /* set column name */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MIN_TS) {
        /* set column base names */
        strcpy(col_name_0, name);
        strcat(col_name_0, us_str);
        strcpy(col_name_1, name);
        strcat(col_name_1, us_str);

        /* update column timestamp names by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_1, pt_str);

        /* set column names */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[1].name, col_name_1);
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
        /* set column base names */
        strcpy(col_name_0, name);
        strcat(col_name_0, us_str);
        strcpy(col_name_1, name);
        strcat(col_name_1, us_str);

        /* update column timestamp names by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_1, pt_str);

        /* set column names */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[1].name, col_name_1);
    } else {
        /* if we landed here, this data-type doesn't support the process-type provided in the arguments */
        ESP_RETURN_ON_FALSE( false, ESP_ERR_NOT_SUPPORTED, TAG, "data-table column process-type is not supported float data-type, data-table add float column failed");
    }

    //ESP_LOGW(TAG, "datatable_add_float_column->dt_data_buffer_size: %u", dt_data_buffer_size);

    /* initialize data-table column */
    datatable_handle->columns[datatable_handle->columns_index + 1].index                        = datatable_handle->columns_index + 1;
    datatable_handle->columns[datatable_handle->columns_index + 1].data_type                    = DATATABLE_COLUMN_DATA_FLOAT;
    datatable_handle->columns[datatable_handle->columns_index + 1].process_type                 = process_type;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_count           = 0;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_size            = dt_data_buffer_size;

    /* increment data-table columns index and set output parameter */
    datatable_handle->columns_index += 1;
    *index = datatable_handle->columns_index;

    return ESP_OK;
}

esp_err_t datatable_add_float_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append float sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_SMP, index), TAG, "add float column for add float sample process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_avg_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append float average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_AVG, index), TAG, "add float column for add float average process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_min_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append float minimum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MIN, index), TAG, "add float column for add float minimum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_max_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append float maximum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MAX, index), TAG, "add float column for add float maximum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_min_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append float minimum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MIN_TS, index), TAG, "add float column for add float minimum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_float_max_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append float maximum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_float_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MAX_TS, index), TAG, "add float column for add float maximum with timestamp process-type column failed");

    return ESP_OK;
}

/**
 * @brief Appends a int16 based data-type column to the data-table.
 * 
 * @param[in] datatable_handle Data-table handle.
 * @param[in] name Textual name of the data-table column to be added.
 * @param[in] process_type Data processing type of the data-table column to be added.
 * @param[out] index Index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t datatable_add_int16_column(datatable_handle_t datatable_handle, const char *name, const datatable_column_process_types_t process_type, uint8_t *index) {
    static char col_name_0[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";
    static char col_name_1[DATATABLE_COLUMN_NAME_MAX_SIZE] = "";
    static char pt_str[10]       = "";
    static char us_str[2]        = "_";
    uint16_t dt_data_buffer_size = 0;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate column name length */
    ESP_RETURN_ON_FALSE( (strlen(name) <= DATATABLE_COLUMN_NAME_SIZE), ESP_ERR_INVALID_ARG, TAG, "column name is too long, data-table add int16 column failed" );

    /* validate columns size */
    ESP_RETURN_ON_FALSE( (datatable_handle->columns_index + 1 <= datatable_handle->columns_size), ESP_ERR_INVALID_SIZE, TAG, "unable to add columns to data-table for add int16 column" );

    /* validate data-table column data buffer size by processing type */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        /* static size for data-table column data buffer */
        dt_data_buffer_size = 1;
    } else {
        /* determine size of samples for data-table column data buffer */
        ESP_RETURN_ON_ERROR( datatable_get_column_data_buffer_size(datatable_handle, &dt_data_buffer_size),TAG, "get column data buffer size for add int16 column failed" );
    }

    /* validate memory availability for default data-table column data buffer */
    datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.int16_samples = (datatable_int16_column_data_type_t*)malloc(dt_data_buffer_size * sizeof(datatable_int16_column_data_type_t));
    ESP_RETURN_ON_FALSE( datatable_handle->columns[datatable_handle->columns_index + 1].data.buffer.int16_samples, ESP_ERR_NO_MEM, TAG, "no memory for data-table column data buffer for add int16 column");

    /* validate processing type and set column name(s) */
    if(process_type == DATATABLE_COLUMN_PROCESS_SMP || process_type == DATATABLE_COLUMN_PROCESS_AVG || 
       process_type == DATATABLE_COLUMN_PROCESS_MIN || process_type == DATATABLE_COLUMN_PROCESS_MAX) {
        /* set column base names */
        strcpy(col_name_0, name);
        strcat(col_name_0, us_str);

        /* update column name by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);

        /* set column name */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MIN_TS) {
        /* set column base names */
        strcpy(col_name_0, name);
        strcat(col_name_0, us_str);
        strcpy(col_name_1, name);
        strcat(col_name_1, us_str);

        /* update column timestamp names by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_1, pt_str);

        /* set column names */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[1].name, col_name_1);
    } else if(process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
        /* set column base names */
        strcpy(col_name_0, name);
        strcat(col_name_0, us_str);
        strcpy(col_name_1, name);
        strcat(col_name_1, us_str);

        /* update column timestamp names by appending processing type short string */
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_0, pt_str);
        strcpy(pt_str, datatable_process_type_to_short_string(process_type));
        strcat(col_name_1, pt_str);

        /* set column names */
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[0].name, col_name_0);
        strcpy(datatable_handle->columns[datatable_handle->columns_index + 1].names[1].name, col_name_1);
    } else {
        /* if we landed here, this data-type doesn't support the process-type provided in the arguments */
        ESP_RETURN_ON_FALSE( false, ESP_ERR_NOT_SUPPORTED, TAG, "data-table column process-type is not supported int16 data-type, data-table add int16 column failed");
    }

    /* initialize data-table column */
    datatable_handle->columns[datatable_handle->columns_index + 1].index                        = datatable_handle->columns_index + 1;
    datatable_handle->columns[datatable_handle->columns_index + 1].data_type                    = DATATABLE_COLUMN_DATA_INT16;
    datatable_handle->columns[datatable_handle->columns_index + 1].process_type                 = process_type;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_count           = 0;
    datatable_handle->columns[datatable_handle->columns_index + 1].data.samples_size            = dt_data_buffer_size;

    /* increment data-table columns index and set output parameter */
    datatable_handle->columns_index += 1;
    *index = datatable_handle->columns_index;

    return ESP_OK;
}

esp_err_t datatable_add_int16_smp_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append int16 sample column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_SMP, index), TAG, "add int16 column for add int16 sample process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_avg_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append int16 average column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_AVG, index), TAG, "add int16 column for add int16 average process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_min_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append int16 minimum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MIN, index), TAG, "add int16 column for add int16 minimum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_max_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append int16 maximum column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MAX, index), TAG, "add int16 column for add int16 maximum process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_min_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append int16 minimum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MIN_TS, index), TAG, "add int16 column for add int16 minimum with timestamp process-type column failed");

    return ESP_OK;
}

esp_err_t datatable_add_int16_max_ts_column(datatable_handle_t datatable_handle, const char *name, uint8_t *index) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* append int16 maximum with timestamp column to data-table */
    ESP_RETURN_ON_ERROR( datatable_add_int16_column(datatable_handle, name, DATATABLE_COLUMN_PROCESS_MAX_TS, index), TAG, "add int16 column for add int16 maximum with timestamp process-type column failed");

    return ESP_OK;
}




esp_err_t datatable_get_columns_count(datatable_handle_t datatable_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* set output parameter */
    *count = datatable_handle->columns_index + 1;

    return ESP_OK;
}

esp_err_t datatable_get_rows_count(datatable_handle_t datatable_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* set output parameter */
    *count = datatable_handle->rows_index + 1;

    return ESP_OK;
}

esp_err_t datatable_get_column(datatable_handle_t datatable_handle, const uint8_t index, datatable_column_t *column) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE( (index < datatable_handle->columns_size), ESP_ERR_INVALID_ARG, TAG, "index is out of range, get column failed" );

    /* set output parameter */
    *column = datatable_handle->columns[index];

    return ESP_OK;
}

esp_err_t datatable_get_row(datatable_handle_t datatable_handle, const uint8_t index, datatable_row_t *row) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE( (index < datatable_handle->rows_size), ESP_ERR_INVALID_ARG, TAG, "index is out of range, get row failed" );

    /* set output parameter */
    *row = datatable_handle->rows[index];

    return ESP_OK;
}

esp_err_t datatable_push_vector_sample(datatable_handle_t datatable_handle, const uint8_t index, const float value_uc, const float value_vc) {
    bool dt_is_full;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the data-table is full */
    ESP_RETURN_ON_ERROR( datatable_is_full(datatable_handle, &dt_is_full), TAG, "data-table is full, process data failed" );

    /* handle data storage scheme */
    if(dt_is_full == true) {
        switch(datatable_handle->data_storage_type) {
            case DATATABLE_DATA_STORAGE_MEMORY_RING:
                // pop first row and push remaining rows to top of stack
                ESP_RETURN_ON_ERROR( datatable_fifo_rows(datatable_handle), TAG, "unable fifo data-table rows, process data failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_RESET:
                // reset data-table by clearing rows
                ESP_RETURN_ON_ERROR( datatable_reset_rows(datatable_handle), TAG, "unable to reset data-table rows, process data failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_STOP:
                // stop processing samples by existing function
                return ESP_OK; // log as debug info
                break;
        }
    }

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range, push vector sample failed" );

    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_VECTOR, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push vector sample failed" );

    /* handle column process-type */
    if(datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        datatable_handle->columns[index].data.samples_count = 1;
        datatable_handle->columns[index].data.buffer.vector_samples[0].value_ts = datalogger_get_epoch_timestamp();
        datatable_handle->columns[index].data.buffer.vector_samples[0].value_uc = value_uc;
        datatable_handle->columns[index].data.buffer.vector_samples[0].value_vc = value_vc;
    } else {
        // validate data buffer samples count
        if(datatable_handle->columns[index].data.samples_count + 1 > datatable_handle->columns[index].data.samples_size) {
            // pop and shift data buffer by 1 sample (fifo)
            ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_handle, index), TAG, "unable to fifo column data buffer, push vector sample failed" );

            // samples count remains the same but append sample to column data buffer
            datatable_handle->columns[index].data.buffer.vector_samples[datatable_handle->columns[index].data.samples_count-1].value_ts = datalogger_get_epoch_timestamp();
            datatable_handle->columns[index].data.buffer.vector_samples[datatable_handle->columns[index].data.samples_count-1].value_uc = value_uc;
            datatable_handle->columns[index].data.buffer.vector_samples[datatable_handle->columns[index].data.samples_count-1].value_vc = value_vc;
        } else {
            // increment samples count and append sample to column data buffer
            datatable_handle->columns[index].data.samples_count += 1;
            datatable_handle->columns[index].data.buffer.vector_samples[datatable_handle->columns[index].data.samples_count-1].value_ts = datalogger_get_epoch_timestamp();
            datatable_handle->columns[index].data.buffer.vector_samples[datatable_handle->columns[index].data.samples_count-1].value_uc = value_uc;
            datatable_handle->columns[index].data.buffer.vector_samples[datatable_handle->columns[index].data.samples_count-1].value_vc = value_vc;
        }
    }

    return ESP_OK;
}

esp_err_t datatable_push_bool_sample(datatable_handle_t datatable_handle, const uint8_t index, const bool value) {
    bool dt_is_full;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the data-table is full */
    ESP_RETURN_ON_ERROR( datatable_is_full(datatable_handle, &dt_is_full), TAG, "data-table is full, push bool sample failed" );

    /* handle data storage scheme */
    if(dt_is_full == true) {
        switch(datatable_handle->data_storage_type) {
            case DATATABLE_DATA_STORAGE_MEMORY_RING:
                // pop first row and push remaining rows to top of stack
                ESP_RETURN_ON_ERROR( datatable_fifo_rows(datatable_handle), TAG, "unable to fifo data-table rows, push bool sample failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_RESET:
                // reset data-table by clearing rows
                ESP_RETURN_ON_ERROR( datatable_reset_rows(datatable_handle), TAG, "unable to reset data-table rows, push bool sample failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_STOP:
                // stop processing samples by existing function
                return ESP_OK; // log as debug info
                break;
        }
    }

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range, push bool sample failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_BOOL, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push bool sample failed" );

    /* validate column process-type */
    ESP_RETURN_ON_FALSE( datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP, ESP_ERR_INVALID_ARG, TAG, "column process-type is incorrect, push bool sample failed" );

    /* handle column process-type */
    datatable_handle->columns[index].data.samples_count = 1;
    datatable_handle->columns[index].data.buffer.bool_samples[0].value = value;

    ESP_LOGW(TAG, "datatable_push_bool_sample(column-index: %u) buffer-data(%d) %d", index, datatable_handle->columns[index].data.samples_count, datatable_handle->columns[index].data.buffer.bool_samples[datatable_handle->columns[index].data.samples_count-1].value);

    return ESP_OK;
}

esp_err_t datatable_push_float_sample(datatable_handle_t datatable_handle, const uint8_t index, const float value) {
    bool dt_is_full;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the data-table is full */
    ESP_RETURN_ON_ERROR( datatable_is_full(datatable_handle, &dt_is_full), TAG, "data-table is full, push float sample failed" );

    /* handle data storage scheme */
    if(dt_is_full == true) {
        switch(datatable_handle->data_storage_type) {
            case DATATABLE_DATA_STORAGE_MEMORY_RING:
                // pop first row and push remaining rows to top of stack
                ESP_RETURN_ON_ERROR( datatable_fifo_rows(datatable_handle), TAG, "unable to fifo data-table rows, push float sample failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_RESET:
                // reset data-table by clearing rows
                ESP_RETURN_ON_ERROR( datatable_reset_rows(datatable_handle), TAG, "unable to reset data-table rows, push float sample failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_STOP:
                // stop processing samples by existing function
                return ESP_OK; // log as debug info
                break;
        }
    }

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range, push float sample failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_FLOAT, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push float sample failed");

    /* handle column process-type */
    if(datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        datatable_handle->columns[index].data.samples_count = 1;
        datatable_handle->columns[index].data.buffer.float_samples[0].value_ts = datalogger_get_epoch_timestamp();
        datatable_handle->columns[index].data.buffer.float_samples[0].value = value;
    } else {
        // validate data buffer samples index
        if(datatable_handle->columns[index].data.samples_count + 1 > datatable_handle->columns[index].data.samples_size) {
            // pop and shift data buffer by 1 sample (fifo)
            ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_handle, index), TAG, "unable to fifo column data buffer, push float sample failed" );

            // samples count remains the same and append sample to column data buffer
            datatable_handle->columns[index].data.buffer.float_samples[datatable_handle->columns[index].data.samples_count-1].value_ts  = datalogger_get_epoch_timestamp();
            datatable_handle->columns[index].data.buffer.float_samples[datatable_handle->columns[index].data.samples_count-1].value     = value;
        } else {
            // increment samples count and append sample to column data buffer
            datatable_handle->columns[index].data.samples_count += 1;
            datatable_handle->columns[index].data.buffer.float_samples[datatable_handle->columns[index].data.samples_count-1].value_ts  = datalogger_get_epoch_timestamp();
            datatable_handle->columns[index].data.buffer.float_samples[datatable_handle->columns[index].data.samples_count-1].value     = value;
        }
    }

    ESP_LOGW(TAG, "datatable_push_float_sample(column-index: %u) buffer-data(%d) %.2f", index, datatable_handle->columns[index].data.samples_count, datatable_handle->columns[index].data.buffer.float_samples[datatable_handle->columns[index].data.samples_count-1].value);

    return ESP_OK;
}

esp_err_t datatable_push_int16_sample(datatable_handle_t datatable_handle, const uint8_t index, const int16_t value) {
    bool dt_is_full;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* check if the data-table is full */
    ESP_RETURN_ON_ERROR( datatable_is_full(datatable_handle, &dt_is_full), TAG, "data-table is full, push int16 sample failed" );

    /* handle data storage scheme */
    if(dt_is_full == true) {
        switch(datatable_handle->data_storage_type) {
            case DATATABLE_DATA_STORAGE_MEMORY_RING:
                // pop first row and push remaining rows to top of stack
                ESP_RETURN_ON_ERROR( datatable_fifo_rows(datatable_handle), TAG, "unable to fifo data-table rows, push int16 sample failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_RESET:
                // reset data-table by clearing rows
                ESP_RETURN_ON_ERROR( datatable_reset_rows(datatable_handle), TAG, "unable to reset data-table rows for push int16 sample failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_STOP:
                // stop processing samples by existing function
                return ESP_OK; // log as debug info
                break;
        }
    }

    /* check if the column exist by column index */
    ESP_RETURN_ON_ERROR( datatable_column_exist(datatable_handle, index), TAG, "column does not exist or index is out of range, push int16 sample failed" );
    
    /* validate column data-type */
    ESP_RETURN_ON_FALSE(datatable_handle->columns[index].data_type == DATATABLE_COLUMN_DATA_INT16, ESP_ERR_INVALID_ARG, TAG, "column data-type is incorrect, push int16 sample failed");

    /* handle column process-type */
    if(datatable_handle->columns[index].process_type == DATATABLE_COLUMN_PROCESS_SMP) {
        datatable_handle->columns[index].data.samples_count = 1;
        datatable_handle->columns[index].data.buffer.int16_samples[0].value_ts = datalogger_get_epoch_timestamp();
        datatable_handle->columns[index].data.buffer.int16_samples[0].value = value;
    } else {
        // validate data buffer samples index
        if(datatable_handle->columns[index].data.samples_count + 1 > datatable_handle->columns[index].data.samples_size) {
            // pop and shift data buffer by 1 sample (fifo)
            ESP_RETURN_ON_ERROR( datatable_fifo_data_buffer(datatable_handle, index), TAG, "unable to fifo column data buffer, push int16 sample failed" );

            // samples count remains the same but append sample to column data buffer
            datatable_handle->columns[index].data.buffer.int16_samples[datatable_handle->columns[index].data.samples_count-1].value_ts  = datalogger_get_epoch_timestamp();
            datatable_handle->columns[index].data.buffer.int16_samples[datatable_handle->columns[index].data.samples_count-1].value     = value;
        } else {
            // increment samples count and append sample to column data buffer
            datatable_handle->columns[index].data.samples_count += 1;
            datatable_handle->columns[index].data.buffer.int16_samples[datatable_handle->columns[index].data.samples_count-1].value_ts  = datalogger_get_epoch_timestamp();
            datatable_handle->columns[index].data.buffer.int16_samples[datatable_handle->columns[index].data.samples_count-1].value     = value;
        }
    }

    ESP_LOGW(TAG, "datatable_push_int16_sample(column-index: %u) buffer-data(%d) %u", index, datatable_handle->columns[index].data.samples_count, datatable_handle->columns[index].data.buffer.int16_samples[datatable_handle->columns[index].data.samples_count-1].value);

    return ESP_OK;
}

esp_err_t datatable_sampling_task_delay(datatable_handle_t datatable_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    /* delay data-table sampling task per sampling task-schedule handle */
    ESP_RETURN_ON_ERROR( task_schedule_delay(datatable_handle->sampling_ts_handle), TAG, "unable to delay task-schedule, data-table sampling task-schedule delay failed." );

    return ESP_OK;
}

esp_err_t datatable_process_samples(datatable_handle_t datatable_handle) {
    uint16_t    dt_data_buffer_size;
    bool        dt_is_full;

    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    // increment sampling count, TODO - see if this can be sync'd to the sampling task
    datatable_handle->sampling_count += 1;

    /* validate data-table processing interval event */
    if(time_into_interval(datatable_handle->processing_tti_handle) == false) {
        /* data-table processing time into interval condition failed, exit the function */
        return ESP_OK;
    }

    /* get number of data buffer samples */
    ESP_RETURN_ON_ERROR( datatable_get_column_data_buffer_size(datatable_handle, &dt_data_buffer_size), TAG, "get column data buffer size for data-table process samples failed" );

    /* validate data buffer samples processing condition */
    if(datatable_handle->sampling_count == dt_data_buffer_size) {
        /* reset sampling count */
        datatable_handle->sampling_count = 0;
    } else if(datatable_handle->sampling_count < dt_data_buffer_size) {
        /* insuficient number of data buffer samples at processing interval event, reset column data buffers */
        ESP_RETURN_ON_ERROR( datatable_reset_data_buffers(datatable_handle), TAG, "reset column data buffer for data-table process samples failed" );

        /* exit the function */
        return ESP_OK;
    } else { 
        /* reset sampling count */
        datatable_handle->sampling_count = 0;

        /* unsupported condition */
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* check if the data-table is full */
    ESP_RETURN_ON_ERROR( datatable_is_full(datatable_handle, &dt_is_full), TAG, "data-table is full for process samples failed" );

    /* handle data storage scheme */
    if(dt_is_full == true) {
        switch(datatable_handle->data_storage_type) {
            case DATATABLE_DATA_STORAGE_MEMORY_RING:
                // pop first row and push remaining rows to top of stack
                ESP_RETURN_ON_ERROR( datatable_fifo_rows(datatable_handle), TAG, "data-table fifo rows for process samples failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_RESET:
                // reset data-table by clearing rows
                ESP_RETURN_ON_ERROR( datatable_reset_rows(datatable_handle), TAG, "data-table reset rows for process samples failed" );
                break;
            case DATATABLE_DATA_STORAGE_MEMORY_STOP:
                // stop processing samples by exiting function
                return ESP_OK; // TODO log as debug info
                break;
        }
    }

    /* handle data-table row count and index */
    if(datatable_handle->rows_count == 0) {
        /* initialize data-table row count and index */
        datatable_handle->rows_count = 1;
        datatable_handle->rows_index = 0;
    } else {
        /* increment data-table row count and index */
        datatable_handle->rows_count += 1;
        datatable_handle->rows_index += 1;
    }

    /* validate memory availability for data-table row data columns */
    datatable_handle->rows[datatable_handle->rows_index].data_columns = (datatable_row_data_column_t*)malloc(datatable_handle->columns_size * sizeof(datatable_row_data_column_t));
    ESP_RETURN_ON_FALSE( datatable_handle->rows[datatable_handle->rows_index].data_columns, ESP_ERR_NO_MEM, TAG, "no memory for data-table row data columns, process samples failed");

    /* process data-table row data columns by data-type for each column */
    for(uint8_t i = 0; i <= datatable_handle->columns_index; i++) {
        // set data column row index, column index, data-type
        datatable_handle->rows[datatable_handle->rows_index].data_columns[i].column_index   = i;
        datatable_handle->rows[datatable_handle->rows_index].data_columns[i].row_index      = datatable_handle->rows_index;
        datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data_type      = datatable_handle->columns[i].data_type;

        // process data-table buffer data by data-type
        switch(datatable_handle->columns[i].data_type) {
            case DATATABLE_COLUMN_DATA_ID:
                datatable_handle->record_id++;
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.id_data.value = datatable_handle->record_id;
                break;
            case DATATABLE_COLUMN_DATA_TS:
                // unix utc time for timestamp value
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.ts_data.value = datalogger_get_epoch_timestamp(); // unix epoch timestamp
                break;
            case DATATABLE_COLUMN_DATA_VECTOR:
                float uc_value = 0; float vc_value = NAN; time_t value_ts = 0;
                ESP_RETURN_ON_ERROR( datatable_process_vector_data_buffer(datatable_handle, i, &uc_value, &vc_value, &value_ts), TAG, "process vector data buffer for data-table process samples failed" );
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.vector_data.value_ts = value_ts;
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.vector_data.value_uc = uc_value;
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.vector_data.value_vc = vc_value;
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_handle, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
            case DATATABLE_COLUMN_DATA_BOOL:
                bool bool_value = false;
                ESP_RETURN_ON_ERROR( datatable_process_bool_data_buffer(datatable_handle, i, &bool_value), TAG, "process bool data buffer for data-table process samples failed" );
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.bool_data.value = bool_value;
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_handle, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
            case DATATABLE_COLUMN_DATA_FLOAT:
                float float_value = NAN;
                ESP_RETURN_ON_ERROR( datatable_process_float_data_buffer(datatable_handle, i, &float_value, &value_ts), TAG, "process float data buffer for data-table process samples failed" );
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.float_data.value_ts   = value_ts;
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.float_data.value      = float_value;
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_handle, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
            case DATATABLE_COLUMN_DATA_INT16:
                int16_t int16_value = 0;
                ESP_RETURN_ON_ERROR( datatable_process_int16_data_buffer(datatable_handle, i, &int16_value, &value_ts), TAG, "process int16 data buffer for data-table process samples failed" );
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.int16_data.value_ts   = value_ts;
                datatable_handle->rows[datatable_handle->rows_index].data_columns[i].data.int16_data.value      = int16_value;
                ESP_RETURN_ON_ERROR( datatable_reset_data_buffer(datatable_handle, i), TAG, "reset data buffer for data-table process samples failed" );
                break;
        }
    }

    return ESP_OK;
}

esp_err_t datatable_del(datatable_handle_t datatable_handle) {
    /* free resource */
    if(datatable_handle) {
        free(datatable_handle);
    }

    return ESP_OK;
}


esp_err_t datatable_to_json(datatable_handle_t datatable_handle, cJSON **datatable) {
    /* validate arguments */
    ESP_ARG_CHECK( datatable_handle );

    // create root object for data-table
    cJSON *json_table = cJSON_CreateObject();

    // set data-table attributes
    cJSON_AddStringToObject(json_table, "name", datatable_handle->name);
    cJSON_AddStringToObject(json_table, "process-interval", datatable_json_serialize_interval_type(datatable_handle->processing_interval_type));
    cJSON_AddNumberToObject(json_table, "process-period", datatable_handle->processing_interval_period);

    // create columns array for data-table
    cJSON *json_columns = cJSON_CreateArray();

    // render each data-table column to json column object
    uint8_t col_index = 0;
    for(uint8_t ci = 0; ci <= datatable_handle->columns_index; ci++) {
        datatable_column_t dt_column = datatable_handle->columns[ci];
        
        /* handle basic and complex data-types */
        if(dt_column.data_type == DATATABLE_COLUMN_DATA_ID || dt_column.data_type == DATATABLE_COLUMN_DATA_TS ||
           dt_column.data_type == DATATABLE_COLUMN_DATA_BOOL) {
            cJSON *json_column = cJSON_CreateObject();

            // set column attributes and append column to array
            cJSON_AddNumberToObject(json_column, "index", col_index++);
            cJSON_AddStringToObject(json_column, "name", dt_column.names[0].name);
            cJSON_AddStringToObject(json_column, "data-type", datatable_json_serialize_column_data_type(dt_column.data_type));
            cJSON_AddStringToObject(json_column, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
            cJSON_AddItemToArray(json_columns, json_column);
        } else {
            /* handle complex data-types*/
            if(dt_column.data_type == DATATABLE_COLUMN_DATA_VECTOR) {
                /* handle process-types */
                if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_column.process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column_0 = cJSON_CreateObject(); // u-component
                    cJSON *json_column_1 = cJSON_CreateObject(); // v-component

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_0, "index", col_index++);
                    cJSON_AddStringToObject(json_column_0, "name", dt_column.names[0].name);
                    cJSON_AddStringToObject(json_column_0, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                    cJSON_AddStringToObject(json_column_0, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_1, "index", col_index++);
                    cJSON_AddStringToObject(json_column_1, "name", dt_column.names[1].name);
                    cJSON_AddStringToObject(json_column_1, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                    cJSON_AddStringToObject(json_column_1, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_1);
                } else if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateObject(); // u-component
                    cJSON *json_column_1 = cJSON_CreateObject(); // v-component
                    cJSON *json_column_2 = cJSON_CreateObject(); // timestamp

                    /* 3 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_0, "index", col_index++);
                    cJSON_AddStringToObject(json_column_0, "name", dt_column.names[0].name);
                    cJSON_AddStringToObject(json_column_0, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                    cJSON_AddStringToObject(json_column_0, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_1, "index", col_index++);
                    cJSON_AddStringToObject(json_column_1, "name", dt_column.names[1].name);
                    cJSON_AddStringToObject(json_column_1, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                    cJSON_AddStringToObject(json_column_1, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_1);

                    // set column 2 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_2, "index", col_index++);
                    cJSON_AddStringToObject(json_column_2, "name", dt_column.names[2].name);
                    cJSON_AddStringToObject(json_column_2, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_TS));
                    cJSON_AddStringToObject(json_column_2, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_2);
                }
            } else if(dt_column.data_type == DATATABLE_COLUMN_DATA_FLOAT || dt_column.data_type == DATATABLE_COLUMN_DATA_INT16) {
                /* handle process-types */
                if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_column.process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                    dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                    cJSON *json_column = cJSON_CreateObject();

                    // set column attributes and append column to array
                    cJSON_AddNumberToObject(json_column, "index", col_index++);
                    cJSON_AddStringToObject(json_column, "name", dt_column.names[0].name);
                    cJSON_AddStringToObject(json_column, "data-type", datatable_json_serialize_column_data_type(dt_column.data_type));
                    cJSON_AddStringToObject(json_column, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column);
               } else if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                    cJSON *json_column_0 = cJSON_CreateObject();
                    cJSON *json_column_1 = cJSON_CreateObject();

                    /* 2 columns */

                    // set column 0 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_0, "index", col_index++);
                    cJSON_AddStringToObject(json_column_0, "name", dt_column.names[0].name);
                    cJSON_AddStringToObject(json_column_0, "data-type", datatable_json_serialize_column_data_type(dt_column.data_type));
                    cJSON_AddStringToObject(json_column_0, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_0);

                    // set column 1 attributes and append column to array
                    cJSON_AddNumberToObject(json_column_1, "index", col_index++);
                    cJSON_AddStringToObject(json_column_1, "name", dt_column.names[1].name);
                    cJSON_AddStringToObject(json_column_1, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_TS));
                    cJSON_AddStringToObject(json_column_1, "process-type", datatable_json_serialize_process_type(dt_column.process_type));
                    cJSON_AddItemToArray(json_columns, json_column_1);
               } /* process-types */   
            } /* complex data-types */
        } /* simple data-types */
    } /* for each data-table column */

    // append rendered columns to data-table
    cJSON_AddItemToObject(json_table, "columns", json_columns);

    /* validate rows count */
    if(datatable_handle->rows_count > 0) {
        // create rows array for data-table
        cJSON *json_rows = cJSON_CreateArray();

        // render each data-table row to json row object
        for(uint16_t ri = 0; ri <= datatable_handle->rows_index; ri++) {
            datatable_row_t dt_row = datatable_handle->rows[ri];
            cJSON *json_row = cJSON_CreateObject();

            // set row attributes
            cJSON_AddNumberToObject(json_row, "index", ri);

            // create row data columns array
            cJSON *json_row_data_columns = cJSON_CreateArray();

            // render each data-table row data column
            col_index = 0;
            for(uint8_t ci = 0; ci <= datatable_handle->columns_index; ci++) {
                datatable_column_t dt_column = datatable_handle->columns[ci];
                datatable_row_data_column_t dt_row_data_column = dt_row.data_columns[ci];

                /* handle basic and complex data-types */
                if(dt_column.data_type == DATATABLE_COLUMN_DATA_ID || dt_column.data_type == DATATABLE_COLUMN_DATA_TS ||
                dt_column.data_type == DATATABLE_COLUMN_DATA_BOOL) {
                    cJSON *json_row_data_column = cJSON_CreateObject();

                    // set row data column attributes
                    cJSON_AddNumberToObject(json_row_data_column, "index", col_index++);
                    cJSON_AddStringToObject(json_row_data_column, "data-type", datatable_json_serialize_column_data_type(dt_row_data_column.data_type));

                    /* handle data-type */
                    if(dt_column.data_type == DATATABLE_COLUMN_DATA_ID) {
                        cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.id_data.value);
                    } else if(dt_column.data_type == DATATABLE_COLUMN_DATA_TS) {
                        cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.ts_data.value);
                    } else {
                        cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.bool_data.value);
                    }

                    // append rendered row data column to row data columns array
                    cJSON_AddItemToArray(json_row_data_columns, json_row_data_column);
                } else {
                    /* handle complex data-types*/
                    if(dt_row_data_column.data_type == DATATABLE_COLUMN_DATA_VECTOR) {
                        /* handle process-types */
                        if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_column.process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                            dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                            cJSON *json_row_data_column_0 = cJSON_CreateObject();  // u-component
                            cJSON *json_row_data_column_1 = cJSON_CreateObject();  // v-component

                            /* 2-columns */

                            // set row data column 0 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_0, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_0, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                            cJSON_AddNumberToObject(json_row_data_column_0, "value", dt_row_data_column.data.vector_data.value_uc);

                            // append rendered row data column 0 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_0);

                            // set row data column 1 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_1, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_1, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                            cJSON_AddNumberToObject(json_row_data_column_1, "value", dt_row_data_column.data.vector_data.value_vc);

                            // append rendered row data column 1 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_1);
                        } else if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                            cJSON *json_row_data_column_0 = cJSON_CreateObject();  // u-component
                            cJSON *json_row_data_column_1 = cJSON_CreateObject();  // v-component
                            cJSON *json_row_data_column_2 = cJSON_CreateObject();  // timestamp

                            /* 3-columns */

                            // set row data column 0 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_0, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_0, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                            cJSON_AddNumberToObject(json_row_data_column_0, "value", dt_row_data_column.data.vector_data.value_uc);

                            // append rendered row data column 0 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_0);

                            // set row data column 1 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_1, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_1, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_FLOAT));
                            cJSON_AddNumberToObject(json_row_data_column_1, "value", dt_row_data_column.data.vector_data.value_vc);

                            // append rendered row data column 1 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_1);

                            // set row data column 2 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_2, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_2, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_TS));
                            cJSON_AddNumberToObject(json_row_data_column_2, "value", dt_row_data_column.data.vector_data.value_ts);

                            // append rendered row data column 0 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_2);
                        }
                    } else if(dt_column.data_type == DATATABLE_COLUMN_DATA_FLOAT || dt_column.data_type == DATATABLE_COLUMN_DATA_INT16) {
                        /* handle process-types */
                        if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_SMP || dt_column.process_type == DATATABLE_COLUMN_PROCESS_AVG || 
                            dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX) {
                            cJSON *json_row_data_column = cJSON_CreateObject();

                            // set row data column attributes
                            cJSON_AddNumberToObject(json_row_data_column, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column, "data-type", datatable_json_serialize_column_data_type(dt_row_data_column.data_type));

                            /* handle data-type */
                            if(dt_column.data_type == DATATABLE_COLUMN_DATA_FLOAT) {
                                cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.float_data.value);
                            } else {
                                cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.int16_data.value);
                            }

                            // append rendered row data column to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column);
                        } else if(dt_column.process_type == DATATABLE_COLUMN_PROCESS_MIN_TS || dt_column.process_type == DATATABLE_COLUMN_PROCESS_MAX_TS) {
                            /* 2 columns: value with timestamp */
                            cJSON *json_row_data_column_0 = cJSON_CreateObject();
                            cJSON *json_row_data_column_1 = cJSON_CreateObject();

                            // set row data column 0 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_0, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_0, "data-type", datatable_json_serialize_column_data_type(dt_row_data_column.data_type));

                            /* handle data-type for row data column 0 */
                            if(dt_column.data_type == DATATABLE_COLUMN_DATA_FLOAT) {
                                cJSON_AddNumberToObject(json_row_data_column_0, "value", dt_row_data_column.data.float_data.value);
                            } else {
                                cJSON_AddNumberToObject(json_row_data_column_0, "value", dt_row_data_column.data.int16_data.value);
                            }

                            // append rendered row data column 0 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_0);

                            // set row data column 1 attributes - value
                            cJSON_AddNumberToObject(json_row_data_column_1, "index", col_index++);
                            cJSON_AddStringToObject(json_row_data_column_1, "data-type", datatable_json_serialize_column_data_type(DATATABLE_COLUMN_DATA_TS));

                            /* handle data-type for row data column 0 */
                            if(dt_column.data_type == DATATABLE_COLUMN_DATA_FLOAT) {
                                cJSON_AddNumberToObject(json_row_data_column_1, "value", dt_row_data_column.data.float_data.value_ts);
                            } else {
                                cJSON_AddNumberToObject(json_row_data_column_1, "value", dt_row_data_column.data.int16_data.value_ts);
                            }

                            // append rendered row data column 1 to row data columns array
                            cJSON_AddItemToArray(json_row_data_columns, json_row_data_column_1);
                        } /* process-types */   
                    } /* complex data-types */
                } /* simple data-types */
            } // for-each row-data-column

            // append rendered row data columns array to row 
            cJSON_AddItemToObject(json_row, "columns", json_row_data_columns);
            
            // append rendered row to rows array
            cJSON_AddItemToArray(json_rows, json_row);
        } // for-each row

        // append rendered rows to data-table
        cJSON_AddItemToObject(json_table, "rows", json_rows);
    } /* row count */

    /* set json output table */
    *datatable = json_table;

    return ESP_OK;
}
