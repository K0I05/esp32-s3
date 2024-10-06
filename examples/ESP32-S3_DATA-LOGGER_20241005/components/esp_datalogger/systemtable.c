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
 * @file systemtable.c
 *
 * ESP-IDF library for SYSTEM-TABLE
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "systemtable.h"
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
static const char *TAG = "system-table";



/**
 * @brief Serializes system-table column data-type to a textual string for json formating.
 * 
 * @param data_type System-table column data-type to serialize.
 * @return char* Serialized system-table column data-type as a textual string.
 */
static inline const char* systemtable_json_serialize_column_data_type(const systemtable_column_data_types_t data_type) {
    static char data[8] = "-";

    /* normalize  */
    switch(data_type) {
        case SYSTEMTABLE_COLUMN_DATA_ID:
            strcpy(data, "id");
            break;
        case SYSTEMTABLE_COLUMN_DATA_TS:
            strcpy(data, "ts");
            break;
        case SYSTEMTABLE_COLUMN_DATA_MSG:
            strcpy(data, "msg");
            break;
    }

    return data;
}



/**
 * @brief Checks if the system-table column exist by column index.
 * 
 * @param[in] systemtable_handle System-table handle.
 * @param[in] index System-table column index to check if it exist.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG when the index is out of range and column does not exist.
 */
static inline esp_err_t systemtable_column_exist(systemtable_handle_t systemtable_handle, const uint8_t index) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* validate index */
    ESP_RETURN_ON_FALSE((index < systemtable_handle->columns_size), ESP_ERR_INVALID_ARG, TAG, "index is out of range, get column failed");

    return ESP_OK;
}

/**
 * @brief Checks if the system-table is full (i.e. number of rows matches configured rows size).
 * 
 * @param systemtable_handle System-table handle.
 * @param full True when system-table is full, otherwise, false.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t systemtable_is_full(systemtable_handle_t systemtable_handle, bool *full) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* validate if the system-table is full and set output parameter */
    if(systemtable_handle->rows_count >= systemtable_handle->rows_size) {
        *full = true;
    } else {
        *full = false;
    }

    return ESP_OK;
}

/**
 * @brief Pops the top system-table row and shifts the index of remaining rows up by one i.e. first-in-first-out (FIFO) principal.
 * 
 * @param systemtable_handle System-table handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t systemtable_fifo_rows(systemtable_handle_t systemtable_handle) {
    systemtable_row_t* tmp_rows;

    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* validate memory availability for default system-table tmp rows */
    tmp_rows = (systemtable_row_t*)calloc(systemtable_handle->rows_size, sizeof(systemtable_row_t));
    ESP_RETURN_ON_FALSE(tmp_rows, ESP_ERR_NO_MEM, TAG, "no memory for data-table temporary rows, data-table fifo rows failed");

    /* copy working rows to tmp rows, skip the first row */
    for(int r = 1; r < systemtable_handle->rows_count; r++) {
        tmp_rows[r - 1] = systemtable_handle->rows[r];
    }

    /* copy tmp rows to working rows */
    for(int r = 0; r < systemtable_handle->rows_count - 1; r++) {
        systemtable_handle->rows[r] = tmp_rows[r];
    }

    /* free up memory used by the tmp rows */
    if(tmp_rows) {
        free(tmp_rows);
    }

    return ESP_OK;
}

/**
 * @brief Resets system-table rows, this is full reset, all data is deleted and system-table row array is re-initialized per configured row size.
 * 
 * @param systemtable_handle System-table handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t systemtable_reset_rows(systemtable_handle_t systemtable_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* reset rows by freeing up memory */
    if(systemtable_handle->rows) {
        free(systemtable_handle->rows);
    }

    /* reset row attributes */
    systemtable_handle->rows_index = 0;
    systemtable_handle->rows_count = 0;

    /* validate memory availability for default system-table rows */
    systemtable_handle->rows = (systemtable_row_t*)calloc(systemtable_handle->rows_size, sizeof(systemtable_row_t));
    ESP_RETURN_ON_FALSE(systemtable_handle->rows, ESP_ERR_NO_MEM, TAG, "no memory for data-table rows, data-table reset rows failed");

    return ESP_OK;
}





esp_err_t systemtable_init(systemtable_handle_t *systemtable_handle) {
    esp_err_t               ret = ESP_OK;
    systemtable_handle_t    out_handle = NULL;

    /* validate memory availability for system-table handle */
    out_handle = (systemtable_handle_t)calloc(1, sizeof(systemtable_t));
    ESP_GOTO_ON_FALSE( out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for system-table handle, system-table handle initialization failed" );

    /* initialize system-table state object */
    strcpy(out_handle->name, "System-Tbl");
    out_handle->columns_index   = 0;
    out_handle->columns_size    = SYSTEMTABLE_COLUMNS_MAX;
    out_handle->rows_index      = 0;
    out_handle->rows_count      = 0;
    out_handle->rows_size       = SYSTEMTABLE_ROWS_MAX;
    out_handle->event_id        = 0;

    /* define default event identifier system-table column */
    systemtable_column_t st_id_column = {
        .index                  = out_handle->columns_index,
        .data_type              = SYSTEMTABLE_COLUMN_DATA_ID
    };
    strcpy(st_id_column.name, "Event ID");

    /* increment column index */
    out_handle->columns_index   += 1;

    /* define default event timestamp system-table column */
    systemtable_column_t st_ts_column = {
        .index                  = out_handle->columns_index,
        .data_type              = SYSTEMTABLE_COLUMN_DATA_TS
    };
    strcpy(st_ts_column.name, "Event TS");

    /* increment column index */
    out_handle->columns_index   += 1;

    /* define default event message system-table column */
    systemtable_column_t st_msg_column = {
        .index                  = out_handle->columns_index,
        .data_type              = SYSTEMTABLE_COLUMN_DATA_MSG
    };
    strcpy(st_msg_column.name, "Event MSG");

    /* validate memory availability for default system-table columns */
    out_handle->columns = (systemtable_column_t*)calloc(out_handle->columns_size, sizeof(systemtable_column_t));
    ESP_GOTO_ON_FALSE( out_handle->columns, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for system-table columns, system-table handle initialization failed" );

    /* validate memory availability for default system-table rows */
    out_handle->rows = (systemtable_row_t*)calloc(out_handle->rows_size, sizeof(systemtable_row_t));
    ESP_GOTO_ON_FALSE( out_handle->rows, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for system-table rows, system-table handle initialization failed" );

    /* set default system-table columns (event id, timestamp, and message) to state object */
    out_handle->columns[0] = st_id_column;
    out_handle->columns[1] = st_ts_column;
    out_handle->columns[2] = st_msg_column;

    /* set output handle */
    *systemtable_handle = out_handle;

    return ESP_OK;

    err_out_handle:
        free(out_handle);
    err:
        return ret;
}

esp_err_t systemtable_get_columns_count(systemtable_handle_t systemtable_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* set output parameter */
    *count = systemtable_handle->columns_index + 1;

    return ESP_OK;
}

esp_err_t systemtable_get_rows_count(systemtable_handle_t systemtable_handle, uint8_t *count) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* set output parameter */
    *count = systemtable_handle->rows_index + 1;

    return ESP_OK;
}

esp_err_t systemtable_push_event_msg(systemtable_handle_t systemtable_handle, const char* message) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    /* validate event message length */
    ESP_RETURN_ON_FALSE((strlen(message) < SYSTEMTABLE_DATA_COL_MSG_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "event message is too long, maximum of 40-characters, system-table handle push event message failed");

    /* check if the system-table is full */
    bool st_is_full;

    /* check if the system-table is full */
    ESP_RETURN_ON_ERROR( systemtable_is_full(systemtable_handle, &st_is_full), TAG, "unable to tell if system-table is full, system-table handle push event message failed" );

    /* if system-table is full, pop the first event */
    if(st_is_full == true) {
        // pop first row and push remaining rows to top of stack
        ESP_RETURN_ON_ERROR( systemtable_fifo_rows(systemtable_handle), TAG, "unable to fifo system-table rows, system-table handle push event message failed" );
    }

    /* handle system-table row count and index */
    if(systemtable_handle->rows_count == 0) {
        /* initialize system-table row count and index */
        systemtable_handle->rows_count = 1;
        systemtable_handle->rows_index = 0;
    } else {
        /* increment system-table row count and index */
        systemtable_handle->rows_count += 1;
        systemtable_handle->rows_index += 1;

        /* if the system-table is full, decrement row count and index */
        if(systemtable_handle->rows_count > systemtable_handle->rows_size) {
            systemtable_handle->rows_count -= 1;
            systemtable_handle->rows_index -= 1;
        }
    }

    /* validate memory availability for default system-table rows */
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns = (systemtable_row_data_column_t*)calloc(systemtable_handle->columns_size, sizeof(systemtable_row_data_column_t));
    ESP_RETURN_ON_FALSE( systemtable_handle->rows[systemtable_handle->rows_index].data_columns, ESP_ERR_NO_MEM, TAG, "no memory for system-table rows, system-table handle push event message failed" );

    /* set system-table row data-column size and row index */
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns_size              = SYSTEMTABLE_COLUMNS_MAX;
    systemtable_handle->rows[systemtable_handle->rows_index].index                          = systemtable_handle->rows_index;

    /* set system-table row data-column event identifier */
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[0].column_index   = 0;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[0].row_index      = systemtable_handle->rows_index;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[0].data_type      = SYSTEMTABLE_COLUMN_DATA_ID;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[0].data.id.value  = systemtable_handle->event_id++;

    /* set system-table row data-column event timestamp (utc) */
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[1].column_index   = 1;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[1].row_index      = systemtable_handle->rows_index;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[1].data_type      = SYSTEMTABLE_COLUMN_DATA_TS;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[1].data.ts.value  = datalogger_get_epoch_timestamp();

    /* set system-table row data-column event message */
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[2].column_index   = 2;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[2].row_index      = systemtable_handle->rows_index;
    systemtable_handle->rows[systemtable_handle->rows_index].data_columns[2].data_type      = SYSTEMTABLE_COLUMN_DATA_MSG;
    strcpy(systemtable_handle->rows[systemtable_handle->rows_index].data_columns[2].data.msg.value, message);

    return ESP_OK;
}

esp_err_t systemtable_del(systemtable_handle_t systemtable_handle) {
    /* free resource */
    if(systemtable_handle) {
        free(systemtable_handle);
    }

    return ESP_OK;
}


esp_err_t systemtable_to_json(systemtable_handle_t systemtable_handle, cJSON **systemtable) {
    /* validate arguments */
    ESP_ARG_CHECK( systemtable_handle );

    // create root object for system-table
    cJSON *json_table = cJSON_CreateObject();

    // set system-table attributes
    cJSON_AddStringToObject(json_table, "name", systemtable_handle->name);

    // create columns array for data-table
    cJSON *json_columns = cJSON_CreateArray();

    // render each system-table column to json column object
    uint8_t col_index = 0;
    for(uint8_t ci = 0; ci <= systemtable_handle->columns_index; ci++) {
        systemtable_column_t dt_column = systemtable_handle->columns[ci];
        
        /* handle basic and complex data-types */
        if(dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_ID || dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_TS ||
           dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_MSG) {
            cJSON *json_column = cJSON_CreateObject();

            // set column attributes and append column to array
            cJSON_AddNumberToObject(json_column, "index", col_index++);
            cJSON_AddStringToObject(json_column, "name", dt_column.name);
            cJSON_AddStringToObject(json_column, "data-type", systemtable_json_serialize_column_data_type(dt_column.data_type));
            cJSON_AddItemToArray(json_columns, json_column);
        } 
    } /* for each system-table column */

    // append rendered columns to system-table
    cJSON_AddItemToObject(json_table, "columns", json_columns);

    /* validate rows count */
    if(systemtable_handle->rows_count > 0) {
        // create rows array for system-table
        cJSON *json_rows = cJSON_CreateArray();

        // render each system-table row to json row object
        for(uint16_t ri = 0; ri <= systemtable_handle->rows_index; ri++) {
            systemtable_row_t dt_row = systemtable_handle->rows[ri];
            cJSON *json_row = cJSON_CreateObject();

            // set row attributes
            cJSON_AddNumberToObject(json_row, "index", ri);

            // create row data columns array
            cJSON *json_row_data_columns = cJSON_CreateArray();

            // render each system-table row data column
            col_index = 0;
            for(uint8_t ci = 0; ci <= systemtable_handle->columns_index; ci++) {
                systemtable_column_t dt_column = systemtable_handle->columns[ci];
                systemtable_row_data_column_t dt_row_data_column = dt_row.data_columns[ci];

                /* handle basic and complex data-types */
                if(dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_ID || dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_TS ||
                dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_MSG) {
                    cJSON *json_row_data_column = cJSON_CreateObject();

                    // set row data column attributes
                    cJSON_AddNumberToObject(json_row_data_column, "index", col_index++);
                    cJSON_AddStringToObject(json_row_data_column, "data-type", systemtable_json_serialize_column_data_type(dt_row_data_column.data_type));

                    /* handle data-type */
                    if(dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_ID) {
                        cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.id.value);
                    } else if(dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_TS) {
                        cJSON_AddNumberToObject(json_row_data_column, "value", dt_row_data_column.data.ts.value);
                    } else if(dt_column.data_type == SYSTEMTABLE_COLUMN_DATA_MSG) {
                        cJSON_AddStringToObject(json_row_data_column, "value", dt_row_data_column.data.msg.value);
                    }

                    // append rendered row data column to row data columns array
                    cJSON_AddItemToArray(json_row_data_columns, json_row_data_column);
                } 
            } // for-each row-data-column

            // append rendered row data columns array to row 
            cJSON_AddItemToObject(json_row, "columns", json_row_data_columns);
            
            // append rendered row to rows array
            cJSON_AddItemToArray(json_rows, json_row);
        } // for-each row

        // append rendered rows to system-table
        cJSON_AddItemToObject(json_table, "rows", json_rows);
    } /* row count */

    /* set json output table */
    *systemtable = json_table;

    return ESP_OK;
}
