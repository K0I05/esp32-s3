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
 * @file systemtable.h
 * @defgroup drivers systemtable
 * @{
 *
 * ESP-IDF library for systetable
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SYSTEMTABLE_H__
#define __SYSTEMTABLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <cJSON.h>
#include <datalogger_common.h>


#ifdef __cplusplus
extern "C"
{
#endif


/*
 * ESP SYSTEM-TABLE definitions
 */
#define SYSTEMTABLE_NAME_MAX_SIZE         (15)        //!< 15-characters for user-defined table name
#define SYSTEMTABLE_COLUMN_NAME_SIZE      (15)        //!< 15-characters for user-defined column name 
#define SYSTEMTABLE_COLUMN_NAME_MAX_SIZE  (25)        //!< 25-characters for column name
#define SYSTEMTABLE_COLUMNS_MAX           (3)         //!< 3-columns for system-table (event id, event timestamp, message) 
#define SYSTEMTABLE_ROWS_MAX              (20)        //!< 
#define SYSTEMTABLE_DATA_COL_MSG_MAX_SIZE (40)        //!< 40-characters for data-column message
#define SYSTEMTABLE_NAME                  "system_tbl"
#define SYSTEMTABLE_COLUMN_ID_NAME        "Event ID"
#define SYSTEMTABLE_COLUMN_TS_NAME        "Event TS"
#define SYSTEMTABLE_COLUMN_MSG_NAME       "Event MSG"

/**
 * @brief System-table column data-types enumerator.
 */
typedef enum {
    SYSTEMTABLE_COLUMN_DATA_ID,       /*!< event identifier column data-type, system default, see `systemtable_id_data_type_t` for data-type structure. */
    SYSTEMTABLE_COLUMN_DATA_TS,       /*!< event timestamp (date and time) column data type, system default, see `systemtable_ts_data_type_t` for data-type structure. */
    SYSTEMTABLE_COLUMN_DATA_MSG,      /*!< event message column data type, system default, see `systemtable_message_data_type_t` for data-type structure. */
} systemtable_column_data_types_t;

/**
 * @brief System-table event identifier column data-type structure.
 */
typedef struct {
    uint16_t                            value;      // record id value   
} systemtable_id_column_data_type_t;

/**
 * @brief System-table event timestamp (utc) column data-type structure.
 */
typedef struct {
    time_t                              value;      // timestamp value    
} systemtable_ts_column_data_type_t;

/**
 * @brief System-table event message column data-type structure.
 */
typedef struct {
    const char*                        value;      // message value    
} systemtable_message_column_data_type_t;


typedef struct {
    systemtable_column_data_types_t data_type;
    union {
        systemtable_id_column_data_type_t       id;
        systemtable_ts_column_data_type_t       ts;
        systemtable_message_column_data_type_t  msg;
    } data;
} systemtable_row_data_column_t;

typedef struct {
    const char*                     name;
    systemtable_column_data_types_t data_type;
} systemtable_column_t;

typedef struct {
    uint16_t                        data_columns_size;      // system-table size of row data columns, automatically populated when row is created.
    systemtable_row_data_column_t** data_columns;
} systemtable_row_t;


struct systemtable_t {
    const char*                     name;
    uint8_t                         columns_count;              /*!< system-table column count seed number, this number should not exceed the column size */
    uint8_t                         columns_size;               /*!< system-table column array size, static, set when system-table is created */
    systemtable_column_t**          columns;                    /*!< array of system-table columns */
    uint16_t                        rows_count;                 /*!< system-table row count seed number, this number should not exceed the row size */
    uint16_t                        rows_size;                  /*!< system-table row array size, static, set when system-table is created */
    systemtable_row_t**             rows;                       /*!< array of system-table rows */   
    uint16_t                        event_id;  
};


/**
 * @brief System-table structure.
 */
typedef struct systemtable_t systemtable_t;

 /**
  * @brief System-table handle structure.
  */
typedef struct systemtable_t *systemtable_handle_t;



esp_err_t systemtable_init(systemtable_handle_t *systemtable_handle);

esp_err_t systemtable_get_columns_count(systemtable_handle_t systemtable_handle, uint8_t *count);

esp_err_t systemtable_get_rows_count(systemtable_handle_t systemtable_handle, uint8_t *count);

esp_err_t systemtable_push_event_msg(systemtable_handle_t systemtable_handle, const char* message);

esp_err_t systemtable_del(systemtable_handle_t systemtable_handle);

esp_err_t systemtable_to_json(systemtable_handle_t systemtable_handle, cJSON **systemtable);






#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __SYSTEMTABLE_H__
