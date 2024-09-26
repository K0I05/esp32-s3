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
 * @file datatable.h
 * @defgroup drivers datatable
 * @{
 *
 * ESP-IDF library for datatable
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DATATABLE_H__
#define __DATATABLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <cJSON.h>
#include <datalogger_common.h>
#include <time_into_interval.h>
#include <task_schedule.h>



#ifdef __cplusplus
extern "C"
{
#endif

/*
 * ESP DATA-TABLE definitions
 */
#define DATATABLE_NAME_SIZE             (15)        //!< 15-characters for user-defined table name
#define DATATABLE_COLUMN_NAME_SIZE      (15)        //!< 15-characters for user-defined column name 
#define DATATABLE_COLUMN_NAME_MAX_SIZE  (25)        //!< 25-characters for column name
#define DATATABLE_COLUMNS_MAXIMUM       (255)       //!< 
#define DATATABLE_ROWS_MAXIMUM          (65535)     //!< 


/*
 * ESP DATA-TABLE macro definitions
 */

/*
 * ESP DATA-TABLE enum and struct definitions
 */


/**
 * @brief Data-table data storage-types enumerator.
 */
typedef enum {
    DATATABLE_DATA_STORAGE_MEMORY_RING,     /*!< data-table ring memory performs a first-in first-out (FIFO) with the data-table */
    DATATABLE_DATA_STORAGE_MEMORY_RESET,    /*!< data-table memory reset performs a data-table reset when data-table is full */
    DATATABLE_DATA_STORAGE_MEMORY_STOP      /*!< data-table memory stop pauses data storage when data-table is full */
} datatable_data_storage_types_t;

/**
 * @brief Data-table column statistical process-types enumerator.
 */
typedef enum {
    DATATABLE_COLUMN_PROCESS_SMP,       /*!< a sample is stored at every processing interval */
    DATATABLE_COLUMN_PROCESS_AVG,       /*!< stored samples are averaged over the processing interval */
    DATATABLE_COLUMN_PROCESS_MIN,       /*!< stored samples are analyzed for minimum over the processing interval */
    DATATABLE_COLUMN_PROCESS_MAX,       /*!< stored samples are analyzed for maximum over the processing interval */
    DATATABLE_COLUMN_PROCESS_MIN_TS,    /*!< stored samples are analyzed for minimum with timestamp over the processing interval */
    DATATABLE_COLUMN_PROCESS_MAX_TS,    /*!< stored samples are analyzed for maximum with timestamp over the processing interval */
} datatable_column_process_types_t;

/**
 * @brief Data-table column data-types enumerator.
 */
typedef enum {
    DATATABLE_COLUMN_DATA_ID,       /*!< record identifier column data-type, system default, see `datatable_id_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_TS,       /*!< record timestamp (date and time) column data type, system default, see `datatable_ts_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_VECTOR,   /*!< vector (u and v components) column data type, user-defined, see `datatable_vector_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_BOOL,     /*!< boolean column data type, user-defined, see `datatable_bool_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_FLOAT,    /*!< float column data type, user-defined, see `datatable_float_data_type_t` for data-type structure. */
    DATATABLE_COLUMN_DATA_INT16     /*!< int16 column data type, user-defined, see `datatable_int16_data_type_t` for data-type structure. */
} datatable_column_data_types_t;


/**
 * @brief Data-table record identifier column data-type structure.
 */
typedef struct {
    uint16_t                            value;      // record id value   
} datatable_id_column_data_type_t;

/**
 * @brief Data-table record timestamp (utc) column data-type structure.
 */
typedef struct {
    time_t                              value;      // timestamp value    
} datatable_ts_column_data_type_t;

/**
 * @brief Data-table vector data-type column structure.
 */
typedef struct {
    float                               value_uc;   // u-component (angle) value 
    float                               value_vc;   // v-component (velocity) value
    time_t                              value_ts;   // timestamp of values, used for time of max or min
} datatable_vector_column_data_type_t;

/**
 * @brief Data-table bool data-type column structure.
 */
typedef struct {
    bool                                value;      // boolean value 
} datatable_bool_column_data_type_t;

/**
 * @brief Data-table float data-type column structure.
 */
typedef struct {
    float                               value;      // float value
    time_t                              value_ts;   // timestamp of value, used for time of max or min  
} datatable_float_column_data_type_t;

/**
 * @brief Data-table int16 data-type column structure.
 */
typedef struct {
    int16_t                             value;      // int16 value
    time_t                              value_ts;   // timestamp of value, used for time of max or min   
} datatable_int16_column_data_type_t;

/**
 * @brief Data-table column data buffer structure.  The record identifier and
 * record timestamp data-types are excluded from data processing.
 */
typedef struct {
    uint16_t                            samples_size;       // data-table size of data buffer samples, automatically populated when column is created
    uint16_t                            samples_count;      // data-table number of samples in the data buffer, automatically populated when data-table is processed
    datatable_column_data_types_t       data_type;          // data-table column data buffer data-type, automatically populated when column is created
    union {
        datatable_vector_column_data_type_t*   vector_samples;     // data-table vector samples data buffer, automatic array sizing when column is created based configured column data-type
        datatable_bool_column_data_type_t*     bool_samples;       // data-table boolean samples data buffer, automatic array sizing when column is created based configured column data-type
        datatable_float_column_data_type_t*    float_samples;      // data-table float samples data buffer, automatic array sizing when column is created based configured column data-type
        datatable_int16_column_data_type_t*    int16_samples;      // data-table int16 samples data buffer, automatic array sizing when column is created based configured column data-type
    } buffer;                                               // data-table column data buffer union based on the configured column data-type, automatically populated when column is created.
} datatable_column_data_buffer_t;

typedef struct {
    char                                name[DATATABLE_COLUMN_NAME_MAX_SIZE];  // data-table column name, maximum 15 characters.
} datatable_column_name_t;
/**
 * @brief Data-table column structure.  The data-table record identifier and timestamp columns
 * are created by default when the data-table is created.
 */
typedef struct {
    uint8_t                             index;              // data-table column index, automatically populated when column is created.
    datatable_column_name_t             names[3];           // data-table column names, index 0 and 1 for vector data-type or index 0, 1, and 2 for max and min with timestamp process-types, and index 0 for all other scenarios.
    datatable_column_data_types_t       data_type;          // data-table column data-type setting, automatically populated when column is created.
    datatable_column_data_buffer_t      data;               // data-table column data buffer structure for data processing, automatically populated when column is created.
    datatable_column_process_types_t    process_type;       // data-table statistical data processing type setting.
} datatable_column_t;

/**
 * @brief Data-table row data column structure.  This structure is a data model that represents 
 * data storage of the record based on the data-table's column column data-type.
 */
typedef struct {
    uint8_t                             column_index;       // data-table column index, automatically populated when row is created.
    uint16_t                            row_index;          // data-table row index, automatically populated when row is created.
    datatable_column_data_types_t       data_type;          // data-table column data-type, automatically populated when row is created.
    union {
        datatable_id_column_data_type_t        id_data;            // data-table column record identifier data-type structure, automatically populated when row is created.
        datatable_ts_column_data_type_t        ts_data;            // data-table column record timestamp data-type structure, automatically populated when row is created.
        datatable_vector_column_data_type_t    vector_data;        // data-table column unit-vector data-type structure, automatically populated when row is created.
        datatable_bool_column_data_type_t      bool_data;          // data-table column boolean data-type structure, automatically populated when row is created.
        datatable_float_column_data_type_t     float_data;         // data-table column float data-type structure, automatically populated when row is created.
        datatable_int16_column_data_type_t     int16_data;         // data-table column int16 data-type structure, automatically populated when row is created.
    } data;                                                 // data-table row data column data union based on the configured column data-type, automatically populated when row is created.
} datatable_row_data_column_t;

/**
 * @brief Data-table row structure.  This structure is a data model that represents
 * data storage of record by data-table row and configured data-table columns.
 */
typedef struct {
    uint16_t                        index;                  // data-table row index, automatically populated when row is created.
    uint16_t                        data_columns_size;      // data-table size of row data columns, automatically populated when row is created.
    datatable_row_data_column_t*    data_columns;           // data-table data columns of the record contained in the row, automatically populated when row is created.
} datatable_row_t;


/**
 * @brief Data-table structure.
 *
 */
typedef struct datatable_t datatable_t;

 /**
  * @brief Data-table handle structure.
  * 
  */
typedef struct datatable_t *datatable_handle_t;

/**
 * @brief Data-table state object structure definition.  Do not modify these fields once the
 * data-table handle is created, these are read-only, and represent a state machine.
 * 
 */
struct datatable_t {
    char                                name[DATATABLE_NAME_SIZE];  /*!< data-table name, maximum of 15 characters */
    datatable_data_storage_types_t      data_storage_type;          /*!< data-table data storage type, set when data-table is created */
    uint64_t                            sampling_ticks;             /*!< ticks since the last scan to validate timeticks of samples */
    uint16_t                            sampling_count;             /*!< data-table data sampling count seed number */
    task_schedule_handle_t              sampling_task_schedule_handle;  /*!< data-table sampling task schedule handle */
    //datalogger_time_interval_types_t    sampling_interval_type;     /*!< sampling time interval type of samples pushed onto the data-table data buffer stack */
    //uint16_t                            sampling_interval_period;   /*!< sampling time interval period of samples pushed onto the data-table data buffer stack */
    //uint16_t                            sampling_interval_offset;   /*!< sampling time interval offset of samples pushed onto the data-table data buffer stack */
    datalogger_time_interval_types_t    processing_interval_type;   /*!< processing time interval type of samples to process and store data-table records */
    uint16_t                            processing_interval_period; /*!< processing time interval period of samples to process and store data-table records */
    uint16_t                            processing_interval_offset; /*!< processing time interval offset of samples to process and store data-table records */
    uint16_t                            record_id;                  /*!< data-table record identifer seed number */
    uint8_t                             columns_index;              /*!< data-table column index seed number, this number is always smaller than the column size */
    uint8_t                             columns_size;               /*!< data-table column array size, static, set when data-table is created */
    datatable_column_t*                 columns;                    /*!< array of data-table columns */
    uint16_t                            rows_index;                 /*!< data-table row index seed number, this number is always smaller than the column size */
    uint16_t                            rows_count;                 /*!< data-table row count seed number, this number should not exceed the column size*/
    uint16_t                            rows_size;                  /*!< data-table row array size, static, set when data-table is created */
    datatable_row_t*                    rows;                       /*!< array of data-table rows */
    time_into_interval_handle_t         processing_tti_handle;      /*!< data-table processing time-into-interval handle */
};


/*
 * ESP DATA-TABLE sub-routine and function definitions
 */


/**
 * @brief Creates a data-table handle and must be the first function called.
 * 
 * Use the `datatable_add_[data-type]_column` functions to define data-table columns by data-type.  
 * The data-table columns are ordered as they are added and column index for the first user-defined 
 * column always starts at 2 given  the record identifier and timestamp columns are created by 
 * default and consume column indexes 0 and 1 respectively.
 * 
 * @param[in] name data-table textual name, 15-characters maximum.
 * @param[in] columns_size data-table column array size, this setting cannot be 0.
 * @param[in] rows_size data-table row array size,this setting cannot be 0.
 * @param[in] processing_interval_type processing time interval type of samples to process and store data-table records
 * @param[in] processing_interval_period processing time interval period of samples to process and store data-table records
 * @param[in] processing_interval_offset processing time interval offset of samples to process and store data-table records
 * @param[in] sampling_task_schedule_handle data-table sampling task schedule handle.
 * @param[in] data_storage_type data-table data storage type.
 * @param[out] datatable_handle data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_new(char *name, uint8_t columns_size, uint16_t rows_size, datalogger_time_interval_types_t processing_interval_type, uint16_t processing_interval_period, uint16_t processing_interval_offset, task_schedule_handle_t sampling_task_schedule_handle, datatable_data_storage_types_t data_storage_type, datatable_handle_t *datatable_handle);

/**
 * @brief Appends a vector based data-type column as a sample to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name_uc textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc textual name of the data-table column to be added for vector v-component.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_smp_column(datatable_handle_t datatable_handle, char *name_uc, char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as an average to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name_uc textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc textual name of the data-table column to be added for vector v-component.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_avg_column(datatable_handle_t datatable_handle, char *name_uc, char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component minimum to the data-table.
 * 
 * The u-component at v-component minimum is sampled and stored.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name_uc textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc textual name of the data-table column to be added for vector v-component.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_min_column(datatable_handle_t datatable_handle, char *name_uc, char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component maximum to the data-table.
 * 
 * The u-component at v-component maximum is sampled and stored.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name_uc textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc textual name of the data-table column to be added for vector v-component.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_max_column(datatable_handle_t datatable_handle, char *name_uc, char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component minimum with timestamp to the data-table.
 * 
 * The u-component at v-component minimum is sampled and stored.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name_uc textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc textual name of the data-table column to be added for vector v-component.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_min_ts_column(datatable_handle_t datatable_handle, char *name_uc, char *name_vc, uint8_t *index);

/**
 * @brief Appends a vector based data-type column as a v-component maximum with timestamp to the data-table.
 * 
 * The u-component at v-component maximum is sampled and stored.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name_uc textual name of the data-table column to be added for vector u-component.
 * @param[in] name_vc textual name of the data-table column to be added for vector v-component.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_vector_max_ts_column(datatable_handle_t datatable_handle, char *name_uc, char *name_vc, uint8_t *index);

/**
 * @brief Appends a bool based data-type column as a sample process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_bool_smp_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a sample process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_smp_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as an average process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_avg_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a minimum process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_min_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a maximum process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_max_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a minimum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_min_ts_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a float based data-type column as a maximum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_float_max_ts_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a sample process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_smp_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as an average process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_avg_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a minimum process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_min_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a maximum process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_max_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a minimum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_min_ts_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Appends a int16 based data-type column as a maximum with timestamp process-type to the data-table.
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[in] name textual name of the data-table column to be added.
 * @param[out] index index of the column that was added to the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_add_int16_max_ts_column(datatable_handle_t datatable_handle, char *name, uint8_t *index);

/**
 * @brief Gets the number of columns in the data-table.
 * 
 * @param datatable_handle data-table handle.
 * @param count number of columns in the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_columns_count(datatable_handle_t datatable_handle, uint8_t *count);

/**
 * @brief Gets the number of rows in the data-table.
 * 
 * @param datatable_handle data-table handle.
 * @param count number of rows in the data-table.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_rows_count(datatable_handle_t datatable_handle, uint8_t *count);

/**
 * @brief Gets the column structure from the data-table based on the column index.
 * 
 * @param datatable_handle data-table handle.
 * @param index data-table column index to output.
 * @param column data-table column structure output.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_column(datatable_handle_t datatable_handle, uint8_t index, datatable_column_t *column);

/**
 * @brief Gets the row structure from the data-table based on the row index.
 * 
 * @param datatable_handle data-table handle.
 * @param index data-table row index to output.
 * @param row data-table row structure output.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_get_row(datatable_handle_t datatable_handle, uint8_t index, datatable_row_t *row);

/**
 * @brief Pushes a vector data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle data-table handle.
 * @param index sample data-table column index.
 * @param uc_value vector data-type u-component sample to process.
 * @param vc_value vector data-type v-component sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_vector_sample(datatable_handle_t datatable_handle, uint8_t index, float uc_value, float vc_value);

/**
 * @brief Pushes a boolean data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle data-table handle.
 * @param index sample data-table column index.
 * @param value boolean data-type sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_bool_sample(datatable_handle_t datatable_handle, uint8_t index, bool value);

/**
 * @brief Pushes a float data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle data-table handle.
 * @param index sample data-table column index..
 * @param value float data-type sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_float_sample(datatable_handle_t datatable_handle, uint8_t index, float value);

/**
 * @brief Pushes an int16 data-type sample onto the column sample data buffer stack for processing.
 * 
 * @param datatable_handle data-table handle.
 * @param index sample data-table column index.
 * @param value int16 data-type sample to process.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_push_int16_sample(datatable_handle_t datatable_handle, uint8_t index, int16_t value);

/**
 * @brief Processes data-table samples on the data buffer stack in each column based on the data-table's  
 * configured processing interval. When the samples are processed, the data buffer stack is cleared 
 * for each column.  This function must be called after data-table samples are pushed in the sampling task.
 * 
 * If the sampling period exceeds the data-table's configured sampling interval, a skipped task event is
 * recorded, and data-table may not process samples as expected.
 * 
 * @param datatable_handle data-table handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_process_samples(datatable_handle_t datatable_handle);

/**
 * @brief Renders a data-table to a textual representation in JSON format.
 * 
 * JSON data-table output example;
 * 
 * {
        "name": "Tbl_1-Min",
        "process-interval":     "minute",
        "process-period":       1,
        "columns":      [{
                        "index":        0,
                        "name": "Record ID",
                        "data-type":    "id",
                        "process-type": "sample"
                }, {
                        "index":        1,
                        "name": "TS",
                        "data-type":    "ts",
                        "process-type": "sample"
                }, {
                        "index":        2,
                        "name": "Pa_1-Min_Avg",
                        "data-type":    "float",
                        "process-type": "average"
                }, {
                        "index":        3,
                        "name": "Ta_1-Min_Avg",
                        "data-type":    "float",
                        "process-type": "average"
                }, {
                        "index":        4,
                        "name": "Ta_1-Min_Min",
                        "data-type":    "float",
                        "process-type": "minimum"
                }, {
                        "index":        5,
                        "name": "Ta_1-Min_Max",
                        "data-type":    "float",
                        "process-type": "maximum"
                }, {
                        "index":        6,
                        "name": "Td_1-Min_Avg",
                        "data-type":    "float",
                        "process-type": "average"
                }, {
                        "index":        7,
                        "name": "Rh_1-Min_Avg",
                        "data-type":    "float",
                        "process-type": "average"
                }],
        "rows": [{
                        "index":        0,
                        "columns":      [{
                                        "index":        0,
                                        "data-type":    "id",
                                        "value":        1
                                }, {
                                        "index":        1,
                                        "data-type":    "ts",
                                        "value":        0
                                }, {
                                        "index":        2,
                                        "data-type":    "float",
                                        "value":        1018.5899047851562
                                }, {
                                        "index":        3,
                                        "data-type":    "float",
                                        "value":        23.547393798828125
                                }, {
                                        "index":        4,
                                        "data-type":    "float",
                                        "value":        23.552894592285156
                                }, {
                                        "index":        5,
                                        "data-type":    "float",
                                        "value":        23.552894592285156
                                }, {
                                        "index":        6,
                                        "data-type":    "float",
                                        "value":        10.899570465087891
                                }, {
                                        "index":        7,
                                        "data-type":    "float",
                                        "value":        44.939468383789062
                                }]
                }]
}
 * 
 * @param[in] datatable_handle data-table handle.
 * @param[out] datatable data-table in JSON format.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datatable_to_json(datatable_handle_t datatable_handle, char *datatable);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __DATATABLE_H__
