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
 * @file datalogger.h
 * @defgroup datalogger
 * @{
 *
 * ESP-IDF datalogger
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DATALOGGER_H__
#define __DATALOGGER_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <esp_err.h>

#include <datatable.h>
#include <task_schedule.h>
#include <time_into_interval.h>

#ifdef __cplusplus
extern "C" {
#endif


/*
 * ESP DATA-LOGGER definitions
 */
#define DATALOGGER_NAME_SIZE                    (15)    //!< a maximum of 15-characters for user-defined name of data-logger handle
#define DATALOGGER_DT_HANDLES_MAXIMUM           (5)     //!< a maximum of 5 user-defined data-table handles for a data-logger handle
#define DATALOGGER_TTI_HANDLES_MAXIMUM          (5)     //!< a maximum of 5 user-defined time-into-interval handles for a data-logger handle
#define DATALOGGER_TS_HANDLES_MAXIMUM           (5)     //!< a maximum of 5 user-defined task-schedule handles for a data-logger handle

/**
 * @brief Data-logger structure.
 */
typedef struct datalogger_t datalogger_t;

 /**
  * @brief Data-logger handle structure.
  */
typedef struct datalogger_t *datalogger_handle_t;

/**
 * @brief Data-logger state object structure definition.  Do not modify these fields once the
 * data-logger handle is created, these are read-only, and represent a state machine.
 */
struct datalogger_t {
    char                            name[DATALOGGER_NAME_SIZE];       /*!< data-logger textual name, 15-characters maximum */
    //datatable_handle_t              system_datatable_handle;          /*!< data-logger system data-table handle, initialized when data-logger handle is created */
    uint8_t                         datatable_handles_count;          /*!< data-logger data-tables handles count, initialized to 0 when data-logger handle is created */
    datatable_handle_t             *datatable_handles;                /*!< data-logger array of referenced data-table handles, initialized when data-logger handle is created */
    uint8_t                         time_into_interval_handles_count; /*!< data-logger time-into-interval handles count, initialized to 0 when data-logger handle is created */
    time_into_interval_handle_t    *time_into_interval_handles;       /*!< data-logger array of referenced time-into-interval handles, initialized when data-logger handle is created */
    uint8_t                         task_schedule_handles_count;      /*!< data-logger task-schedule handles count, initialized to 0 when data-logger handle is created */
    task_schedule_handle_t         *task_schedule_handles;            /*!< data-logger array of referenced task-schedule handles, initialized when data-logger handle is created */
};



/**
 * @brief Creates a new data-logger handle and only one handle should be created.
 * 
 * @param name Data-logger textual name, 15-character maximum.
 * @param datalogger_handle Data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_new(const char *name, datalogger_handle_t *datalogger_handle);

/**
 * @brief Gets the data-logger system data-table handle by the referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the system data-table handle of interest.
 * @param[out] datatable_handle Data-logger handle reference to the system data-table handle of interest.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_system_datatable(datalogger_handle_t datalogger_handle, datatable_handle_t *system_datatable_handle);


/**
 * @brief Creates a new data-table handle that is referenced to the data-logger handle.
 * 
 * @param[in] name Data-logger data-table textual name, 15-characters maximum.
 * @param[in] columns_size Data-logger data-table column array size, this setting cannot be 0.
 * @param[in] rows_size Data-logger data-table row array size,this setting cannot be 0.
 * @param[in] sampling_interval_type Data-logger data-table sampling time interval type setting.
 * @param[in] sampling_interval_period Data-logger data-table sampling time interval nonzero period, per interval type setting.
 * @param[in] sampling_interval_offset Data-logger data-table sampling time interval offset, per interval type setting.
 * @param[in] processing_interval_type Data-logger data-table sampling time interval type setting.
 * @param[in] processing_interval_period Data-logger data-table processing time interval nonzero period, per interval type setting, of samples to process and store data-table records.
 * @param[in] processing_interval_offset Data-logger data-table processing time interval offset, per interval type setting, of samples to process and store data-table records.
 * @param[in] data_storage_type Data-logger data-table data storage type, defines handling of records when the data-table is full.
 * @param[in] datalogger_handle Data-logger handle to reference the data-table handle.
 * @param[out] index Index of the new data-table handle that will be referenced to the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_new_datatable(const char *name, const uint8_t columns_size, const uint16_t rows_size, 
                    const datalogger_time_interval_types_t sampling_interval_type, const uint16_t sampling_interval_period, const uint16_t sampling_interval_offset,
                    const datalogger_time_interval_types_t processing_interval_type, const uint16_t processing_interval_period, const uint16_t processing_interval_offset, 
                    const datatable_data_storage_types_t data_storage_type, datalogger_handle_t datalogger_handle, uint8_t *index);

/**
 * @brief Gets a data-table handle by index and referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the data-table handle of interest.
 * @param[in] index Index of the data-table handle referenced to the data-logger handle.
 * @param[out] datatable_handle Data-logger handle reference to the data-table handle of interest.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_datatable(datalogger_handle_t datalogger_handle, uint8_t index, datatable_handle_t *datatable_handle);

/**
 * @brief Gets a count of data-table handles referenced by the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the data-table handles of interest.
 * @param[out] count Number of data-table handles created and referenced in the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_datatables_count(datalogger_handle_t datalogger_handle, uint8_t *count);

//esp_err_t datalogger_rm_datatable(datalogger_handle_t datalogger_handle, const uint8_t index);


/**
 * @brief Creates a new time-into-interval handle that is referenced to the data-logger handle.
 * 
 * @param[in] interval_type Data-logger time-into-interval, interval type setting.
 * @param[in] interval_period Data-logger time-into-interval, a non-zero interval period setting per interval type setting.
 * @param[in] interval_offset Data-logger time-into-interval, interval offset setting, per interval type setting, that must be less than the interval period.
 * @param[in] datalogger_handle Data-logger handle to reference the time-into-interval handle.
 * @param[out] index Index of the new time-into-interval handle that will be referenced to the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_new_time_into_interval(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset,
                                        datalogger_handle_t datalogger_handle, uint8_t *index);

/**
 * @brief Gets a time-into-interval handle by index and referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the time-into-interval handle of interest.
 * @param[in] index Index of the time-into-interval handle referenced to the data-logger handle.
 * @param[out] time_into_interval_handle Data-logger handle reference to the time-into-interval handle of interest.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_time_into_interval(datalogger_handle_t datalogger_handle, const uint8_t index, time_into_interval_handle_t *time_into_interval_handle);

/**
 * @brief Gets a count of time-into-interval handles referenced by the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the time-into-interval handles of interest.
 * @param[out] count Number of time-into-interval handles created and referenced in the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_time_into_intervals_count(datalogger_handle_t datalogger_handle, uint8_t *count);

//esp_err_t datalogger_rm_time_into_interval(datalogger_handle_t datalogger_handle, const uint8_t index);


/**
 * @brief Creates a new task-schedule handle that is referenced to the data-logger handle.
 * 
 * @param[in] interval_type Data-logger task-schedule, interval type setting.
 * @param[in] interval_period Data-logger task-schedule, a non-zero interval period setting per interval type setting.
 * @param[in] interval_offset Data-logger task-schedule, interval offset setting, per interval type setting, that must be less than the interval period.
 * @param[in] datalogger_handle Data-logger handle to reference the task-schedule handle.
 * @param[out] index Index of the new task-schedule handle that will be referenced to the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_new_task_schedule(const datalogger_time_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset,
                                        datalogger_handle_t datalogger_handle, uint8_t *index);

/**
 * @brief Gets a task-schedule handle by index and referenced data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the task-schedule handle of interest.
 * @param[in] index Index of the task-schedule handle referenced to the data-logger handle.
 * @param[out] task_schedule_handle Data-logger handle reference to the task-schedule handle of interest.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_task_schedule(datalogger_handle_t datalogger_handle, const uint8_t index, task_schedule_handle_t *task_schedule_handle);

/**
 * @brief Gets a count of task-schedule handles referenced by the data-logger handle.
 * 
 * @param[in] datalogger_handle Data-logger handle reference for the task-schedule handles of interest.
 * @param[out] count Number of task-schedule handles created and referenced in the data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_get_task_schedules_count(datalogger_handle_t datalogger_handle, uint8_t *count);

//esp_err_t datalogger_rm_task_schedule(datalogger_handle_t datalogger_handle, const uint8_t index);


/**
 * @brief Deletes the data-logger handle and frees up resources.
 * 
 * @param datalogger_handle Data-logger handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t datalogger_del(datalogger_handle_t datalogger_handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __DATALOGGER_H__
