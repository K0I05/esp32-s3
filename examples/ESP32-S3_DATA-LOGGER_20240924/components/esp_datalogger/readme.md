# ESP Data-Logger Component
The ESP data-logger component simplifies in-situ data processing and recording for measurement and control use-cases.  The ESP data-logger component integrates with FreeRTOS and synchronizes tasks with the system clock for temporal based task execution with a resolution of +/- 100 micro-seconds.

The ESP data-logger component includes the following common helper components:

1. **Data-Table**: table based data storage with user-defined columns, data-types, and analitics.
2. **Task Schedule**: synchronizes a FreeRTOS task with the system clock with user-defined time interval.
3. **Time Into Interval**: synchronizes user-defined time interval with the system clock for temporal conditional scenarios.

## Data-Table Example
Declare the data-table handle, configuration, and column indexes for columns that will be added.
```
static datatable_handle_t dt_sample_hdl;
static datatable_config_t dt_sample_cfg = {
        .columns_size               = 4, 
        .rows_size                  = 10, 
        .sampling_interval_type     = DATALOGGER_TIME_INTERVAL_SEC, 
        .sampling_interval_period   = 10, 
        .processing_interval_type   = DATALOGGER_TIME_INTERVAL_MIN, 
        .processing_interval_period = 1, 
        .data_storage_type          = DATATABLE_DATA_STORAGE_MEMORY_RING
    };
static uint8_t dt_pa_avg_column_index;
static uint8_t dt_ta_avg_column_index;
static uint8_t dt_ta_max_column_index;
static uint8_t dt_ta_min_column_index;
```

The next step is to create the data-table handle and add columns to the data-table.

> The first colunm added to the data-table will return an index of 2 because the record identifier and timestamp columns are added when the data-table is created.
```
// create new data-table handle
datatable_new("Tbl_1-Min", &dt_sample_cfg, &dt_sample_hdl);

// add float average column to data-table
datatable_add_float_avg_column(dt_sample_hdl, "Pa_1-Min", &dt_pa_avg_column_index);                 // column index 2
// add float average column to data-table
datatable_add_float_avg_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_avg_column_index);                 // column index 3
// add float minimum column to data-table
datatable_add_float_min_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_min_column_index);                 // column index 4
// add float maximum column to data-table
datatable_add_float_max_column(dt_sample_hdl, "Ta_1-Min", &dt_ta_max_column_index); 
```

The next step is to synchronize the FreeRTOS task with the system clock by delcaring a task schedule handle and creating a new task schedule handle.

> In this case the task will excute once every 10-seconds with 0-seconds into the interval (i.e. 12:00:00, 12:00:10, 12:00:20, etc.).

The task execution time is accounted for in the task schedule delay.  If the task duration exceeds the task interval period, a skipped task event will be generated, indicated that data-table was unable to process the samples.

The final step is to push samples into the data-table and process the samples.
```
static void sample_task( void *pvParameters ) {
    task_schedule_handle_t      task_schedule_hdl;

    // instantiate a new task schedule handle - task system clock synchronization
    task_schedule_new(DATALOGGER_TIME_INTERVAL_SEC, 10, 0, &task_schedule_hdl);
    if (task_schedule_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "task_new_schedule new task schedule handle failed");

    for ( ;; ) {
        // delay task until task schedule condition is valid
        task_schedule_delay(task_schedule_hdl);

        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_sample_hdl, dt_pa_avg_column_index, pa);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_avg_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_min_column_index, ta);
        datatable_push_float_sample(dt_sample_hdl, dt_ta_max_column_index, ta);

        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_sample_hdl);
    }
}
```

Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
