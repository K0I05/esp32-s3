# ESP Data-Logger Component
The ESP data-logger component simplifies in-situ data processing and recording for measurement and control use-cases.  The ESP data-logger component integrates with FreeRTOS and synchronizes tasks with the system clock for temporal based task execution with a typical resolution of approximately +/- 50 micro-seconds.  If have applied experience with commercial measurement and control products for use-cases that require on-board real-time analytics for in-situ processing, then you are most likely familiar with the terms used in the ESP data-logger component and know how it can be applied.

The ESP data-logger component includes the following common helper components:

1. **Data-Table**: table based data storage with user-defined columns, scalar and vector data-types, and analitics.
2. **Task Schedule**: synchronizes a FreeRTOS task with the system clock with user-defined time interval.
3. **Time Into Interval**: synchronizes user-defined time interval with the system clock for temporal conditional scenarios.

The working example included was developed in Visual Studio Code and requires an internet connection to synchronize the system clock using Simple Network Time Protocol (SNTP) over Wi-Fi.  Likewise, you can comment out the Wi-Fi connect and ntp start routines and enable the get and set time routines.  The set time routine provides a user configurable date and time structure to initliaze the system clock to a base date-time.  Otherwise, the included example may not run as expected.

## Data-Table Example
Declare the sampling task-schedule handle for the example data-table, data-table handle and column indexes for columns that will be added.  A data-table must have one task-schedule handle declared and referenced to manage internal temporal based processing within the associated data-table.

```
static task_schedule_handle_t dt_sampling_task_sch_hdl; /* task schedule handle for data-table sampling task */
static datatable_handle_t dt_example_hdl;               /* example data-table handle */
static uint8_t dt_pa_avg_column_index;                  /* data-table average atmospheric pressure (pa-avg) column index */
static uint8_t dt_ta_avg_column_index;                  /* data-table average air temperature (ta-avg) column index */
static uint8_t dt_ta_max_column_index;                  /* data-table minimum air temperature (ta-min) column index */
static uint8_t dt_ta_min_column_index;                  /* data-table maximum air temperature (ta-max) column index */
```

The next step is to synchronize the FreeRTOS task with the system clock by declaring a task schedule handle and creating a new task schedule handle for the sampling task.

The next step is to create the data-table handle and add columns to the data-table.  A data-table can have a maximum of 255 columns and a maximum of 65535 rows.

> In this case, the task will excute once every 10-seconds with 0-seconds into the interval (i.e. 12:00:00, 12:00:10, 12:00:20, etc.), and the data-table will process sample once every minute.

> The first colunm added to the data-table will return an index of 2 because the record identifier and timestamp columns are automatically added when the data-table is created.

```
/* create a new task schedule handle for the example data-table - task system clock synchronization */
task_schedule_new(DATALOGGER_TIME_INTERVAL_SEC, 10, 0, &dt_sampling_task_sch_hdl);
if (dt_sampling_task_sch_hdl == NULL) ESP_LOGE(APP_TAG, "task_schedule_new, new task-schedule handle failed");

/* create a new data-table handle for the example and reference the sampling task schedule handle */
datatable_new("Tbl_1-Min", 4, 10, DATALOGGER_TIME_INTERVAL_MIN, 1, 0, dt_sampling_task_sch_hdl, DATATABLE_DATA_STORAGE_MEMORY_RING, &dt_example_hdl);
if (dt_sample_hdl == NULL) ESP_LOGE(APP_TAG, "datatable_new, new data-table handle failed");

// add float average column to data-table
datatable_add_float_avg_column(dt_example_hdl, "Pa_1-Min", &dt_pa_avg_column_index); // column index 2
// add float average column to data-table
datatable_add_float_avg_column(dt_example_hdl, "Ta_1-Min", &dt_ta_avg_column_index); // column index 3
// add float minimum column to data-table
datatable_add_float_min_column(dt_example_hdl, "Ta_1-Min", &dt_ta_min_column_index); // column index 4
// add float maximum column to data-table
datatable_add_float_max_column(dt_example_hdl, "Ta_1-Min", &dt_ta_max_column_index); // column index 5
```

The task execution time is accounted for in the task schedule delay.  If the task duration exceeds the task interval period, a skipped task event will be generated, indicated that data-table was unable to process the samples within the defined sampling interval.  This is an indication that the task takes longer to execute then the defined sampling interval.

The final step is to push samples into the data-table's data buffer stack and process the samples.  In this example, i.e. 10-second sampling and a 1-min storage interval, a total of 6 samples must be pushed onto the data-table's buffer stack for a valid processing period.  Otherwise, the data-table's data buffer stack is purged, record is skipped, and the next sampling period will restart based on the processing interval.

```
static void dt_sampling_task( void *pvParameters ) {
    for ( ;; ) {
        /* delay data-table sampling task until task-schedule condition is valid */
        datatable_sampling_task_delay(dt_example_hdl);

        /* get measurement samples from sensors and set sensor variables (pa and ta)  */

        /* push samples onto the datatable's data buffer stack for processing */
        datatable_push_float_sample(dt_example_hdl, dt_pa_avg_column_index, pa);
        datatable_push_float_sample(dt_example_hdl, dt_ta_avg_column_index, ta);
        datatable_push_float_sample(dt_example_hdl, dt_ta_min_column_index, ta);
        datatable_push_float_sample(dt_example_hdl, dt_ta_max_column_index, ta);

        /* process data-table data buffer stack samples (i.e. data-table's configured processing interval) */
        datatable_process_samples(dt_example_hdl);
    }
}
```

The data-table records can be extracted by row index or the entire data-table can be rendered to json format.  

```
char *dt_json = "";
datatable_to_json(dt_example_hdl, dt_json);
ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_json);
```

See data-table component and review documentation on features implemented to date.

## Time-Into-Interval Example
Let's extend the **Data-Table Example**, declare a time-into-interval handle within the data-table sampling task sub-routine, and create a new time-into-interval instance.  The time-into-interval instance interval is every 5-minutes with 0-minutes into the interval.

The next step is declaring a conditional statement leveraging the `time_into_interval` function.  This function returns true when the configured interval condition is valid, otherwise, it returns false.  In this example, the time-into-interval function will output the data-table in json format every 5-minutes (i.e. 12:00:00, 12:05:00, 12:10:00, etc.).

```
static void dt_sampling_task( void *pvParameters ) {
    time_into_interval_handle_t time_into_interval_hdl; /* time-into-interval handle for data-table sampling task */

    /* create a new time-into-interval handle - task system clock synchronization */
    time_into_interval_new(DATALOGGER_TIME_INTERVAL_MIN, 5, 0, &time_into_interval_hdl);
    if (time_into_interval_hdl == NULL) ESP_LOGE(APP_TAG, "time_into_interval_new, new time-into-interval handle failed");

    for ( ;; ) {
        /* delay data-table sampling task until task-schedule condition is valid */
        datatable_sampling_task_delay(dt_example_hdl);

        /* get measurement samples from sensors and set sensor variables (pa and ta)  */

        /* push samples onto the data-table's data buffer stack for processing */
        datatable_push_float_sample(dt_example_hdl, dt_pa_avg_column_index, pa);
        datatable_push_float_sample(dt_example_hdl, dt_ta_avg_column_index, ta);
        datatable_push_float_sample(dt_example_hdl, dt_ta_min_column_index, ta);
        datatable_push_float_sample(dt_example_hdl, dt_ta_max_column_index, ta);

        /* process data-table data buffer stack samples (i.e. data-table's configured processing interval) */
        datatable_process_samples(dt_example_hdl);

        /* serialize data-table and output in json format every 5-minutes (i.e. 12:00:00, 12:05:00, 12:10:00, etc.) */
        if(time_into_interval(time_into_interval_hdl)) {
            char *dt_json = "";
            datatable_to_json(dt_example_hdl, dt_json);
	        ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_json);
        }
    }
}
```

You can declare as many time-into-interval handles as needed, or memory available, to sychronize real-time events with the system clock.  See time-into-interval component and review documentation on features implemented to date.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
