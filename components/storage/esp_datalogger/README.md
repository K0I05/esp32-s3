# ESP Data-Logger Component
The ESP data-logger component simplifies in-situ data processing and recording for measurement and control use-cases.  The ESP data-logger component integrates with FreeRTOS and synchronizes tasks with the system clock for temporal based task execution with a typical resolution of approximately +/- 50 micro-seconds.  If you have applied experience (i.e. systems design, systems integrations, instrumentation, etc.) with commercial measurement and control products for use-cases that require on-board real-time analytics for in-situ processing, then you are most likely familiar with the terms used in the ESP data-logger component and know how it can be applied.

The ESP data-logger component includes the following common helper components:

1. **Data-Table**: table based data storage with user-defined columns, scalar and vector data-types, and basic analitics (i.e. average, minimum, and maximum).
2. **Time Into Interval**: synchronizes a FreeRTOS task with the system clock with user-defined time interval for temporal conditional scenarios.

The working example included was developed in Visual Studio Code and requires an internet connection to synchronize the system clock using Simple Network Time Protocol (SNTP) over Wi-Fi.  Likewise, you can comment out the Wi-Fi connect and ntp start routines and enable the get and set time routines.  The set time routine provides a user configurable date and time structure to initliaze the system clock to a base date-time.  Otherwise, the included example may not run as expected.

> Please keep in mind that this is still under development but if you have any suggestions or would like to contribute please contact me.

## Data-Table Example
Declare the data-table's handle, data-table configuration, and column indexes for columns that will be added.  A data-table will have one task-schedule handle declared for sampling purposes and one time-into-interval handle declared for processing purpose and referenced internally to manage temporal based sampling and processing within the data-table.

```
// data-table variables for the example
static datatable_handle_t       dt_1min_hdl;          /* example data-table handle */
static datatable_config_t       dt_1min_cfg = {       /* example data-table configuration */
    .name                       = "1min_tbl",
    .data_storage_type          = DATATABLE_DATA_STORAGE_MEMORY_RING,
    .columns_size               = 5,
    .rows_size                  = 10,
    .sampling_config            = {
        .interval_type          = TIME_INTO_INTERVAL_SEC,
        .interval_period        = 10,
        .interval_offset        = 0
    },
    .processing_config          = {
        .interval_type          = TIME_INTO_INTERVAL_MIN,
        .interval_period        = 1,
        .interval_offset        = 0
    }
};
static uint8_t              dt_1min_pa_avg_col_index;     /* data-table average atmospheric pressure (pa-avg) column index reference */
static uint8_t              dt_1min_ta_avg_col_index;     /* data-table average air temperature (ta-avg) column index reference */
static uint8_t              dt_1min_ta_max_col_index;     /* data-table minimum air temperature (ta-min) column index reference */
static uint8_t              dt_1min_ta_min_col_index;     /* data-table maximum air temperature (ta-max) column index reference */
static uint8_t              dt_1min_td_avg_col_index;     /* data-table average dew-point temperature (td-avg) column index reference */
```

The textual name of the data-table cannot exceed 15-characters, a data-table can have a maximum of 255 columns and a maximum of 65535 rows, size of columns and rows must be larger than 0.  The data-table sampling interval must be smaller than the data-table processing interval.  For this example, the data-table data storage type is configured to ring memory, first-in-first-out (FIFO), which will pop the oldest record in the data-table for the next record when the data-table becomes full.

> In this case, the data-table sampling task will execute once every 10-seconds with 0-seconds into the interval (i.e. 12:00:00, 12:00:10, 12:00:20, etc.), and the data-table will process samples in the data-table column data buffer once every minute.  

> The example averages, and determines the minimum and maximum, from the samples in data-table column data buffer over a one minute period.

> The first column added to the data-table will return an index of 2 because the record identifier and timestamp columns are automatically added when the data-table is created.

```
// create a new data-table handle for the example
datatable_new(&dt_1min_cfg, &dt_1min_hdl);   
if (dt_1min_hdl == NULL) {
    ESP_LOGE(APP_TAG, "datatable_new, new data-table handle failed");
    abort();
}

// configure data-table columns
//
// add float average column to data-table
datatable_add_float_avg_column(dt_1min_hdl, "Pa_1-Min", &dt_1min_pa_avg_col_index); // column index 2
// add float average column to data-table
datatable_add_float_avg_column(dt_1min_hdl, "Ta_1-Min", &dt_1min_ta_avg_col_index); // column index 3
// add float minimum column to data-table
datatable_add_float_min_column(dt_1min_hdl, "Ta_1-Min", &dt_1min_ta_min_col_index); // column index 4
// add float maximum column to data-table
datatable_add_float_max_column(dt_1min_hdl, "Ta_1-Min", &dt_1min_ta_max_col_index); // column index 5
// add float average column to data-table
datatable_add_float_avg_column(dt_1min_hdl, "Td_1-Min", &dt_1min_td_avg_col_index); // column index 6
```

The task execution time is accounted for in the data-table sampling task delay sub-routine (`datatable_sampling_task_delay`).  If the data-table sampling task duration exceeds the data-table sampling interval, a skipped sampling event will be generated, indicating that data-table was unable to process the samples within the defined sampling interval.  This is an indication that the data-table sampling task takes longer to execute then the configured sampling interval and the data-table sampling interval must be increased to avoid skipped samples and/or records.

The final step is to push samples into the data-table's data buffer stack, process the samples, and store the record.  In this example, i.e. 10-second sampling and a 1-min storage interval is configured, a total of 6 samples must be pushed onto the data-table's buffer stack for a processing period to be valid.  Otherwise, the data-table's data buffer stack is purged, record is skipped, and the next sampling period will restart based on the data-table's configured processing interval.

```
static void dt_1min_smp_task( void *pvParameters ) {
    for ( ;; ) {
        /* delay data-table sampling task until sampling interval has elapsed */
        datatable_sampling_task_delay(dt_1min_hdl);

        /* measure samples from sensors and set sensor variables (pa, ta, td)  */

        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_1min_hdl, dt_1min_pa_avg_col_index, pa_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_avg_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_min_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_max_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_td_avg_col_index, td_samples[samples_index]);

        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_1min_hdl);
    }

    // free up task resources
    vTaskDelete( NULL );
}
```

The data-table records can be extracted by row index or the entire data-table can be rendered to json format.  

```
// print data-table as a string in json format
//
// create root object for data-table
cJSON *dt_1min_json = cJSON_CreateObject();
// convert the data-table to json object
datatable_to_json(dt_1min_hdl, &dt_1min_json);
// render json data-table object to text and print
char *dt_1min_json_str = cJSON_Print(dt_1min_json);
ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_1min_json_str);
// free-up json resources
cJSON_free(dt_1min_json_str);
cJSON_Delete(dt_1min_json);
```
Sample data-table in json format:
```
{
        "name": "1min_tbl",
        "process-interval":     "minute",
        "process-period":       1,
        "columns":      ["Record ID", "TS", "Pa_1-Min_Avg", "Ta_1-Min_Avg", "Ta_1-Min_Min", "Ta_1-Min_Max", "Hr_1-Min_Avg", "Td_1-Min_Avg", "Wd_1-Min_Avg", "Ws_1-Min_Avg"],
        "types":        ["id", "ts", "float", "float", "float", "float", "int16", "float", "float", "float"],
        "processes":    ["sample", "sample", "average", "average", "minimum", "maximum", "average", "average", "average", "average"],
        "rows": [
		[1, 61684380120, 1001.3500366210938, 22.350000381469727, 22.340000152587891, 22.360000610351562, 51, 20.350000381469727, 210, 1.4500000476837158], 
		[2, 61684380180, 1001.3533325195312, 22.35333251953125, 22.340000152587891, 22.360000610351562, 53, 20.353334426879883, 210, 1.4500000476837158], 
		[3, 61684380240, 1001.3583374023438, 22.358335494995117, 22.350000381469727, 22.3700008392334, 55, 20.358333587646484, 210, 1.4500000476837158], 
		[4, 61684380300, 1001.3417358398438, 22.341667175292969, 22.329999923706055, 22.350000381469727, 57, 20.341667175292969, 210, 1.4500000476837158]]
}
```

See data-table component and review documentation on features implemented to date.

## Time-Into-Interval Example
Let's extend the **Data-Table Example**, declare a time-into-interval handle within the data-table sampling task sub-routine, and create a time-into-interval instance.  The time-into-interval instance interval is every 5-minutes with 0-minutes into the interval.

The next step is declaring a conditional statement leveraging the `time_into_interval` function.  This function returns true when the configured interval condition is valid, otherwise, it returns false.  In this example, the time-into-interval function will output the data-table in json format every 5-minutes with a 10-second offset into the interval (i.e. 12:00:10, 12:05:10, 12:10:10, etc.).

```
static void dt_1min_smp_task( void *pvParameters ) {
    time_into_interval_handle_t dt_1min_tii_5min_hdl;
    time_into_interval_config_t dt_1min_tii_5min_cfg = {
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 5 * 60,
        .interval_offset    = 10
    };

    // create a new time-into-interval handle - task system clock synchronization
    time_into_interval_new(&dt_1min_tii_5min_cfg, &dt_1min_tii_5min_hdl);
    if (dt_1min_tii_5min_hdl == NULL) ESP_LOGE(APP_TAG, "time_into_interval_new, new time-into-interval handle failed"); 
    

    for ( ;; ) {
        /* delay data-table sampling task until sampling interval has lapsed */
        datatable_sampling_task_delay(dt_1min_hdl);

        /* get measurement samples from sensors and set sensor variables (pa and ta)  */

        // push samples onto the data buffer stack for processing
        datatable_push_float_sample(dt_1min_hdl, dt_1min_pa_avg_col_index, pa_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_avg_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_min_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_ta_max_col_index, ta_samples[samples_index]);
        datatable_push_float_sample(dt_1min_hdl, dt_1min_td_avg_col_index, td_samples[samples_index]);

        // process data buffer stack samples (i.e. data-table's configured processing interval)
        datatable_process_samples(dt_1min_hdl);

        /* serialize data-table and output in json format every 5-minutes (i.e. 12:00:00, 12:05:00, 12:10:00, etc.) */
        if(time_into_interval(dt_1min_tii_5min_hdl)) {
            // create root object for data-table
            cJSON *dt_1min_json = cJSON_CreateObject();

            // convert the data-table to json object
            datatable_to_json(dt_1min_hdl, &dt_1min_json);

            // render json data-table object to text and print
            char *dt_1min_json_str = cJSON_Print(dt_1min_json);
            ESP_LOGI(APP_TAG, "JSON Data-Table:\n%s",dt_1min_json_str);

            // free-up json resources
            cJSON_free(dt_1min_json_str);
            cJSON_Delete(dt_1min_json);
        }
    }

    // free up task resources
    time_into_interval_del( dt_1min_tii_5min_hdl ); //delete time-into-interval handle
    vTaskDelete( NULL );
}
```

You can declare as many time-into-interval handles as needed, or as memory permits, to sychronize real-time events with the system clock.  See time-into-interval component and review documentation on features implemented to date.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
