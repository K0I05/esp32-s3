# ESP Time-Into-Interval Component
The ESP time-into-interval component synchronizes a FreeRTOS task with the system clock with user-defined time interval for temporal conditional scenarios.

## Time-Into-Interval Example
See **Data-Table Example**, declare a time-into-interval handle within the data-table sampling task sub-routine, and create a time-into-interval instance.  The time-into-interval instance interval is every 5-minutes with 0-minutes into the interval.

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
