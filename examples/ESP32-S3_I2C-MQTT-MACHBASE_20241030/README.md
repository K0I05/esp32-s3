# ESP32-S3 I2C to MQTT to MACHBASE
This is a fully working application that polls a BMP280 and SHT40 MEMS sensors over I2C, samples are queued, and then transmitted to a **MACHBASE** instance that resides on a local network.  The device connects to the local network over WIFI, synchronizes the system clock through SNTP services, and publishes data to the **MACHBASE** MQTT broker at a 3-second interval.  

See 'network_connect.c' for WIFI and MQTT connection configuration parameters.  Just ensure that the 'NET_DEVICE_ID' matches the 'MQTT_BROKER_CLIENT_ID' and that you are publishing to the correct 'topic'.  See **MACHBASE** helpfiles but it is basically the database name that you create.

For this example, all have to do is create a 'TAG' table with the following columns: NAME (varchar 150), TIMESTAMP (datetime), VALUE (double), PARAMETER (varchar 100), and DEVICE_ID (varchar 50).  The MQTT message format published to **MACHBASE** over MQTT looks like the following:

```
    ["ca.nb.01-1000.Air-Temperature",1729957661187888000,1002.928162,"Air-Temperature", "ca.nb.aws.01-1000"] 
```

Likewise, lookup tables can be created as well for category or code based parameters.  See 'SQL_[name]_Create.sql' files for more information.

## MACHBASE Time-Series Database
**MACHBASE** is a time-series database that is designed to manage temporal data, needless to say, it is ideal for Internet of Things (IoT) use-cases and can handle highspeed sampling (e.g., much faster than a 3-second interval).  Time-series databases are relatively new, commercially available, and are amazingly faster than legacy relational databases.  **MACHBASE** has some of the highest performance metrics in the industry and they offer it as a community based release but you do have to buy a license for commercial use-cases.  However, if you enjoy tinkering and looking to store and visualize your data, then look no further and give **MACHBASE** a try.  **MACHBASE** is loaded with features for both data manipulation, transformation, management and visualization, and much more.  It leverages Apache eCharts and they offer a language called TQL that is out of this world for data scientists or data engineers but even the average joe can build impressive charts to showcase your work.

## Updates
I plan to post more **MACHBASE** examples once I have them completed.  Their help files are okay but if you are like myself, I learn from real world examples i.e. applied, and will share what I can shortly.  If you have any questions or would like to contribute feel free to conact me.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
