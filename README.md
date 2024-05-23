# esp32-s3
 Peripheral device drivers with examples for the ESP32-S3 chipset.  This is a revised release utilizing esp-idf suggested design patterns through `handles`.
 
 I have to give credit where credit is due, thank you Sheinz (https://github.com/sheinz>), Ruslan (<unclerus@gmail.com>), mistak1992 (526337554@qq.com), and others where your code helped very much throughout this journey.

# Peripheral Drivers
 Supported drivers include the following device peripherals:
 
 - ADC: GUVA-S12SD
 - I2C: ROHM BH1750FVI
 - I2C: Melexis MLX90614
 - I2C: Sensirion SHT4x
 - I2C: Bosch BMP280
 
 Above peripheral drivers have been tested and validated with a logic analyzer.  If any problems arise please feel free to log an issue.

# WIP Peripheral Drivers
 Peripherial device drivers that are work in progress (WIP) include:

 - Honeywell HMC5883L
