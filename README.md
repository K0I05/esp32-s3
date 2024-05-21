# esp32-s3
 Peripheral device drivers with examples for the ESP32-S3 chipset.  This is a revised release utilizing esp-idf suggested design patterns through `handles`.
 
 I have to give credit where credit is due, thank you Sheinz (https://github.com/sheinz>), Ruslan (<unclerus@gmail.com>), mistak1992 (526337554@qq.com), and others where your code helped very much throughout this journey.

# Peripheral Drivers
 Supported drivers include the following device peripherals:
 
 - ADC: GUVA-S12SD
 - I2C: ROHM BH1750FVI
 - I2C: Melexis MLX90614
 - I2C: Sensirion SHT4x
 
 Above peripheral drivers have been tested and validated with a logic analyzer.  If any problems arise please feel free to create a bug ticket.

# WIP Peripheral Drivers
 Peripherial device drivers that are work in progress (WIP) include:

 - Bosch BMP280 (to be posted)
 - Honeywell HMC5883L (https://github.com/K0I05/esp32-s3/issues/1)

 Seeking assistance with above WIP drivers given they currently crash core and I am unable to decode the backtrace exception.  Any assistance or guidance would be greatly appreciated.