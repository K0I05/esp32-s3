# Welcome
Thanks for visiting and hope you find something useful.  The code base is maintained as well as one person can manage in their spare time. The development environment is under Visual Studio Code with the PlatformIO (6.1.16) and ESP-IDF (v5.3.1) extensions.  There is always room for improvement to optimize the code base and open to suggestions.

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
 - I2C: AKM AK8975
 - I2C: ScioSense ENS160
 - I2C: Vishay VEML7700
 
Above peripheral drivers have been tested, validated with a logic analyzer where applicable, and still under development. With every ESP-IDF release there are bound to be quirks with the code base, a major one was with the release of ESP-IDF (v5.3.1), the i2c_master.h has introduced timing issues and above drivers did require some maintenance.  If any problems arise please feel free to log an issue and if you would to contribute please contact me.

# ESP Data-Logger
A user friendly table based data logging for measurement and control use-cases.  See Data-Logger examples for more details, see readme file in the component folder.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)