# Welcome
Thanks for visiting and hope you find something useful.  The code base is maintained as well as one person can manage in their spare time. The development environment is under Visual Studio Code with the PlatformIO (6.1.16) and ESP-IDF (v5.3.1) extensions.  There is always room for improvement to optimize the code base and open to suggestions.

# esp32-s3
Peripheral device drivers with examples for the ESP32-S3 chipset.  This is a revised release utilizing esp-idf suggested design patterns through `handles` and using `i2c_master.h` for I2C transactions.  The drivers are organized in the components folder within the PlatformIO environment.

The folder structure for drivers, and its associated example, are outlined as follows:
```
|--components
|  |--peripherals
|  | 
|  |  |--adc
|  |  |--i2c
|  |  |  |--esp_bmp_280
|  |  |  |  |- BMP280 Datasheet.pdf
|  |  |  |  |- bmp280.c
|  |  |  |  |- bmp280.h
|  |  |  |  |- CMakeList.txt
|  |  |  |  |- LICENSE
|  |  |
|  |  |--spi
|  |
|  |--sensirion_gas_index_algorithm
|  |--storage
|
|--include
|  |- app_config.h
|  |- bmp280_task.h
|
|--src
|  |- bmp280_task.c
|  |- main.c
|
|- platformio.ini
```
To get started, locate and open the `app_config.h` file from the `include` folder and configure GPIO pins as needed.  Now, locate and open the `main.c` file from the `src`folder and go to the `void app_main( void )` subroutine to enable the device of interest.
```
/**
 * @brief Main application entry point.
 */
void app_main( void ) {
    /* start a component example */
    /* note: only one component can run at a time */
    
    //i2c0_component_example_start(I2C_COMPONENT_AHTXX);
    //i2c0_component_example_start(I2C_COMPONENT_AS7341);
    //i2c0_component_example_start(I2C_COMPONENT_BH1750);
    //i2c0_component_example_start(I2C_COMPONENT_BMP280);
    i2c0_component_example_start(I2C_COMPONENT_BMP390);
    //i2c0_component_example_start(I2C_COMPONENT_CCS811);
    //i2c0_component_example_start(I2C_COMPONENT_ENS160);
    //i2c0_component_example_start(I2C_COMPONENT_HDC1080);
    //i2c0_component_example_start(I2C_COMPONENT_HMC5883L);
    //i2c0_component_example_start(I2C_COMPONENT_MLX90614);
    //i2c0_component_example_start(I2C_COMPONENT_MPU6050);
    //i2c0_component_example_start(I2C_COMPONENT_SGP4X);
    //i2c0_component_example_start(I2C_COMPONENT_SHT4X);
    //i2c0_component_example_start(I2C_COMPONENT_SSD1306);
    //i2c0_component_example_start(I2C_COMPONENT_TLV493D);
    //i2c0_component_example_start(I2C_COMPONENT_VEML7700);
}
```
Once these initial steps are done, compile and upload the program, assuming you development board is equivalent to the `esp32s3box`.  Otherwise, you will have to configure the environment for your development board and recompile before uploading the program.

# Peripheral Drivers
Supported drivers include the following device peripherals:
 
 - ADC: GUVA-S12SD
 - I2C: ASAIR AHTXX
 - I2C: AKM AK8975 - Work in Progress
 - I2C: OSRAM AS7341
 - I2C: ROHM BH1750FVI
 - I2C: Bosch BME680 - Work in Progress, Coming Soon!! 
 - I2C: Bosch BMP280
 - I2C: Bosch BMP390
 - I2C: ScioSense CCS811
 - I2C: ScioSense ENS160
 - I2C: Texas Instruments HDC1080
 - I2C: Honeywell HMC5883L
 - I2C: Melexis MLX90614
 - I2C: InvenSense MPU6050
 - I2C: Sensirion SGP4x
 - I2C: Sensirion SHT4x
 - I2C: Generic SSD1306 (128x32 and 128x64)
 - I2C: Infineon TLV493D
 - I2C: Vishay VEML7700
 - SPI: Analog Devices MAX31865 - Work in Progress
 
Above peripheral drivers have been tested, validated with a logic analyzer where applicable, and still under development. With every ESP-IDF release there are bound to be quirks with the code base, a major one was with the release of ESP-IDF (v5.3.1), the i2c_master.h has introduced timing issues and above drivers did require some maintenance.  If any problems arise please feel free to log an issue and if you would to contribute please contact me.

# ESP Data-Logger
A user friendly table based data logging for measurement and control use-cases.  See Data-Logger examples for more details, see readme file in the component folder.



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)