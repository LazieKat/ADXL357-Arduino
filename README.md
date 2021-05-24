# ADXL357-Arduino
A library to operate the ADXL357 accelerometer on Arduino-compatible microcontrollers. 
This library is based on [Seeed Technology Inc. ADXL357b library](https://github.com/Seeed-Studio/Seeed_ADXL357B).

This version has the following changes:
* Doxygen comments for readability with references to datasheet pages
* Correct defines for all register addresses
* Defines for power control options
* Wire as a define to allow usage of Wire1 and Wire2
* Force user to enter device address for Adxl357.begin() to ensure correct ASEL pin setting
* A private calibration variable with a setter and scaled mearments get function
* Remove redundant I2C read and write functions
* Remove unnecessary enums
* Remove duplicated XYZ read function and stick to FIFO reads only for accuracy
* Minor refactoring