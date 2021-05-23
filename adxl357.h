/*
MIT License

Copyright (c) 2021 Seeed Technology Inc. & Ahmad Ziad Zain Aldeen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef __ADXL357_H__
#define __ADXL357_H__


////    Includes    ////
#include <Arduino.h>
#include <Wire.h>


////    Defines    ////
// I2C settings
#define ADXL357_WIRE                Wire    // Change to Wire1 or Wire2 if needed
#define ADXL357_ID                  0xED

// Device adresses
#define ADXL357_DEF_ADD             0x1D    // When MISO is set to 0
#define ADXL357_ALT_ADD             0x53    // When MISO is set to 1

// Ranges                                   // Datasheet P.39
#define ADXL357_TEN_G               0b01
#define ADXL357_TWENTY_G            0b10
#define ADXL357_FOUTY_G             0b11

// Power modes
#define ADXL357_DRDY_OFF            0x04
#define ADXL357_TEMP_OFF            0x02
#define ADXL357_STANDBY             0x01
#define ADXL357_ALL_ON              0x00

// Register addresses                       // Datasheet P.32
#define ADXL357_REG_DEVID_AD        0x00    /// \todo
#define ADXL357_REG_DEVID_MST       0x01    /// \todo
#define ADXL357_REG_PARTID          0x02
#define ADXL357_REG_REVID           0x03
#define ADXL357_REG_STATUS          0x04
#define ADXL357_REG_FIFO_ENTRIES    0x05
#define ADXL357_REG_TEMP2           0x06    /// \todo
#define ADXL357_REG_TEMP1           0x07
#define ADXL357_REG_XDATA3          0x08
#define ADXL357_REG_XDATA2          0x09
#define ADXL357_REG_XDATA1          0x0A
#define ADXL357_REG_YDATA3          0x0B
#define ADXL357_REG_YDATA2          0x0C
#define ADXL357_REG_YDATA1          0x0D
#define ADXL357_REG_ZDATA3          0x0E
#define ADXL357_REG_ZDATA2          0x0F
#define ADXL357_REG_ZDATA1          0x10
#define ADXL357_REG_FIFO_DATA       0x11
#define ADXL357_REG_OFFSET_X_H      0x1E    /// \todo
#define ADXL357_REG_OFFSET_X_L      0x1F
#define ADXL357_REG_OFFSET_Y_H      0x20
#define ADXL357_REG_OFFSET_Y_L      0x21
#define ADXL357_REG_OFFSET_Z_H      0x22
#define ADXL357_REG_OFFSET_Z_L      0x23
#define ADXL357_REG_ACT_EN          0x24
#define ADXL357_REG_ACT_THRESH_H    0x25
#define ADXL357_REG_ACT_THRESH_L    0x26
#define ADXL357_REG_ACT_COUNT       0x27
#define ADXL357_REG_FILTER          0x28
#define ADXL357_REG_FIFO_SAMPLES    0x29
#define ADXL357_REG_INT_MAP         0x2A
#define ADXL357_REG_SYNC            0x2B    /// \todo
#define ADXL357_REG_RANGE           0x2C
#define ADXL357_REG_POWER_CTL       0x2D
#define ADXL357_REG_SELF_TEST       0x2E    /// \todo
#define ADXL357_REG_RESET           0x2F


////    Adxl357 class    ////
class Adxl357
{
    public:
        Adxl357()  {};
        ~Adxl357() {};

        uint8_t begin                   (uint8_t addr);
        uint8_t reset                   ();
        bool    isDataReady             ();

        uint8_t getDeviceID             (uint8_t *result);
        uint8_t getDeviceVer            (uint8_t *result);
        uint8_t getStatus               (uint8_t *result);
        uint8_t getFIFOEntries          (uint8_t *result);
        uint8_t getRawAccelData         (int32_t *x, int32_t *y, int32_t *z);
        uint8_t getScaledAccelData      (double *x, double *y, double *z);
        uint8_t getActivityCount        (uint8_t *result);

        uint8_t setActivityEnable       (bool x, bool y, bool z);
        uint8_t setActivityThreashold   (int16_t threashold);
        uint8_t setActivityCount        (uint8_t count);
        uint8_t setPowerCTL             (uint8_t val);
        uint8_t setFilter               (uint8_t hpf, uint8_t odr_lpf);
        uint8_t setIntMap               (uint8_t val);
        uint8_t setAccelRange           (uint8_t range);
        uint8_t setCalibrationConstant  (double calib);

    private:
        uint8_t  _address;
        double   _calib = 1;

        uint8_t writeBytes              (uint8_t reg, uint8_t *data, uint32_t len);
        uint8_t readBytes               (uint8_t reg, uint8_t *dest, uint32_t len, uint32_t retries = 10);
};

#endif