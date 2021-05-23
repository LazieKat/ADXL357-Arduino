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


#include "adxl357.h"


////    Public Functions    ////

/**
 * @brief Initilize ADXL357 sensor
 * 
 * @param addr the I2C adress of the devide
 * @return Zero on success
 */
uint8_t Adxl357::begin(uint8_t addr)
{
    uint8_t ID;

    _address = addr;        // object address
    ADXL357_WIRE.begin();   // establish connection
    getDeviceID(&ID);

    if(ID != ADXL357_ID)    // check for correct part
        return 1;

    reset();
    delay(200);

    return 0;
}


/**
 * @brief Reset ADXL357
 * 
 * @return Zero on success
 */
uint8_t Adxl357::reset()
{
    uint8_t reset_code = 0x52;  // Datasheet P.40

    return writeBytes(ADXL357_REG_RESET, &reset_code, 1);
}


/**
 * @brief Check if data is ready to read
 */
bool Adxl357::isDataReady()
{
    uint8_t status = 0;

    getStatus(&status);
    return status & 0x01;   // Datasheet P.33
}


/**
 * @brief Read device ID at the current address, should be 0xED
 * 
 * @param result pointer to store read value
 * @return Zero on read success
 */
uint8_t Adxl357::getDeviceID(uint8_t *result)
{
    return readBytes(ADXL357_REG_PARTID, result, 1);
}


/**
 * @brief Read device revision ID
 * 
 * @param result pointer to store read value
 * @return Zero on read success
 */
uint8_t Adxl357::getDeviceVer(uint8_t *result)
{
    return readBytes(ADXL357_REG_REVID, result, 1);
}


/**
 * @brief Read the status register of the ADXL357
 * 
 * @param result pointer to store status
 * @return Zero on success
 */
uint8_t Adxl357::getStatus(uint8_t *result)
{
    return readBytes(ADXL357_REG_STATUS, result, 1);
}


/**
 * @brief Read the number of valid FIFO enteries in the buffer
 * 
 * @param result pointer to store number of valid enteries, range from 0 to 96
 * @return Zero on success
 */
uint8_t Adxl357::getFIFOEntries(uint8_t *result)
{
    return readBytes(ADXL357_REG_FIFO_ENTRIES, result, 1);
}


/**
 * @brief Read X, Y, and Z acceleration values from FIFO (safer).
 * Reading from FIFO ensures the read data is from the same measurement.
 * For more info check datasheet P.28.
 * 
 * @param x pointer to store X value
 * @param y pointer to store Y value
 * @param z pointer to store Z value
 * @return Zero on success
 */
uint8_t Adxl357::getRawAccelData(int32_t *x, int32_t *y, int32_t *z)
{
    uint8_t data[9] = {0};                          // array to read 3 measurements 3-byte each

    if(readBytes(ADXL357_REG_FIFO_DATA, data, 9))   // read 9 bytes from FIFO
        return 1;

    // FIFO puts X then Y then X. Bytes in each measurement contain the following bits:
    // byte 1 = 19 18 17 16 15 14 13 12
    // byte 2 = 11 10 09 08 07 06 05 04
    // byte 3 = 03 02 01 00 ?? ?? rr xx
    // rr is set when an attempt to read an empty FIFO is done
    // xx is set for all X-axis measurements
    // shifting is needed to compile the 20-bit measurement
    *x = ((uint32_t) data[0] << 12) | ((uint32_t) data[1] << 4) | ((uint32_t) data[2] >> 4);
    *y = ((uint32_t) data[3] << 12) | ((uint32_t) data[4] << 4) | ((uint32_t) data[5] >> 4);
    *z = ((uint32_t) data[6] << 12) | ((uint32_t) data[7] << 4) | ((uint32_t) data[8] >> 4);

    // all final measurements are signed with two's complement
    // MSB check is needed for correction
    if(*x & 0x80000)
        *x = (*x & 0x7FFFF) - 0x80000;
    if(*y & 0x80000)
        *y = (*y & 0x7FFFF) - 0x80000;
    if(*z & 0x80000)
        *z = (*z & 0x7FFFF) - 0x80000;

    return 0;
}


/**
 * @brief Read X, Y, and Z acceleration values from FIFO (safer), then scale according to _calib.
 * Reading from FIFO ensures the read data is from the same measurement.
 * For more info check datasheet P.28.
 * 
 * @param x pointer to store X value
 * @param y pointer to store Y value
 * @param z pointer to store Z value
 * @return Zero on success
 */
uint8_t Adxl357::getScaledAccelData(double *x, double *y, double *z)
{
    int32_t xr, yr, zr;

    if(getRawAccelData(&xr, &yr, &zr))
        return 1;

    *x = xr * _calib;
    *y = yr * _calib;
    *z = zr * _calib;

    return 0;
}

/**
 * @brief Read number of events above threshold required to detect activity
 * 
 * @param result pointer to store reading
 * @return Zero on success
 */
uint8_t Adxl357::getActivityCount(uint8_t *result)
{
    return readBytes(ADXL357_REG_ACT_COUNT, result, 1);
}


/**
 * @brief Set the components for the activity detection algorithm
 * 
 * @param x set X as a component
 * @param y set Y as a component
 * @param z set y as a component
 * @return Zero on success
 */
uint8_t Adxl357::setActivityEnable(bool x, bool y, bool z)
{
    uint8_t val = 0 | (z << 2) | (y << 1) | x;      // ACT_EN's lowest 3 bits are zyx enable

    return writeBytes(ADXL357_REG_ACT_EN, &val, 1);
}


/**
 * @brief Set the activity threashold that needs to be met to count an activity
 * 
 * @param threashold the threashold 2-byte value
 * @return Zero on write success
 */
uint8_t Adxl357::setActivityThreashold(int16_t threashold)
{
    uint8_t data1 = threashold >> 8;        // split into two bytes and write sequentially
    uint8_t data2 = threashold & 0xff;

    if(writeBytes(ADXL357_REG_ACT_THRESH_H, &data1, 1))
        return 1;

    return writeBytes(ADXL357_REG_ACT_THRESH_L, &data2, 1);
}


/**
 * @brief Set the Activity Count register
 * 
 * @param count value to write to count register
 * @return Zero on write success
 */
uint8_t Adxl357::setActivityCount(uint8_t count)
{
    return writeBytes(ADXL357_REG_ACT_COUNT, &count, 1);
}


/**
 * @brief Set ADXL357 power modes. Use the defines in the header file.
 * Several power-saving options can be OR'ed
 * 
 * @param val the modes to apply
 * @return Zero on write success
 */
uint8_t Adxl357::setPowerCTL(uint8_t val)
{
    return writeBytes(ADXL357_REG_POWER_CTL, &val, 1);
}


/**
 * @brief Set the register for interal high-pass and low-pass filters.
 * Please datasheet P.38 for different opptions and their values.
 * 
 * @param hpf high-pass filter relative to ODR (output data rate)
 * @param lpf ODR (output data rate) and low-pass filter 
 * @return Zero on success
 */
uint8_t Adxl357::setFilter(uint8_t hpf, uint8_t odr_lpf)
{
    uint8_t val = (hpf << 4) | odr_lpf;     // hpf is 3-bits while lpf is 4-bits
    return writeBytes(ADXL357_REG_FILTER, &val, 1);
}


/**
 * @brief Set inturrupt pins and map them for different events. 
 * Please read datasheet P.38 for info on bit fieldss.
 * 
 * @param val value of interuupt pin map
 * @return Zero on write success
 */
uint8_t Adxl357::setIntMap(uint8_t val)
{
    return writeBytes(ADXL357_REG_INT_MAP, &val, 1);
}


/**
 * @brief Set range register, can be +/- 10, 20, or 40g. 
 * Use the defines to avoid issues.
 * 
 * @param range desired range for accelerometer.
 * @return Zero on success
 */
uint8_t Adxl357::setAccelRange(uint8_t range)
{
    uint8_t val;

    // register contains other data we don't wanna change
    if(readBytes(ADXL357_REG_RANGE, &val, 1))
        return 1;

    val = (val & 0xFC) | range;     // reset previous range and set new one
    return writeBytes(ADXL357_REG_RANGE, &val, 1);
}


/**
 * @brief Set calibration constant to scale XYZ results
 * 
 * @param calib the constant to set
 * @return Always zero
 */
uint8_t Adxl357::setCalibrationConstant(double calib)
{
    _calib = calib;
    return 0;
}

////    Private Functions    ////

/**
 * @brief Write an array of bytes to a register on the ADXL357
 * 
 * @param reg the register to write on
 * @param data data array (pointer)
 * @param len length of data to write
 * @return Zero on success
 */
uint8_t Adxl357::writeBytes(uint8_t reg, uint8_t *data, uint32_t len)
{
    ADXL357_WIRE.beginTransmission(_address);   // transmit to ADXL357 address
    ADXL357_WIRE.write(reg);                    // call for register attention

    for(unsigned int i = 0; i < len; i++)
        ADXL357_WIRE.write(data[i]);            // send data over

    return ADXL357_WIRE.endTransmission();
}


/**
 * @brief Read an array of bytes fron a register on the ADXK357
 * 
 * @param reg register to read from
 * @param dest array to write on
 * @param len number of bytes to lead
 * @param retries number of retries before failure (default = 10)
 * @return Zero on success
 */
uint8_t Adxl357::readBytes(uint8_t reg, uint8_t *dest, uint32_t len, uint32_t retries)
{
    uint32_t time_outs = 0;

    ADXL357_WIRE.beginTransmission(_address);   // transmit to ADXL357 address
    ADXL357_WIRE.write(reg);                    // call for register attention
    ADXL357_WIRE.endTransmission();             // end transmission as master
    ADXL357_WIRE.requestFrom(_address, len);    // request data

    while(ADXL357_WIRE.available() != len)      // check if data is ready
    {
        time_outs++;

        if(time_outs > retries)
            return 1;

        delay(2);
    }

    for(unsigned int i = 0; i < len; i++)
        dest[i] = ADXL357_WIRE.read();

   return 0;
}