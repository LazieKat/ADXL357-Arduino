#include <Arduino.h>
#include <adxl357.h>
#include <math.h>


double  calib = 1.0 / (double) 12490; // calculate your own calibration constant
Adxl357 adxl357;


void setup(void)
{
    Serial.begin(9600);

    while(adxl357.begin(ADXL357_DEF_ADD))
        Serial.println("Can't detect an ADXL357 device");

    Serial.println("Init OK");
    
    adxl357.setAccelRange(ADXL357_FOUTY_G);
    adxl357.setPowerCTL(ADXL357_ALL_ON);
    adxl357.setCalibrationConstant(calib);
    delay(100);

    delay(1000);
}

void loop(void)
{
    double x, y, z, r;

    if (adxl357.isDataReady())
    {
        if (adxl357.getScaledAccelData(&x, &y, &z))
        {
            Serial.println("Data reading failed");
        }

        r = sqrt(x*x + y*y + z*z);

        Serial.print("\nX axis = ");
        Serial.print(x);
        Serial.print("g\t");
        Serial.print("Y axis = ");
        Serial.print(y);
        Serial.print("g\t");
        Serial.print("Z axis = ");
        Serial.print(z);
        Serial.print("g\t");
        Serial.print("Total = ");
        Serial.print(r);
        Serial.print("g\t");
    }
    else
    {
        Serial.println("Data is not ready");
    }

    loopEnd:
    delay(1000);
}