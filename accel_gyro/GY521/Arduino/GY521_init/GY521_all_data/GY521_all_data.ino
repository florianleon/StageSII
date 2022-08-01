#include "GY521_csv.h"

GY521_csv* gy_csv = new GY521_csv(AD0_LOW);

void setup()
{
    Serial.begin(115200);
    Serial.println(__FILE__);

    Wire.begin();

    gy_csv->setColumns( "_100ms", COUNTER + TEMP            // if you want to have a counter, you can precise the time unit 
                               + ACCELX + ACCELY + ACCELZ   // in the first argument with 6 characters like "1200ms" or "__60ms"
                               + ANGLEX + ANGLEY + ANGLEZ
                               + PITCH  + ROLL   + YAW   ); // order is not important
    // if you don't want to use the counter, you dont need to spécify the measure period
    // just use gy_csv->setColumns( ACCELX + ACCELY + ACCELZ );

    delay(100);
    while ( !gy_csv->isConnected() )
    {
        Serial.print(millis());
        Serial.println("\tCould not connect to GY521");
        delay(1000);
    }
    gy_csv->setSensitivity(8, 1);   // 8g and 500 degrees/s

    gy_csv->setThrottle();
    Serial.println("start...");

    // set calibration values from calibration sketch.
    gy_csv->setCalibrationValues(0,0,0,  // accel calibration from GY512 library calibration example 
                                 0,0,0); // gyro  calibration from GY512 library calibration example 

    gy_csv->printHeader();
}


void loop()
{
    gy_csv->update();
    gy_csv->printData();
    delay(100);
}
