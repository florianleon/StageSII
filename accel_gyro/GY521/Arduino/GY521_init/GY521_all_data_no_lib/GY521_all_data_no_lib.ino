#include "GY521.h"

GY521 sensor(0x68); // AD0 low

uint32_t counter = 0;


void setup()
{
    Serial.begin(921600);
    Serial.println(__FILE__);

    Wire.begin();

    delay(100);
    while (sensor.wakeup() == false)
    {
        Serial.print(millis());
        Serial.println("\tCould not connect to GY521");
        delay(1000);
    }
    sensor.setAccelSensitivity(2);  // 8g
    sensor.setGyroSensitivity(1);   // 500 degrees/s

    sensor.setThrottle();
    Serial.println("start...");

    // set calibration values from calibration sketch.
    sensor.axe = 0;
    sensor.aye = 0;
    sensor.aze = 0;
    sensor.gxe = 0;
    sensor.gye = 0;
    sensor.gze = 0;

    Serial.println("\nCounter_?ms;AccelX;AccelY;AccelZ;AngleX;AngleY;AngleZ;GyroX;GyroY;GyroZ;Pitch;Roll;Yaw;Temperature;");
  
}


void loop()
{
    sensor.read();
    float accelx  = sensor.getAccelX();
    float accely  = sensor.getAccelY();
    float accelz  = sensor.getAccelZ();
    float anglex  = sensor.getAngleX();
    float angley  = sensor.getAngleY();
    float anglez  = sensor.getAngleZ();
    float gyrox   = sensor.getGyroX();
    float gyroy   = sensor.getGyroY();
    float gyroz   = sensor.getGyroZ();
    float pitch   = sensor.getPitch();
    float roll    = sensor.getRoll();
    float yaw     = sensor.getYaw();
    float temp    = sensor.getTemperature();
  
    Serial.print(counter);   Serial.print(';');
    Serial.print(accelx, 3); Serial.print(';'); 
    Serial.print(accely, 3); Serial.print(';'); 
    Serial.print(accelz, 3); Serial.print(';'); 
    Serial.print(anglex, 3); Serial.print(';'); 
    Serial.print(angley, 3); Serial.print(';'); 
    Serial.print(anglez, 3); Serial.print(';'); 
    Serial.print(gyrox,  3); Serial.print(';'); 
    Serial.print(gyroy,  3); Serial.print(';'); 
    Serial.print(gyroz,  3); Serial.print(';'); 
    Serial.print(pitch,  3); Serial.print(';'); 
    Serial.print(roll,   3); Serial.print(';'); 
    Serial.print(yaw,    3); Serial.print(';'); 
    Serial.print(temp,   3); Serial.println(';');

    counter++;
    delay(100);
}
