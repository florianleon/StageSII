//
//    FILE: GY521_pitch_roll_yaw.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo pitch roll yaw
//    DATE: 2020-08-06


#include "GY521.h"

GY521 sensor(0x69); // AD0 high

uint32_t counter = 0;


void setup()
{
  Serial.begin(921600); // 115200 for Arduino Uno
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
  
  Serial.println("\nCNT_?ms;PITCH;ROLL;YAW;");
  
}


void loop()
{
  sensor.read();
  float pitch = sensor.getPitch();
  float roll  = sensor.getRoll();
  float yaw   = sensor.getYaw();

  Serial.print(counter);
  Serial.print(';');
  Serial.print(pitch, 3);
  Serial.print(';');
  Serial.print(roll, 3);
  Serial.print(';');
  Serial.print(yaw, 3);
  Serial.println(';');

  counter++;
  //delay(100);
}
