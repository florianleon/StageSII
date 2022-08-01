#include <Wire.h>
#include "GY521.h"

#define RAW2G 1.0/16384.0;

GY521 sensor(0x68);

double accX, accY, accZ;

//Moyenne glissante accélération X
const int numReadingsX = 30;
double readingsX[numReadingsX];
int readIndexX = 0;
double totalX = 0;

//Moyenne glissante accélération Y
const int numReadingsY = 30;
double readingsY[numReadingsY];
int readIndexY = 0;
double totalY = 0;

//Moyenne glissante accélération Z
const int numReadingsZ = 30;
double readingsZ[numReadingsZ];
int readIndexZ = 0;
double totalZ = 0;

double averageX;
double averageY;
double averageZ;

void setup_sensor() {
  delay(100);
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521");
    delay(1000);
  }
  sensor.setAccelSensitivity(2);  // 8g

  sensor.setThrottle();
  Serial.println("start...");

  // set calibration values from calibration sketch.
  sensor.axe = 247 * RAW2G;
  sensor.aye = 2164 * RAW2G;
  sensor.aze = 1304 * RAW2G;
}

double smoothX() {
  double average;
  totalX = totalX - readingsX[readIndexX];
  readingsX[readIndexX] = accX * 980.665;
  totalX = totalX + readingsX[readIndexX];
  readIndexX = (readIndexX + 1) % numReadingsX;
  average = totalX / numReadingsX;
  return average;
}

double smoothY() {
  double average;
  totalY = totalY - readingsY[readIndexY];
  readingsY[readIndexY] = accY * 980.665;
  totalY = totalY + readingsY[readIndexY];
  readIndexY = (readIndexY + 1) % numReadingsY;
  average = totalY / numReadingsY;
  return average;
}

double smoothZ() {
  double average;
  totalZ = totalZ - readingsZ[readIndexZ];
  readingsZ[readIndexZ] = accZ * 980.665;
  totalZ = totalZ + readingsZ[readIndexZ];
  readIndexZ = (readIndexZ + 1) % numReadingsZ;
  average = totalZ / numReadingsZ;
  return average;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_sensor();
  delay(100); // Wait for sensor to stabilize

  sensor.read();
  accX = sensor.getAccelX();
  accY = sensor.getAccelY();
  accZ = sensor.getAccelZ();

  averageX = smoothX();
  averageY = smoothY();
  averageZ = smoothZ();

}

void loop() {
  sensor.read();
  accX = sensor.getAccelX();
  accY = sensor.getAccelY();
  accZ = sensor.getAccelZ();

  averageX = smoothX();
  averageY = smoothY();
  averageZ = smoothZ();

  /* Print Data */
  Serial.print(accY*980.665);Serial.print(",");
  Serial.print(averageY);Serial.print(",");
  Serial.print("\n");
  
  delay(10);
}
