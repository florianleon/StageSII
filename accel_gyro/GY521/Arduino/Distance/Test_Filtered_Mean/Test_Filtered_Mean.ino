#include <Wire.h>
#include "GY521.h"

#define G_TO_CM2 980.665
#define NUM_READINGS 25
//#define G_TO_CM2 1
#define K 0.3
#define K2 K*K
#define K3 K2*K

GY521 sensor(0x68);

double accX, accY, accZ;

double F1X, F2X, F3X;
double F1Y, F2Y, F3Y;
double F1Z, F2Z, F3Z;
double old_F1X, old_F2X, old_F3X;
double old_F1Y, old_F2Y, old_F3Y;
double old_F1Z, old_F2Z, old_F3Z;

//Moyenne glissante accélération X
double readingsX[NUM_READINGS];
int readIndexX = 0;
double totalX = 0;

//Moyenne glissante accélération Y
double readingsY[NUM_READINGS];
int readIndexY = 0;
double totalY = 0;

//Moyenne glissante accélération Z
double readingsZ[NUM_READINGS];
int readIndexZ = 0;
double totalZ = 0;

double averageX, averageY, averageZ;

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
  sensor.axe = 0;
  sensor.aye = 0;
  sensor.aze = 0;
}

double smoothX() {
  double average;
  totalX = totalX - readingsX[readIndexX];
  readingsX[readIndexX] = accX * G_TO_CM2;
  totalX = totalX + readingsX[readIndexX];
  readIndexX = (readIndexX + 1) % NUM_READINGS;
  average = totalX / NUM_READINGS;
  return average;
}

double smoothY() {
  double average;
  totalY = totalY - readingsY[readIndexY];
  readingsY[readIndexY] = accY * G_TO_CM2;
  totalY = totalY + readingsY[readIndexY];
  readIndexY = (readIndexY + 1) % NUM_READINGS;
  average = totalY / NUM_READINGS;
  return average;
}

double smoothZ() {
  double average;
  totalZ = totalZ - readingsZ[readIndexZ];
  readingsZ[readIndexZ] = accZ * G_TO_CM2;
  totalZ = totalZ + readingsZ[readIndexZ];
  readIndexZ = (readIndexZ + 1) % NUM_READINGS;
  average = totalZ / NUM_READINGS;
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

  F1X = K * averageX;
  F2X = (K - K2) * F1X + K2 * averageX;
  F3X = (K - K2) * F2X + (K2-K3) * F1X + K3 * averageX;

  F1Y = K * averageY;
  F2Y = (K - K2) * F1Y + K2 * averageY;
  F3Y = (K - K2) * F2Y + (K2-K3) * F1Y + K3 * averageY;

  F1Z = K * averageZ;
  F2Z = (K - K2) * F1Y + K2 * averageZ;
  F3Z = (K - K2) * F2Y + (K2-K3) * F1Z + K3 * averageZ;

  old_F1X = F1X;
  old_F2X = F2X;
  old_F3X = F3X;
  old_F1Y = F1Y;
  old_F2Y = F2Y;
  old_F3Y = F3Y;
  old_F1Z = F1Z;
  old_F2Z = F2Z;
  old_F3Z = F3Z;
}

void loop() {
  sensor.read();
  accX = sensor.getAccelX();
  accY = sensor.getAccelY();
  accZ = sensor.getAccelZ();

  averageX = smoothX();
  averageY = smoothY();
  averageZ = smoothZ();

  old_F1X = F1X;
  old_F2X = F2X;
  old_F3X = F3X;
  old_F1Y = F1Y;
  old_F2Y = F2Y;
  old_F3Y = F3Y;
  old_F1Z = F1Z;
  old_F2Z = F2Z;
  old_F3Z = F3Z;

  F1X = (1-K) * old_F1X + K * averageX;
  F2X = (1-K) * old_F2X + (K - K2) * old_F1X + K2 * averageX;
  F3X = (1-K) * old_F3X + (K - K2) * old_F2X + (K2-K3) * old_F1X + K3 * averageX;

  F1Y = (1-K) * old_F1Y + K * averageY;
  F2Y = (1-K) * old_F2Y + (K - K2) * old_F1Y + K2 * averageY;
  F3Y = (1-K) * old_F3Y + (K - K2) * old_F2Y + (K2-K3) * old_F1Y + K3 * averageY;

  F1Z = (1-K) * old_F1Z + K * averageZ;
  F2Z = (1-K) * old_F2Z + (K - K2) * old_F1Z + K2 * averageZ;
  F3Z = (1-K) * old_F3Z + (K - K2) * old_F2Z + (K2-K3) * old_F1Z + K3 * averageZ;

  /* Print Data */
  Serial.print(((accY))*G_TO_CM2);Serial.print(",");
  Serial.print(averageY);Serial.print(",");
  //Serial.print((F1Y));Serial.print(",");
  //Serial.print((F2Y));Serial.print(",");
  Serial.print((F3Y));Serial.print(",");
  Serial.print("\n");
  
  delay(10);
}
