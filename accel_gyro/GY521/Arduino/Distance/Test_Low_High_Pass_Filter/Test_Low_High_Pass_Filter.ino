#include <Wire.h>
#include "GY521.h"

#define RAW2G 1.0/16384.0;    // raw data to gravity g's
#define G_TO_CM2 980.665
#define NUM_READINGS 50
#define BUFFER_ZONE 5
#define OFFSET_COUNTER 250

#define FC 0.05

GY521 sensor(0x68);

double accX, accY, accZ;

uint32_t timer;

double tau;
double e;
double s;

double high_e;
double old_high_e;
double high_s;

//Moyenne glissante accélération X
double readingsX[NUM_READINGS];
int readIndexX = 0;
double totalX = 0;
double averageX = 0;

//Moyenne glissante high
double readingsH[NUM_READINGS];
int readIndexH = 0;
double totalH = 0;
double averageH = 0;

double dx = 0;
double dh = 0;

double offsetX = 0;
int counter;

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
  sensor.axe = 204 * RAW2G;
  sensor.aye = 2133 * RAW2G;
  sensor.aze = 1280 * RAW2G;
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

double smoothH() {
  double average;
  totalH = totalH - readingsH[readIndexH];
  readingsH[readIndexH] = high_s;
  totalH = totalH + readingsH[readIndexH];
  readIndexH = (readIndexH + 1) % NUM_READINGS;
  average = totalH / NUM_READINGS;
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

  e = accX;
  s = e;
  high_e = accX;
  old_high_e = high_e;
  high_s = high_e;
  tau = 1/(2 * PI * FC);
  counter = 0;
  timer = micros();
}

void loop() {

  if (counter == OFFSET_COUNTER) {
    offsetX = averageX;
  }

  if (counter < OFFSET_COUNTER + 1) {
    counter++;
  }
  old_high_e = averageX;
  sensor.read();
  accX = sensor.getAccelX();
  accY = sensor.getAccelY();
  accZ = sensor.getAccelZ();

  averageX = smoothX();
  e = averageX;
  high_e = averageX;
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  s = s + (dt/tau) * (e - s);
  high_s = high_s + (high_e - old_high_e) - (dt/tau) * high_s;
  double res = s - offsetX;
  if (res > 1.0) {
    dx += res * dt * dt;
  }
  averageH = smoothH();

  if (abs(averageH) > 3.0) {
    dh += abs(averageH) * dt *dt;
  }
  
  Serial.print(accX*G_TO_CM2);Serial.print(",");
  //Serial.print(dh*20);Serial.print(",");
  //Serial.print(averageFX);Serial.print(",");
  Serial.print(averageH);
  Serial.print("\n");
  
  delay(10);
}
