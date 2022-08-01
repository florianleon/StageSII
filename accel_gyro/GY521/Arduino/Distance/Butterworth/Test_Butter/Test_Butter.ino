#include "GY521.h"

#define RAW2G 1.0/16384.0;    // raw data to gravity g's
#define G_TO_CM2 980.665
#define NUM_READINGS 50


GY521 sensor(0x68);

uint32_t timer;

double t = 0;
int j = 0;
unsigned long time1; //counting since run begins
double f[] = {0,0,0,0,0};
int i = 4; //indexing above the array
double y[] = {0,0,0,0,0};

double accX;

//Moyenne glissante accélération X
double readingsX[NUM_READINGS];
int readIndexX = 0;
double totalX = 0;
double averageX = 0;

double dx = 0;

double integration(double acc, double dt) {
  return abs(acc) * dt * dt;
}

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

double smoothX(double value) {
  double average;
  totalX = totalX - readingsX[readIndexX];
  readingsX[readIndexX] = value;
  totalX = totalX + readingsX[readIndexX];
  readIndexX = (readIndexX + 1) % NUM_READINGS;
  average = totalX / NUM_READINGS;
  return average;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  setup_sensor();
  delay(100); // Wait for sensor to stabilize
  timer = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor.read();
  accX = sensor.getAccelX();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  
  f[i-4] = f[i-3];
  f[i-3] = f[i-2];
  f[i-2] = f[i-1];
  f[i-1] = f[i];
  f[i] = accX;

  y[i-4] = y[i-3];
  y[i-3] = y[i-2];
  y[i-2] = y[i-1];
  y[i-1] = y[i];
  y[i] = 2.42 * y[i-1] - 2.396 * y[i-2] + 1.105 * y[i-3] - 0.198 * y[i-4] + 0.004 * f[i] + 0.017 * f[i-1] - 0.026 * f[i-2] + 0.017 * f[i-3] - 0.004 * f[i-4];

  averageX = smoothX(y[i]) * 980.665;
  averageX -= 6.0;


  if (abs(averageX > 0.5)) {
    dx = dx + integration(averageX, dt);
  }

  //Serial.print(averageX);; Serial.print("\t");
  Serial.print(dx *980.665/2); Serial.print("\t");
  Serial.print("\n");

  delay(10);
}
