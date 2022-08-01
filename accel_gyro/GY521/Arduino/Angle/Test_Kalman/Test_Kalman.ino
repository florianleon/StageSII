#include <Wire.h>
#include "GY521.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RAW2G 1.0/16384.0
#define SQRT_G sqrt(9.80665)*3/4
#define BUFFER_ZONE 0.05

Kalman kalmanX; 
Kalman kalmanY;
Kalman kalmanZ;

GY521 sensor(0x68);
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double offset;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

uint32_t timer;

double total_Z_Angle;
int sens = 0; // 0 = neutre, 1 = droite, 2 = gauche
int cpt = 0; // pour l'intersection avec l'axe des abscisses
int sens_prec = 0;
int sens_count = 0;


void setup_sensor() {
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
  sensor.axe = 204 * RAW2G;
  sensor.aye = 2133 * RAW2G;
  sensor.aze = 1280 * RAW2G;
  sensor.gxe = -50 * RAW2G;
  sensor.gye = 99 * RAW2G;
  sensor.gze = 43 * RAW2G;
}

double integration(double angle, double dt) {
  return abs(angle) * dt;// * SQRT_G;
}

double compute_roll() {
  return atan2(accY, accZ) * 180 / PI;
}

double compute_pitch() {
  return atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
}

double compute_yaw() {
  return atan(accZ / sqrt((accX * accX) + (accZ * accZ))) * RAD_TO_DEG;
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

  double roll = compute_roll();
  double pitch = compute_pitch();
  double yaw = compute_yaw();

  total_Z_Angle = 0;
  
  kalmanX.setAngle(roll); 
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;
  offset = yaw;
  timer = micros();
}

void loop() {
  sensor.read();
  accX = sensor.getAccelX();
  accY = sensor.getAccelY();
  accZ = sensor.getAccelZ();
  gyroX = sensor.getGyroX();
  gyroY = sensor.getGyroY();
  gyroZ = sensor.getGyroZ();
  tempRaw = sensor.getTemperature();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  
  double roll = compute_roll();
  double pitch = compute_pitch();
  double yaw = compute_yaw();

  double gyroXrate = gyroX; //gyroX is already in degrees per second
  double gyroYrate = gyroY; //gyroY is already in degrees per second
  double gyroZrate = gyroZ; //gyroZ is already in degrees per second

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalAngleX) > 90) {
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else {
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalAngleY;
  }
  if (gyroZangle < -180 || gyroZangle > 180) {
    gyroZangle = kalAngleZ;
  }
  double angleZ = kalAngleZ - offset;

  //Corrige la dérive de la mesure de l'angle
  //On a un peu comme une zone tampon
  if (angleZ < -BUFFER_ZONE) {
    sens = -1;
  } else if (angleZ >= BUFFER_ZONE) {
    sens = 1;
  } else {
    sens = 0;
  }
  //On compte combien de fois on a changé de sens
  if ((sens != sens_prec) && (sens != 0)) {
    sens_prec = sens;
    sens_count++;
  }
  if (sens_count > 3) {
    sens_count = 0;
    sens_prec = 0;
  } else {
    if (sens != 0) {
       total_Z_Angle += integration(angleZ, dt) * SQRT_G ;

    }
  }

  //total_Z_Angle += abs(angleZ)* dt * SQRT_G;
  /* Print Data */
  Serial.print(total_Z_Angle);Serial.print(",");
  //Serial.print(offset);Serial.print(",");
  //Serial.print(kalAngleZ);Serial.print(",");
  //Serial.print(angleZ);Serial.print(",");
  //Serial.print(sens);
  Serial.print("\n");
  
  delay(10);
}
