
// You need to install Adafruit 9DOF from the arduino librairy first. 
#include "Accel_Mag.h"
#include "Gyro.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RAD2DEG 180/PI
#define DEG2RAD PI/180
#define NUM_READINGS 50

/* Assign a unique ID to this sensor at the same time */
Accel_Mag accelmag = Accel_Mag(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Gyro gyro = Gyro(0x0021002C);

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magRX, magRY, magRZ;
double magRRX, magRRY, magRRZ;
// https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects --> A mettre dans la doc finale + lien adafruit
double magX, magY;
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

//Moyenne glissante Yaw
double readingsYaw[NUM_READINGS];
int readIndexYaw = 0;
double totalYaw = 0;
double yaw = 0;

Kalman kalmanX; 
Kalman kalmanY;
Kalman kalmanZ;

sensors_event_t aread, mread, gread;

uint32_t timer;

double smoothYaw() {
  double average;
  totalYaw = totalYaw - readingsYaw[readIndexYaw];
  readingsYaw[readIndexYaw] = kalAngleZ;
  totalYaw = totalYaw + readingsYaw[readIndexYaw];
  readIndexYaw = (readIndexYaw + 1) % NUM_READINGS;
  average = totalYaw / NUM_READINGS;
  return average;
}

float calibrated_values[3]; 
float scaler;
boolean scaler_flag = false;
float normal_vector_length;

void transformation(float uncalibrated_values[3]) {
  double calibration_matrix[3][3] = 
  {
    {1.002, -0.012, 0.002},
    {-0.012, 0.974, -0.010},
    {0.002, -0.010, 1.025}  
  };

 double bias[3] = 
  {
    -3.87,
    -35.45,
    -50.32
  }; 
  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

void vector_length_stabilasation(){
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //calculate the current scaler
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

void setup_sensor() {
  /* Initialise the sensor */
  if (!accelmag.begin()) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }

    /* Initialise the sensor */
  if (!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
     */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }

  accelmag.setAccelRange(ACCEL_RANGE_2G);
  accelmag.setOutputDataRate(ODR_200HZ);
  gyro.setRange(GYRO_RANGE_500DPS);
}

double compute_roll() {
  return atan2(accY, accZ) * RAD2DEG;
}

double compute_pitch() {
  return atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD2DEG;
}

double compute_yaw(double roll, double pitch) {

  roll *= DEG2RAD;
  pitch *= DEG2RAD;

  magRRX = calibrated_values[0];
  magRRY = calibrated_values[1];
  magRRZ = calibrated_values[2];

  magX = magRRX * cos(pitch) + magRRY * sin(roll) * sin(pitch) - magRRZ * cos(roll) * sin(pitch);
  magY = -magRRY * cos(roll) + magRRZ * sin(roll);
  //magX = magRRX*cos(-pitch) + magRRZ*sin(-pitch);
  //magY = magRRX*sin(roll)*sin(-pitch) + magRRY*cos(roll) - magRRZ*sin(roll)*cos(-pitch);

  double res = atan2(-magY, magX);
  return res * RAD_TO_DEG;
}

void read_data() {
  accelmag.getEvent(&aread, &mread);
  gyro.getEvent(&gread);
}

void setup(void) {
  Serial.begin(115200);
  setup_sensor();

  read_data();
  accX = aread.acceleration.x;
  accY = aread.acceleration.y;
  accZ = aread.acceleration.z;
  magRX = mread.magnetic.x;
  magRY = mread.magnetic.y;
  magRZ = mread.magnetic.z;

  double roll = compute_roll();
  double pitch = compute_pitch();
  double yaw = compute_yaw(roll, pitch);

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);

  timer = micros();
}

void loop(void) {

  /* acceleration is measured in m/s^2 */
  /* mag data is in uTesla */
  /* gyro is measured in rad/s */

  float values_from_magnetometer[3];

  read_data();
  accX = aread.acceleration.x;
  accY = aread.acceleration.y;
  accZ = aread.acceleration.z;
  magRX = mread.magnetic.x;
  magRY = mread.magnetic.y;
  magRZ = mread.magnetic.z;
  values_from_magnetometer[0] = magRX;
  values_from_magnetometer[1] = magRY;
  values_from_magnetometer[2] = magRZ;
  gyroX = gread.gyro.x;
  gyroY = gread.gyro.y;
  gyroZ = gread.gyro.z;

  //A grouper dans la lib rust
  transformation(values_from_magnetometer);
  vector_length_stabilasation();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll = compute_roll();
  double pitch = compute_pitch();
  double yaw = compute_yaw(roll, pitch);

  double gyroXrate = gyroX * RAD2DEG; 
  double gyroYrate = gyroY * RAD2DEG;
  double gyroZrate = gyroZ * RAD2DEG;

  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
  } else {
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  }

  if (abs(kalAngleY) > 90) {
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

  double smooth = smoothYaw();
  //Serial.print(calibrated_values[0]);Serial.print(",");
  //Serial.print(calibrated_values[1]);Serial.print(",");
  //Serial.print(calibrated_values[2]);Serial.print("\n");
  /*
  Serial.print(magRX);Serial.print(",");
  Serial.print(magRY);Serial.print(",");
  Serial.print(magRZ);Serial.print("\n");
  */
  //Serial.print(kalAngleX);Serial.print(",");
  //Serial.print(kalAngleY);Serial.print(",");
  Serial.print(smooth);Serial.print("\n");
  
  delay(5);
}
