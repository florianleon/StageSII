
// You need to install Adafruit 9DOF from the arduino librairy first. 
#include "Accel_Mag.h"
#include "Gyro.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RAD2DEG 180/PI
#define DEG2RAD PI/180
#define G_TO_CM2 980.665
#define G 9.80665
#define SQRT_G sqrt(G)
#define NUM_READINGS 50
#define CORRECTION_FACTOR (G/SQRT_G)
#define BUFFER_ZONE_X 13
#define BUFFER_ZONE_Y 10
#define OFFSET_COUNTER 1000
#define PRINT_ACC false

#define LOW_FC 20 //fréquence de coupure filtre passe bas
#define HIGH_FC 0.05 //fréquence de coupure filtre passe haut

/* Assign a unique ID to this sensor at the same time */
Accel_Mag accelmag = Accel_Mag(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Gyro gyro = Gyro(0x0021002C);

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magRX, magRY, magRZ;
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

double offsetAccX, offsetAccY, offsetAccZ;

sensors_event_t aread, mread, gread;

// FILTRE PASS-HAUT
// Axe X
// Vitesse X
double entryHighpassVX;
double highPassVX;
double oldHighPassVX;

// Distance X
double entryHighpassDX;
double highPassDX;
double oldHighPassDX;

// Axe Y
// Vitesse Y
double entryHighpassVY;
double highPassVY;
double oldHighPassVY;

// Distance Y
double entryHighpassDY;
double highPassDY;
double oldHighPassDY;

// Constante de temps
double high_tau;


// FILTRE PASS-BAS
// Axe X
// Vitesse X 
double entryLowpassVX;
double lowPassVX;

// Distance X 
double entryLowpassDX;
double lowPassDX;

// Axe Y
// Vitesse Y
double entryLowpassVY;
double lowPassVY;

// Distance Y
double entryLowpassDY;
double lowPassDY;

// Constante de temps
double low_tau;

//MOYENNE GLISSANTE
double counter = 0;
// Accélération X
double readingsX[NUM_READINGS];
int readIndexX = 0;
double totalX = 0;
double averageX = 0;
double offsetX = 0;

// Accélération Y
double readingsY[NUM_READINGS];
int readIndexY = 0;
double totalY = 0;
double averageY = 0;
double offsetY = 0;

bool ready2go = false;
double vx, vy;
double dx, dy;
double finalDistance = 0;

bool reset_offset = false;
int counter_reset = 0;
double old_dx, old_dy = -1;

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

  double magRRX = calibrated_values[0];
  double magRRY = calibrated_values[1];
  double magRRZ = calibrated_values[2];

  magX = magRRX * cos(pitch) + magRRY * sin(roll) * sin(pitch) - magRRZ * cos(roll) * sin(pitch);
  magY = -magRRY * cos(roll) + magRRZ * sin(roll);

  double res = atan2(-magY, magX);
  return res * RAD_TO_DEG;
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

double smoothY(double value) {
  double average;
  totalY = totalY - readingsY[readIndexY];
  readingsY[readIndexY] = value;
  totalY = totalY + readingsY[readIndexY];
  readIndexY = (readIndexY + 1) % NUM_READINGS;
  average = totalY / NUM_READINGS;
  return average;
}

double lowPassFilter(double e, double s, double dt) {
  return s + (dt/low_tau) * (e - s);
}

double highPassFilter(double e, double old_e, double s, double dt) {
  return s + (e - old_e) - (dt/high_tau) * s;
}

double integration(double value, double dt) {
  return value * dt;
}

double distance(double x, double y) {
  return sqrt(x * x + y * y);
}

void read_data() {
  accelmag.getEvent(&aread, &mread);
  gyro.getEvent(&gread);
}

void setup(void) {
  Serial.begin(115200);
  setup_sensor();

  offsetAccX = -0.0526;
  offsetAccY = -0.5145;
  offsetAccZ = 0.1439;

  read_data();
  accX = (aread.acceleration.x - offsetAccX) * G_TO_CM2;
  accY = (aread.acceleration.y - offsetAccY) * G_TO_CM2;
  accZ = (aread.acceleration.z - offsetAccZ) * G_TO_CM2;

  magRX = mread.magnetic.x;
  magRY = mread.magnetic.y;
  magRZ = mread.magnetic.z;

  double roll = compute_roll();
  double pitch = compute_pitch();
  double yaw = compute_yaw(roll, pitch);

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);

  // Constantes de temps
  high_tau = 1 / (2 * PI * HIGH_FC);
  low_tau = 1 / (2 * PI * LOW_FC);

  // AXE X
  // Filtre passe haut pour la vitesse X
  entryHighpassVX = accX;
  highPassVX = accX;
  oldHighPassVX = accX;
  
  // Filtre passe bas pour la vitesse X
  entryLowpassVX = highPassVX;
  lowPassVX = highPassVX;

  vx = 0;
  dx = 0;

  // Filtre passe haut pour la distance X
  entryHighpassDX = vx;
  highPassDX = vx;
  oldHighPassDX = vx;

  // Filtre passe bas pour la distance X
  entryLowpassDX = highPassDX;
  lowPassDX = highPassDX;

  // AXE Y
  // Filtre passe haut pour la vitesse Y
  entryHighpassVY = accY;
  highPassVY = accY;
  oldHighPassVY = accY;

  // Filtre passe bas pour la vitesse Y
  entryLowpassVY = highPassVY;
  lowPassVY = highPassVY;

  vy = 0;
  dy = 0;

  // Filtre passe haut pour la distance Y
  entryHighpassDY = vy;
  highPassDY = vy;
  oldHighPassDY = vy;

  timer = micros();
}

void loop(void) {

  /* acceleration is measured in m/s^2 */
  /* mag data is in uTesla */
  /* gyro is measured in rad/s */

  float values_from_magnetometer[3];

  // Gestion des offsets
  if (counter == OFFSET_COUNTER) {
    offsetX = averageX;
    offsetY = averageY;
    ready2go = true;
    dx = 0;
    dy = 0;
    Serial.print("Offset Applied!\n");
  } 

  if (counter < OFFSET_COUNTER + 1) {
    counter++;
  }

  // On met à jour l'ancienne valeur du filtre passe haut pour la vitesse X
  oldHighPassVX = accX;

  // On met à jour l'ancienne valeur du filtre passe haut pour la vitesse
  oldHighPassVY = accY;

  read_data();
  accX = (aread.acceleration.x - offsetAccX) * G_TO_CM2;
  accY = (aread.acceleration.y - offsetAccY) * G_TO_CM2;
  accZ = (aread.acceleration.z - offsetAccZ) * G_TO_CM2;
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

  // AXE X
  
  // Filtre passe haut pour la vitesse X
  entryHighpassVX = accX;
  highPassVX = highPassFilter(entryHighpassVX, oldHighPassVX, highPassVX, dt);

  // Filtre passe haut pour la vitesse X
  entryLowpassVX = highPassVX;
  lowPassVX = lowPassFilter(entryLowpassVX, lowPassVX, dt);
  
  // On calcule VX
  vx = integration(lowPassVX, dt);

  // On met à jour l'ancienne valeur du filtre passe haut pour la distance X
  oldHighPassDX = highPassDX;

  //Filtrage passe haut pour la distance X
  entryHighpassDX = vx;
  highPassDX = highPassFilter(entryHighpassDX, oldHighPassDX, highPassDX, dt);

  //Filtrage passe bas pour la distance X
  entryLowpassDX = highPassDX;
  lowPassDX = lowPassFilter(entryLowpassDX, lowPassDX, dt);

  
  // AXE Y

  // Filtre passe haut pour la vitesse Y
  entryHighpassVY = accY;
  highPassVY = highPassFilter(entryHighpassVY, oldHighPassVY, highPassVY, dt);

  // Filtre passe haut pour la vitesse Y
  entryLowpassVY = highPassVY;
  lowPassVY = lowPassFilter(entryLowpassVY, lowPassVY, dt);

  // On calcule VY
  vy = integration(lowPassVY, dt);

  // On met à jour l'ancienne valeur du filtre passe haut pour la distance Y
  oldHighPassDY = highPassDY;

  //Filtrage passe haut pour la distance Y
  entryHighpassDY = vy;
  highPassDY = highPassFilter(entryHighpassDY, oldHighPassDY, highPassDY, dt);

  //Filtrage passe bas pour la distance Y
  entryLowpassDY = highPassDY;
  lowPassDY = lowPassFilter(entryLowpassDY, lowPassDY, dt);

  // Lissage des courbes pour le calcul
  averageX = smoothX(lowPassDX) * 100 - offsetX;
  averageY = smoothY(lowPassDY) * 100 - offsetY;
  
  // Calcul de la distance parcourue sur l'axe X
  if ((abs(averageX) > BUFFER_ZONE_X) && ready2go) {
    old_dx = dx;
    dx += integration(abs(averageX), dt) * CORRECTION_FACTOR/100;
  }

  // Calcul de la distance parcourue sur l'axe Y
  if ((abs(averageY) > BUFFER_ZONE_Y) && ready2go) {
    old_dy = dy;
    dy += integration(abs(averageY), dt) * CORRECTION_FACTOR/100;
  }

  if ((abs(old_dx - dx) || abs(old_dy - dy)) && ready2go) {
    if (counter_reset == 500) {
      counter_reset = 0;
      dx = 0;
      dy = 0;
    } else {
      counter_reset++;
    }
  } else {
    counter_reset = 0;
  }

  double dax = sin(kalAngleX) * G;
  double day = sin(kalAngleY) * G;
  double dpx = 0.5*dax*dt*dt * 100;
  double dpy = 0.5*day*dt*dt * 100;

  // Calcul de la distance finale parcourue
  finalDistance = distance(abs(dx - dpx*dx), abs(dy - dpy*dy));

  //AFFICHAGE
  if (PRINT_ACC) {
    Serial.print(abs(averageX));Serial.print(",");
    Serial.print(abs(averageY));
    //Serial.print(aread.acceleration.x - offsetAccX);Serial.print(",");
    //Serial.print(aread.acceleration.y - offsetAccY);
    Serial.print("\n");
  } else {
    Serial.print(dpx*dx);Serial.print(",");
    Serial.print(dpy*dy);Serial.print(",\t");
    Serial.print(dx);Serial.print(",");
    Serial.print(dy);Serial.print(",\t");
    Serial.print(finalDistance);
    Serial.print("\n");
  }
  
  delay(5);
}
