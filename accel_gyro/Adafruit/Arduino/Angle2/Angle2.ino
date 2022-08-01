
// You need to install Adafruit 9DOF from the arduino librairy first. 
#include "Accel_Mag.h"
#include "Gyro.h"
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RAD2DEG 180/PI
#define DEG2RAD PI/180
#define NUM_READINGS 50
#define G_TO_CM2 980.665
#define G 9.80665
#define SQRT_G sqrt(G)
#define CORRECTION_FACTOR 5//(G/SQRT_G)
#define OFFSET_COUNTER 500
#define BUFFER_ZONE_X 2
#define BUFFER_ZONE_Y 2
#define HIGH_FC 0.05
#define PRINT_ACC false

/* Assign a unique ID to this sensor at the same time */
Accel_Mag accelmag = Accel_Mag(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Gyro gyro = Gyro(0x0021002C);

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magRX, magRY, magRZ;
double kalAngleX, kalAngleY = 0;
double offsetX, offsetY;
double averageX;
double averageY;


bool ready2go = false;
double dx, dy;
double finalDistance = 0;

bool reset_offset = false;
int counter_reset = 0;
double old_dx, old_dy = -1;
double counter = 0;

// Distance X
double entryHighpassX;
double highPassX;
double oldHighPassX;

// Distance Y
double entryHighpassY;
double highPassY;
double oldHighPassY;

// Constante de temps
double high_tau;

Kalman kalmanX; 
Kalman kalmanY;

sensors_event_t aread, mread, gread;

uint32_t timer;

double integration(double value, double dt) {
  return value * dt;
}

double distance(double x, double y) {
  return sqrt(x * x + y * y);
}

double highPassFilter(double e, double old_e, double s, double dt) {
  return s + (e - old_e) - (dt/high_tau) * s;
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

void read_data() {
  accelmag.getEvent(&aread, &mread);
  gyro.getEvent(&gread);
}

void setup(void) {
  Serial.begin(115200);
  setup_sensor();

  high_tau = 1 / (2 * PI * HIGH_FC);

  read_data();
  accX = aread.acceleration.x;
  accY = aread.acceleration.y;
  accZ = aread.acceleration.z;
  magRX = mread.magnetic.x;
  magRY = mread.magnetic.y;
  magRZ = mread.magnetic.z;

  double roll = compute_roll();
  double pitch = compute_pitch();

  // Filtre passe haut pour la distance X
  entryHighpassX = accX;
  highPassX = accX;
  oldHighPassX = accX;

  // Filtre passe haut pour la vitesse Y
  entryHighpassY = accY;
  highPassY = accY;
  oldHighPassY = accY;

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);

  timer = micros();
}

void loop(void) {

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

  /* acceleration is measured in m/s^2 */
  /* mag data is in uTesla */
  /* gyro is measured in rad/s */

  read_data();
  accX = aread.acceleration.x;
  accY = aread.acceleration.y;
  accZ = aread.acceleration.z;
  magRX = mread.magnetic.x;
  magRY = mread.magnetic.y;
  magRZ = mread.magnetic.z;
  gyroX = gread.gyro.x;
  gyroY = gread.gyro.y;
  gyroZ = gread.gyro.z;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll = compute_roll();
  double pitch = compute_pitch();
  //double yaw = compute_yaw(roll, pitch);

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

  // On met à jour l'ancienne valeur du filtre passe haut pour la vitesse X
  oldHighPassX = averageX;

  // On met à jour l'ancienne valeur du filtre passe haut pour la vitesse
  oldHighPassY = averageY;
  
  averageX = kalAngleX * 100 - offsetX;
  averageY = kalAngleY * 100 - offsetY;

  // Filtre passe haut pour la vitesse X
  entryHighpassX = averageX;
  highPassX = highPassFilter(entryHighpassX, oldHighPassX, highPassX, dt);

  //Filtrage passe haut pour la distance Y
  entryHighpassY = averageY;
  highPassY = highPassFilter(entryHighpassY, oldHighPassY, highPassY, dt);


  // Calcul de la distance parcourue sur l'axe X
  if ((abs(highPassX) > BUFFER_ZONE_X) && ready2go) {
    old_dx = dx;
    dx += integration(abs(highPassX), dt) * CORRECTION_FACTOR/100;
  }

  // Calcul de la distance parcourue sur l'axe Y
  if ((abs(highPassY) > BUFFER_ZONE_Y) && ready2go) {
    old_dy = dy;
    dy += integration(abs(highPassY), dt) * CORRECTION_FACTOR/100;
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

  // Calcul de la distance finale parcourue
  finalDistance = distance(dx, dy);

  if (PRINT_ACC) {
    Serial.print(abs(highPassX));Serial.print(",");
    Serial.print(abs(highPassY));
    //Serial.print(aread.acceleration.x - offsetAccX);Serial.print(",");
    //Serial.print(aread.acceleration.y - offsetAccY);
    Serial.print("\n");
  } else {
    Serial.print(dx);Serial.print(",");
    Serial.print(dy);Serial.print(",");
    Serial.print(finalDistance);
    Serial.print("\n");
  }
  
  delay(5);
}
