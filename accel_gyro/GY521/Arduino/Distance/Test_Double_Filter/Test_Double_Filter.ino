/*
Idée venu de ce pdf : https://www.unige.ch/cisa/wiki/lib/exe/fetch.php?media=psychophysiology:accelerometer_data_filtering.pdf 
*/

#include <Wire.h>
#include "GY521.h"

#define RAW2G 1.0/16384.0; 
#define G_TO_CM2 980.665
#define G 9.80665
#define G2 G * G
#define SQRT_G sqrt(G)
#define CORRECTION_FACTOR G2/SQRT_G
#define NUM_READINGS 50
#define BUFFER_ZONE_X 7
#define BUFFER_ZONE_Y 4
#define OFFSET_COUNTER 250
#define PRINT_ACC true

#define LOW_FC 20 //fréquence de coupure filtre passe bas
#define HIGH_FC 0.05 //fréquence de coupure filtre passe haut

GY521 sensor(0x68);

double accX, accY;

uint32_t timer;

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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_sensor();
  delay(100); // Wait for sensor to stabilize

  sensor.read();
  accX = sensor.getAccelX() * G_TO_CM2;
  accY = sensor.getAccelY() * G_TO_CM2;

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

void loop() {

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

  // On récupère la valeur actuelle de l'accélération
  sensor.read();
  accX = sensor.getAccelX() * G_TO_CM2;
  accY = sensor.getAccelY() * G_TO_CM2;

  // On met à jour le delta de temps
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  // AXE X
  // On met à jour l'ancienne valeur du filtre passe haut pour la vitesse
  oldHighPassVX = highPassVX;
  
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
  // On met à jour l'ancienne valeur du filtre passe haut pour la vitesse
  oldHighPassVY = highPassVY;

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
  averageX = smoothX(lowPassDX)*100 - offsetX;
  averageY = smoothY(lowPassDY)*100 - offsetY;
  
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

  // Calcul de la distance finale parcourue
  finalDistance = distance(dx, dy); //A modifier en += plus tard et du coup couvertir les dx et dy aussi

  //AFFICHAGE
  if (PRINT_ACC) {
    Serial.print(accX);Serial.print(",");
    Serial.print(highPassVX);
    Serial.print("\n");
  } else {
    Serial.print(dx);Serial.print(",");
    Serial.print(dy);Serial.print(",");
    Serial.print(finalDistance);
    Serial.print("\n");
  }
  
  delay(10);
}
