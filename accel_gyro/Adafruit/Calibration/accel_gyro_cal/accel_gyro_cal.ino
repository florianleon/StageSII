#include "Accel_Mag.h"
#include "Gyro.h"

#define NUMBER_SAMPLES 10000

/* Assign a unique ID to this sensor at the same time */
Accel_Mag accelmag = Accel_Mag(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Gyro gyro = Gyro(0x0021002C);

sensors_event_t aread, mread, gread;

float gyro_min_x, gyro_max_x, gyro_mid_x;
float gyro_min_y, gyro_max_y, gyro_mid_y;
float gyro_min_z, gyro_max_z, gyro_mid_z;

float acc_min_x, acc_max_x, acc_mid_x;
float acc_min_y, acc_max_y, acc_mid_y;
float acc_min_z, acc_max_z, acc_mid_z;

void read_data() {
  accelmag.getEvent(&aread, &mread);
  gyro.getEvent(&gread);
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
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

  delay(100);

  read_data();
  gyro_min_x = gyro_max_x = gread.gyro.x;
  gyro_min_y = gyro_max_y = gread.gyro.y;
  gyro_min_z = gyro_max_z = gread.gyro.z;
  acc_min_x = acc_max_x = aread.acceleration.x;
  acc_min_y = acc_max_y = aread.acceleration.y;
  acc_min_z = acc_max_z = aread.acceleration.z;
  delay(10);

  Serial.println(F("Place gyro on flat, stable surface!"));

  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");
  
  float x, y, z;
  for (uint16_t sample = 0; sample < NUMBER_SAMPLES; sample++) {
    read_data();
    float gx = gread.gyro.x;
    float gy = gread.gyro.y;
    float gz = gread.gyro.z;
    Serial.print(F("Gyro: ("));
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.print(gz); Serial.print(")");
    Serial.print("\n");
    float ax = aread.acceleration.x;
    float ay = aread.acceleration.y;
    float az = aread.acceleration.z;
    Serial.print(F("Accel: ("));
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az); Serial.print(")");


    gyro_min_x = min(gyro_min_x, gx);
    gyro_min_y = min(gyro_min_y, gy);
    gyro_min_z = min(gyro_min_z, gz);
  
    gyro_max_x = max(gyro_max_x, gx);
    gyro_max_y = max(gyro_max_y, gy);
    gyro_max_z = max(gyro_max_z, gz);
  
    gyro_mid_x = (gyro_max_x + gyro_min_x) / 2;
    gyro_mid_y = (gyro_max_y + gyro_min_y) / 2;
    gyro_mid_z = (gyro_max_z + gyro_min_z) / 2;

    acc_min_x = min(acc_min_x, ax);
    acc_min_y = min(acc_min_y, ay);
    acc_min_z = min(acc_min_z, az);

    acc_max_x = max(acc_max_x, ax);
    acc_max_y = max(acc_max_y, ay);
    acc_max_z = max(acc_max_z, az);

    acc_mid_x = (acc_max_x + acc_min_x) / 2;
    acc_mid_y = (acc_max_y + acc_min_y) / 2;
    acc_mid_z = (acc_max_z + acc_min_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(gyro_mid_x, 4); Serial.print(", ");
    Serial.print(gyro_mid_y, 4); Serial.print(", ");
    Serial.print(gyro_mid_z, 4); Serial.print(")");  
  
    Serial.print(F(" rad/s noise: ("));
    Serial.print(gyro_max_x - gyro_min_x, 3); Serial.print(", ");
    Serial.print(gyro_max_y - gyro_min_y, 3); Serial.print(", ");
    Serial.print(gyro_max_z - gyro_min_z, 3); Serial.println(")");   

    Serial.print(F(" Zero acc offset: ("));
    Serial.print(acc_mid_x, 4); Serial.print(", ");
    Serial.print(acc_mid_y, 4); Serial.print(", ");
    Serial.print(acc_mid_z, 4); Serial.print(")");

    Serial.print(F(" m/s noise: ("));
    Serial.print(acc_max_x - acc_min_x, 3); Serial.print(", ");
    Serial.print(acc_max_y - acc_min_y, 3); Serial.print(", ");
    Serial.print(acc_max_z - acc_min_z, 3); Serial.println(")");
    delay(10);
  }

  Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
  Serial.print(gyro_mid_x, 4); Serial.print(", ");
  Serial.print(gyro_mid_y, 4); Serial.print(", ");
  Serial.println(gyro_mid_z, 4);

  Serial.println(F("\n\nFinal zero acceleration offset in m/s^2: "));
  Serial.print(acc_mid_x, 4); Serial.print(", ");
  Serial.print(acc_mid_y, 4); Serial.print(", ");
  Serial.println(acc_mid_z - 9.80665, 4);
}



void loop() {
  delay(10); 
}
