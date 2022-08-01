// You need to install Adafruit 9DOF from the arduino librairy first. 
#include "Accel_Mag.h"
#include "Gyro.h"

/* Assign a unique ID to this sensor at the same time */
Accel_Mag accelmag = Accel_Mag(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Gyro gyro = Gyro(0x0021002C);

void displaySensorDetails(void) {
  sensor_t accel, mag , gyr;
  accelmag.getSensor(&accel, &mag);
  gyro.getSensor(&gyr);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(gyr.name);
  Serial.print("Driver Ver:   ");
  Serial.println(gyr.version);
  Serial.print("Unique ID:    0x");
  Serial.println(gyr.sensor_id, HEX);
  Serial.print("Max Value:    ");
  Serial.print(gyr.max_value);
  Serial.println(" rad/s");
  Serial.print("Min Value:    ");
  Serial.print(gyr.min_value);
  Serial.println(" rad/s");
  Serial.print("Resolution:   ");
  Serial.print(gyr.resolution);
  Serial.println(" rad/s");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(10000);
}

void setup(void) {
  Serial.begin(115200);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }
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

  accelmag.setAccelRange(ACCEL_RANGE_8G);
  accelmag.setOutputDataRate(ODR_200HZ);
  gyro.setRange(GYRO_RANGE_500DPS);

  /* Display some basic information on this sensor */
  displaySensorDetails();

}

void loop(void) {
  sensors_event_t aevent, mevent, gevent;

  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent);
  gyro.getEvent(&gevent);

  /* Display the accel results (acceleration is measured in m/s^2) */
  Serial.print("A ");
  Serial.print("X: ");
  Serial.print(aevent.acceleration.x, 4);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(aevent.acceleration.y, 4);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(aevent.acceleration.z, 4);
  Serial.print("  ");
  Serial.println("m/s^2");

  /* Display the mag results (mag data is in uTesla) */
  Serial.print("M ");
  Serial.print("X: ");
  Serial.print(mevent.magnetic.x, 1);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(mevent.magnetic.y, 1);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(mevent.magnetic.z, 1);
  Serial.print("  ");
  Serial.println("uT");

  /* Display the results (speed is measured in rad/s) */
  Serial.print("G ");
  Serial.print("X: ");
  Serial.print(gevent.gyro.x);
  Serial.print("  ");
  Serial.print("Y: ");
  Serial.print(gevent.gyro.y);
  Serial.print("  ");
  Serial.print("Z: ");
  Serial.print(gevent.gyro.z);
  Serial.print("  ");
  Serial.println("rad/s ");

  Serial.println("");

<<<<<<< Updated upstream
  delay(5);
=======
  delay(500);
>>>>>>> Stashed changes
}
