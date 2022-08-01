/**
 * Example of 50 Hz (+harmonics) notch filter.
 * 
 * @boards  AVR, AVR USB, Nano 33 IoT, Nano 33 BLE, Due, Teensy 3.x, ESP8266, ESP32
 * 
 * @see <https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/FIR-Notch.html>
 * 
 * Be careful when selecting a sampling frequency that's a multiple of 50 Hz, as
 * this will alias 50 Hz harmonics to 0 Hz (DC), and this might introduce a more
 * or less constant error to your measurements.  
 * It's best to add an analog anti-aliasing filter as well.
 * 
 * ![Filtered mains power noise signal (blue is unfilterd, red is filtered)](50Hz-notch.png)
 * 
 * Written by PieterP, 2019-11-22  
 * https://github.com/tttapa/Arduino-Filters
 */
#include <Wire.h>
#include "GY521.h"
#include <Filters.h>

#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Notch.hpp>

#define RAW2G 1.0/16384.0;

GY521 sensor(0x68);

double accX;

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


void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_sensor();
  delay(100); // Wait for sensor to stabilize
}

// Sampling frequency
const double f_s = 100; // Hz
// Notch frequency (-∞ dB)
const double f_c = 1000; // Hz
// Normalized notch frequency
const double f_n = 2 * f_c / f_s;

// Sample timer
Timer<micros> timer = std::round(1e6 / f_s);

// Very simple Finite Impulse Response notch filter
auto filter1 = simpleNotchFIR(f_n);     // fundamental
auto filter2 = simpleNotchFIR(2 * f_n); // second harmonic

void loop() {
  if (timer) {
    sensor.read();
    accX = sensor.getAccelX();
    auto raw = accX * 980.665;
    Serial.print(raw);
    Serial.print('\t');
    Serial.println(filter2(filter1(raw)));
  }
}
