#include "GY521_csv.h"

GY521_csv::GY521_csv(int AD0_level) : sensor(GY521(AD0_level)) {}

void GY521_csv::setColumns(int columns) { col = columns; }
void GY521_csv::setColumns(char* counterFormat, int columns) { 
    for (int i = 0; i < COUNTER_FORMAT_LENGTH; i++ ) {
        cntFormat[i] = counterFormat[i];
    }
    setColumns(columns);
}

void GY521_csv::update() {
    sensor.read();
    if (col &              0b1) counter++;
    if (col &             0b10) accelx = sensor.getAccelX();
    if (col &            0b100) accely = sensor.getAccelY();
    if (col &           0b1000) accelz = sensor.getAccelZ();
    if (col &          0b10000) anglex = sensor.getAngleX();
    if (col &         0b100000) angley = sensor.getAngleY();
    if (col &        0b1000000) anglez = sensor.getAngleZ();
    if (col &       0b10000000) gyrox  = sensor.getGyroX(); 
    if (col &      0b100000000) gyroy  = sensor.getGyroY(); 
    if (col &     0b1000000000) gyroz  = sensor.getGyroZ(); 
    if (col &    0b10000000000) pitch  = sensor.getPitch(); 
    if (col &   0b100000000000) roll   = sensor.getRoll();  
    if (col &  0b1000000000000) yaw    = sensor.getYaw();   
    if (col & 0b10000000000000) temp   = sensor.getTemperature();
}
void GY521_csv::printHeader() {
    if (col &              0b1) { 
                                Serial.print("Counter_");
                                Serial.print(cntFormat);
                                Serial.print(';'); 
    }
    if (col &             0b10) Serial.print("Accelx;");
    if (col &            0b100) Serial.print("Accely;");
    if (col &           0b1000) Serial.print("Accelz;");
    if (col &          0b10000) Serial.print("Anglex;");
    if (col &         0b100000) Serial.print("Angley;");
    if (col &        0b1000000) Serial.print("Anglez;");
    if (col &       0b10000000) Serial.print("Gyrox;");
    if (col &      0b100000000) Serial.print("Gyroy;");
    if (col &     0b1000000000) Serial.print("Gyroz;");
    if (col &    0b10000000000) Serial.print("Pitch;");
    if (col &   0b100000000000) Serial.print("Roll;");
    if (col &  0b1000000000000) Serial.print("Yaw;");
    if (col & 0b10000000000000) Serial.print("Temperature;");
    Serial.println();
}
void GY521_csv::printData() {
    if (col &              0b1) { Serial.print(counter);   Serial.print(';'); }
    if (col &             0b10) { Serial.print(accelx, 3); Serial.print(';'); }
    if (col &            0b100) { Serial.print(accely, 3); Serial.print(';'); }
    if (col &           0b1000) { Serial.print(accelz, 3); Serial.print(';'); }
    if (col &          0b10000) { Serial.print(anglex, 3); Serial.print(';'); }
    if (col &         0b100000) { Serial.print(angley, 3); Serial.print(';'); }
    if (col &        0b1000000) { Serial.print(anglez, 3); Serial.print(';'); }
    if (col &       0b10000000) { Serial.print(gyrox,  3); Serial.print(';'); }
    if (col &      0b100000000) { Serial.print(gyroy,  3); Serial.print(';'); }
    if (col &     0b1000000000) { Serial.print(gyroz,  3); Serial.print(';'); }
    if (col &    0b10000000000) { Serial.print(pitch,  3); Serial.print(';'); }
    if (col &   0b100000000000) { Serial.print(roll,   3); Serial.print(';'); }
    if (col &  0b1000000000000) { Serial.print(yaw,    3); Serial.print(';'); }
    if (col & 0b10000000000000) { Serial.print(temp,   3); Serial.print(';'); }
    
    Serial.println();
}

void GY521_csv::initGY521() {

}

bool GY521_csv::isConnected() { return sensor.wakeup(); }

void GY521_csv::setSensitivity(int accel, int gyro) {
    sensor.setAccelSensitivity(accel); // 8 = 8g
    sensor.setGyroSensitivity(gyro);   // 1 = 500 degrees/s
}

void GY521_csv::setThrottle() { sensor.setThrottle(); }

void GY521_csv::setCalibrationValues(int axe, int aye, int aze, int gxe, int gye, int gze) {
    sensor.axe = axe;
    sensor.aye = aye;
    sensor.aze = aze;
    sensor.gxe = gxe;
    sensor.gye = gye;
    sensor.gze = gze;
}
