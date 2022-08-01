#ifndef GY521_CSV_H
#define GY521_CSV_H

/**
 * @file GY521_csv.h
 * @author Romain Saboret
 * @brief 
 * @version 0.1
 * @date 2022-01-07
 * 
 * @copyright Copyright (c) 2022
 * 
 * Drop this file with the cpp file and use 
 * #include "GY521_csv.h" to use this library
 */

#include "GY521.h"

/** 
 * @brief Counter format parameters
 * 
 * length and default string for the counter
 * 
 */
const int   COUNTER_FORMAT_LENGTH = 6;
#define COUNTER_FORMAT_STRING "______" // Must match the characters length

/**
 * @brief AD0 logic levels
 * 
 * the AD0 pin on the GY521 module
 * must be set to high or low
 * 
 */
const int AD0_HIGH = 0x69;
const int AD0_LOW  = 0x68;

/**
 * @brief GY521 fields
 * 
 * All the different GY521 output data
 * Each data on a different byte
 * 
 */
enum GY521_fields {
    COUNTER           = 0b1,
    ACCELX           = 0b10,  ACCELY        = 0b100, ACCELZ          = 0b1000,
    ANGLEX        = 0b10000,  ANGLEY     = 0b100000, ANGLEZ       = 0b1000000,
    GYROX      = 0b10000000,  GYROY   = 0b100000000, GYROZ     = 0b1000000000,
    PITCH   = 0b10000000000,  ROLL = 0b100000000000, YAW    = 0b1000000000000,
    TEMP = 0b10000000000000,
};

/**
 * @brief GY521 abstraction to CSV
 * 
 * class to abstract the GY521 output and 
 * provide csv format for the output
 * 
 */
class GY521_csv {
    private:
        char cntFormat[COUNTER_FORMAT_LENGTH + 1] = COUNTER_FORMAT_STRING;
        int counter  = 0;
        int col      = 0x00000000000000;
        float accelx = 0.0;
        float accely = 0.0;
        float accelz = 0.0;
        float anglex = 0.0;
        float angley = 0.0;
        float anglez = 0.0;
        float gyrox  = 0.0;
        float gyroy  = 0.0;
        float gyroz  = 0.0;
        float pitch  = 0.0;
        float roll   = 0.0;
        float yaw    = 0.0;
        float temp   = 0.0;
        
    public:
        GY521 sensor;

        GY521_csv(int AD0_level);

        void setColumns(int columns);
        void setColumns(char* counterFormat, int columns);

        void update();

        void printHeader();
        void printData();

        void initGY521();

        bool isConnected();

        void setSensitivity(int accel, int gyro);
        void setThrottle();
        void setCalibrationValues(int axe, int aye, int aze, int gxe, int gye, int gze);
        
};

#endif