#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

class MPU6050 {
private:
    int imuAddress;
    float imuData[6];  // Array to store accelerometer and gyroscope data

public:
    MPU6050(int address = 0x68);  // Constructor with default address 0x68
    void init();                  // Initialize MPU6050
    void updateData();            // Update accelerometer and gyroscope data
    float* getData();             // Get data from MPU6050
};

#endif
