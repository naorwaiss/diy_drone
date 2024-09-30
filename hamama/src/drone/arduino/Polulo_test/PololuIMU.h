#ifndef POLOLUIMU_H
#define POLOLUIMU_H

#include <Wire.h>
#include <LSM6.h>

class PololuIMU {
private:
    LSM6 imu;
    float imuData[6];  // Array to hold accelerometer (g) and gyroscope (dps) data
    const float ACC_CONVERSION = 0.061 * 0.001; // mg/LSB to g
    const float GYRO_CONVERSION = 8.75 * 0.001; // mdps/LSB to dps

public:
    PololuIMU();
    float* readIMU();  // Returns pointer to float array with SI unit data
};

#endif
