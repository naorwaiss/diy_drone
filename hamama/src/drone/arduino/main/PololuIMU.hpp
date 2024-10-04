#ifndef POLOLUIMU_H
#define POLOLUIMU_H

#include <Wire.h>

class PololuIMU {
public:
    PololuIMU();  // Constructor

    void begin();           // Initialize the IMU (LSM6DS33)
    void readLSM6DS33();    // Read accelerometer and gyroscope data
    void Read_data();       // Trigger the IMU reading
    float* getSensorData(); // Get pointer to sensor data array

private:
    float sensorData[6];    // Array to store IMU (gyro + accelerometer) data

    // Constants for scaling sensor readings
    const float ACCEL_SCALE = 0.061; // Accelerometer scale factor
    const float GYRO_SCALE = 8.75;   // Gyroscope scale factor

    // IMU sensor address
    const int LSM6DS33_ADDRESS = 0x6B; 
};

#endif
