#include "PololuIMU.h"

// Constructor: Initialize and configure the IMU
PololuIMU::PololuIMU() {
    if (!imu.init()) {
        Serial.println("Failed to detect and initialize IMU!");
        while (1);  // Halt if initialization fails
    }
    imu.enableDefault();
}

// Method to read IMU data and convert to SI units
float* PololuIMU::readIMU() {
    imu.read();

    // Convert raw accelerometer data (16-bit) to g (SI unit)
    imuData[0] = imu.a.x * ACC_CONVERSION;  // X-axis in g
    imuData[1] = imu.a.y * ACC_CONVERSION;  // Y-axis in g
    imuData[2] = imu.a.z * ACC_CONVERSION;  // Z-axis in g

    // Convert raw gyroscope data (16-bit) to dps (degrees per second)
    imuData[3] = imu.g.x * GYRO_CONVERSION;  // X-axis in dps
    imuData[4] = imu.g.y * GYRO_CONVERSION;  // Y-axis in dps
    imuData[5] = imu.g.z * GYRO_CONVERSION;  // Z-axis in dps

    return imuData;  // Return pointer to the array with converted values
}
