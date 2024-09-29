#include "MPU6050.h"

// Constructor
MPU6050::MPU6050(int address) : imuAddress(address) {
    for (int i = 0; i < 6; i++) {
        imuData[i] = 0.0;
    }
}

// Initialize MPU6050
void MPU6050::init() {
    Wire.beginTransmission(imuAddress);
    Wire.write(0x6B);  // Power management register
    Wire.write(0x00);  // Wake up the MPU6050 by setting 0 to this register
    Wire.endTransmission(true);

    // Accelerometer range ±2g
    Wire.beginTransmission(imuAddress);
    Wire.write(0x1C);  // Accelerometer configuration register
    Wire.write(0x00);  // Set full-scale range to ±2g
    Wire.endTransmission(true);

    // Gyroscope range ±250 deg/s
    Wire.beginTransmission(imuAddress);
    Wire.write(0x1B);  // Gyroscope configuration register
    Wire.write(0x00);  // Set full-scale range to ±250 deg/s
    Wire.endTransmission(true);
}

// Update MPU6050 data
void MPU6050::updateData() {
    Wire.beginTransmission(imuAddress);
    Wire.write(0x3B);  // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(imuAddress, 14, true);  // Request 14 bytes of data

    if (Wire.available() == 14) {
        int16_t accelX = (Wire.read() << 8) | Wire.read();
        int16_t accelY = (Wire.read() << 8) | Wire.read();
        int16_t accelZ = (Wire.read() << 8) | Wire.read();
        int16_t temp = (Wire.read() << 8) | Wire.read();  // Skip temperature
        int16_t gyroX = (Wire.read() << 8) | Wire.read();
        int16_t gyroY = (Wire.read() << 8) | Wire.read();
        int16_t gyroZ = (Wire.read() << 8) | Wire.read();

        imuData[0] = accelX / 16384.0;
        imuData[1] = accelY / 16384.0;
        imuData[2] = accelZ / 16384.0;
        imuData[3] = gyroX / 131.0;
        imuData[4] = gyroY / 131.0;
        imuData[5] = gyroZ / 131.0;
    }
}

// Get pointer to MPU6050 data array
float* MPU6050::getData() {
    return imuData;
}
