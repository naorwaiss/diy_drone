#include "PololuIMU.hpp"
#include <Wire.h>

// Constructor
PololuIMU::PololuIMU() {
    // Initialize the sensor data array with zeros
    for (int i = 0; i < 6; i++) {
        sensorData[i] = 0.0;
    }
}

// Method to initialize LSM6DS33 (Gyro + Accelerometer)
void PololuIMU::begin() {
    Wire.begin();

    // Initialize LSM6DS33 (Gyro + Accelerometer)
    Wire.beginTransmission(LSM6DS33_ADDRESS);
    Wire.write(0x10); // CTRL1_XL register
    Wire.write(0x80); // Set to 1.66 kHz, 2g, 100 Hz filter
    Wire.endTransmission();
  
    Wire.beginTransmission(LSM6DS33_ADDRESS);
    Wire.write(0x11); // CTRL2_G register
    Wire.write(0x80); // Set to 1.66 kHz, 245 dps
    Wire.endTransmission();
}

// Method to read accelerometer and gyroscope data
void PololuIMU::readLSM6DS33() {
    // Reading accelerometer data
    Wire.beginTransmission(LSM6DS33_ADDRESS);
    Wire.write(0x28); // OUTX_L_XL register
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DS33_ADDRESS, 6); // Request 6 bytes of data

    if (Wire.available() == 6) {
        int16_t ax = Wire.read() | (Wire.read() << 8);
        int16_t ay = Wire.read() | (Wire.read() << 8);
        int16_t az = Wire.read() | (Wire.read() << 8);

        // Convert raw values to g's and store in array
        sensorData[0] = ax * ACCEL_SCALE / 1000.0; // Accel X
        sensorData[1] = ay * ACCEL_SCALE / 1000.0; // Accel Y
        sensorData[2] = az * ACCEL_SCALE / 1000.0; // Accel Z
    }

    // Reading gyroscope data
    Wire.beginTransmission(LSM6DS33_ADDRESS);
    Wire.write(0x22); // OUTX_L_G register
    Wire.endTransmission(false);
    Wire.requestFrom(LSM6DS33_ADDRESS, 6); // Request 6 bytes

    if (Wire.available() == 6) {
        int16_t gx = Wire.read() | (Wire.read() << 8);
        int16_t gy = Wire.read() | (Wire.read() << 8);
        int16_t gz = Wire.read() | (Wire.read() << 8);

        // Convert raw values to dps and store in array
        sensorData[3] = gx * GYRO_SCALE / 1000.0; // Gyro X
        sensorData[4] = gy * GYRO_SCALE / 1000.0; // Gyro Y
        sensorData[5] = gz * GYRO_SCALE / 1000.0; // Gyro Z
    }
}

// Method to activate the LSM6DS33 sensor and store data
void PololuIMU::Read_data() {
    readLSM6DS33();
}

// Method to return a pointer to the sensor data array
float* PololuIMU::getSensorData() {
    Read_data();
    return sensorData;
}
