#ifndef POLOLUIMU_HPP
#define POLOLUIMU_HPP

#include <Wire.h>

class PololuIMU {
public:
    // Constructor
    PololuIMU();

    // Methods for initializing and reading data
    void begin();
    void readLSM6DS33();
    void readLIS3MDL();
    void readLPS25H();
    void Read_data();
    float sensorData[10];
    float* getSensorData();


private:
    // I2C addresses
    static const uint8_t LSM6DS33_ADDRESS = 0x6B;
    static const uint8_t LIS3MDL_ADDRESS = 0x1E;
    static const uint8_t LPS25H_ADDRESS = 0x5D;

    // Conversion factors
    const float ACCEL_SCALE = 0.061;
    const float GYRO_SCALE = 8.75;
    const float MAG_SCALE = 0.14;
    const float PRESSURE_SCALE = 4096.0;
};

#endif // POLOLUIMU_HPP
