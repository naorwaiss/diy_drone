#include <MadgwickAHRS.h>  // Include Madgwick filter library
#include "MPU6050.h"       // Include your MPU6050 IMU header
#include "PololuIMU.hpp"   // Include your Pololu IMU header
#include <cmath>           // For sin and cos functions

class IMUCombiner {
  private:
    MPU6050* mpu;            // Pointer to the MPU6050 IMU instance
    PololuIMU* pololu;       // Pointer to the Pololu IMU instance
    Madgwick madgwickFilter; // Instance of the Madgwick filter
    float fusedData[13];     // Array to store fused data (Roll, Pitch, Yaw, Quaternions, Accelerometer, Gyroscope)

  public:
    // Constructor that takes pointers to the two IMU instances
    IMUCombiner(MPU6050* mpuPtr, PololuIMU* pololuPtr) {
      mpu = mpuPtr;
      pololu = pololuPtr;
    }

    // Function to combine IMU data using the Madgwick filter with weighted data fusion
    float* combine_imu() {
      // Get raw data from MPU6050 (accX, accY, accZ, gyroX, gyroY, gyroZ)
      mpu->updateData();
      float* mpuData = mpu->getData();

      // Get raw data from Pololu IMU (accX, accY, accZ, gyroX, gyroY, gyroZ)
      float* pololuData = pololu->getSensorData();

      // Weighted fusion of accelerometer and gyroscope data
      float accX = 0.35f * mpuData[0] + 0.65f * pololuData[0];
      float accY = 0.35f * mpuData[1] + 0.65f * pololuData[1];
      float accZ = 0.35f * mpuData[2] + 0.65f * pololuData[2];
      float gyroX = 0.35f * mpuData[3] + 0.65f * pololuData[3];
      float gyroY = 0.35f * mpuData[4] + 0.65f * pololuData[4];
      float gyroZ = 0.35f * mpuData[5] + 0.65f * pololuData[5];

      // Apply the Madgwick filter using the fused accelerometer and gyroscope data
      madgwickFilter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

      // Convert quaternion to Euler angles (roll, pitch, yaw)
      fusedData[0] = madgwickFilter.getRoll();  // Roll (phi)
      fusedData[1] = madgwickFilter.getPitch(); // Pitch (theta)
      fusedData[2] = madgwickFilter.getYaw();   // Yaw (psi)

      // Calculate quaternions based on the roll, pitch, and yaw
      float roll = fusedData[0];
      float pitch = fusedData[1];
      float yaw = fusedData[2];

      float halfRoll = roll / 2.0f;
      float halfPitch = pitch / 2.0f;
      float halfYaw = yaw / 2.0f;

      float cosRoll = cos(halfRoll);
      float sinRoll = sin(halfRoll);
      float cosPitch = cos(halfPitch);
      float sinPitch = sin(halfPitch);
      float cosYaw = cos(halfYaw);
      float sinYaw = sin(halfYaw);

      fusedData[3] = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;  // q1 (w)
      fusedData[4] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;  // q2 (x)
      fusedData[5] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;  // q3 (y)
      fusedData[6] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;  // q4 (z)

      // Add the fused accelerometer data
      fusedData[7] = accX;  // Fused accX
      fusedData[8] = accY;  // Fused accY
      fusedData[9] = accZ;  // Fused accZ

      // Add the fused gyroscope data
      fusedData[10] = gyroX;  // Fused gyroX
      fusedData[11] = gyroY;  // Fused gyroY
      fusedData[12] = gyroZ;  // Fused gyroZ

      // Return pointer to the fused data array
      return fusedData;
    }
};
