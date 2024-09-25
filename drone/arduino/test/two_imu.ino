#include <TimerOne.h>
#include "PololuIMU.hpp"
#include "MPU6050.h"

// Instances of IMUs
PololuIMU pololuImu;     // Instance of Pololu IMU
MPU6050 mpuimu(0x68);    // Instance of MPU6050 with I2C address 0x68

// Function to handle the reading of both IMUs
void readIMUs() {
    // Read data from Pololu IMU
    float* pololuData = pololuImu.getSensorData();  // Get pointer to Pololu sensor data
    const char* pololuLabels[] = {"Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z", 
                                  "Magnet X", "Magnet Y", "Magnet Z", "Pressure (hPa)"};

    // Read data from MPU6050
    mpuimu.updateData();  // Update the MPU6050 sensor data
    float* mpuData = mpuimu.getData();  // Get pointer to MPU6050 sensor data
    const char* mpuLabels[] = {"Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z"};

    // Print Pololu IMU sensor data using a loop
    Serial.println("Pololu IMU Sensor Data:");
    for (int i = 0; i < 10; i++) {
        Serial.print(pololuLabels[i]); 
        Serial.print(": "); 
        Serial.println(pololuData[i]);
    }

    // Print MPU6050 sensor data using a loop
    Serial.println("MPU6050 Sensor Data:");
    for (int i = 0; i < 6; i++) {
        Serial.print(mpuLabels[i]);
        Serial.print(": "); 
        Serial.println(mpuData[i]);
    }

    Serial.println();  // Empty line for better readability
}

void setup() {
    Serial.begin(9600);
    
    // Initialize Pololu IMU and MPU6050
    pololuImu.begin();
    mpuimu.init();

    // Set up TimerOne to trigger every 1 second (1,000,000 microseconds)
    Timer1.initialize(1000000);  // 1 second
    Timer1.attachInterrupt(readIMUs);  // Attach the readIMUs function to the timer
}

void loop() {
    // Main loop does nothing, IMU data is handled by the timer interrupt
}

