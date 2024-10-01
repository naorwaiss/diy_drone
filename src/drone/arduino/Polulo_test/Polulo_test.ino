#include "PololuIMU.h"

PololuIMU* myIMU;  // Declare a pointer to the PololuIMU class

void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    myIMU = new PololuIMU();  // Initialize the PololuIMU object
}

void loop() {
    float* data = myIMU->readIMU();  // Get a pointer to the IMU data (in SI units)

    // Labels for the output
    const char* labels[6] = { "Accel X (g): ", "Accel Y (g): ", "Accel Z (g): ",
                              "Gyro X (dps): ", "Gyro Y (dps): ", "Gyro Z (dps): " };

    // Loop through the 6 data points (3 for accelerometer, 3 for gyroscope)
    for (int i = 0; i < 6; i++) {
        Serial.print(labels[i]);  // Print the label
        Serial.print(data[i], 3); // Print the data with 3 decimal places
        Serial.print("  ");       // Print a space between the values
    }
    
    Serial.println();  // Move to the next line after all values are printed

    delay(100);  // Delay for readability
}

