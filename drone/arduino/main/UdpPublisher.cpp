#include "UdpPublisher.hpp"
#include <Arduino.h>  // Add this to use Serial


// Constructor
UdpPublisher::UdpPublisher() 
    : rcArray_(nullptr), rcArraySize_(0), 
      mpu6050Array_(nullptr), mpu6050ArraySize_(0), 
      pololuArray_(nullptr), pololuArraySize_(0) {}

// Destructor
UdpPublisher::~UdpPublisher() {}

// Setters
void UdpPublisher::setRcArray(int32_t* rcArrayPtr, size_t size) {
    rcArray_ = rcArrayPtr;
    rcArraySize_ = size;
}

void UdpPublisher::setMpu6050Array(float* mpu6050ArrayPtr, size_t size) {
    mpu6050Array_ = mpu6050ArrayPtr;
    mpu6050ArraySize_ = size;
}

void UdpPublisher::setPololuArray(float* pololuArrayPtr, size_t size) {
    pololuArray_ = pololuArrayPtr;
    pololuArraySize_ = size;
}

// Print function to print the data from all arrays
void UdpPublisher::printData() {
    // Print RC data
    // Serial.println("RC Data:");
    // for (size_t i = 0; i < rcArraySize_; i++) {
    //     Serial.print("Channel ");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(rcArray_[i]);
    // }

    // // Print MPU6050 data
    // Serial.println("MPU6050 Data:");
    // for (size_t i = 0; i < mpu6050ArraySize_; i++) {
    //     Serial.print("MPU Data ");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(mpu6050Array_[i]);
    // }

    // // Print Pololu data
    // Serial.println("Pololu IMU Data:");
    // for (size_t i = 0; i < pololuArraySize_; i++) {
    //     Serial.print("Pololu Data ");
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.println(pololuArray_[i]);
    // }
}
