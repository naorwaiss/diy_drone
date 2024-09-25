#ifndef UDP_PUBLISHER_HPP
#define UDP_PUBLISHER_HPP

#include <iostream>
#include <string>
#include <array>

class UdpPublisher {
public:
    UdpPublisher(); // Constructor
    ~UdpPublisher(); // Destructor

    // Setters for the data arrays
    void setRcArray(int32_t* rcArrayPtr, size_t size);
    void setMpu6050Array(float* mpu6050ArrayPtr, size_t size);
    void setPololuArray(float* pololuArrayPtr, size_t size);

    // Function to print the data
    void printData();

private:
    // Pointers to the arrays
    int32_t* rcArray_;
    size_t rcArraySize_;
    float* mpu6050Array_;
    size_t mpu6050ArraySize_;
    float* pololuArray_;
    size_t pololuArraySize_;
};

#endif // UDP_PUBLISHER_HPP
