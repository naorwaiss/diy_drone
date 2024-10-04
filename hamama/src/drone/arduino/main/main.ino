#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "PololuIMU.hpp"
#include "MPU6050.h"
#include <IntervalTimer.h>  // Include IntervalTimer
#include "IMUCombiner.hpp"
#include "src/Rtes/Rtes.h"
#include <stdio.h>

#include <iostream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <iomanip>





#define UDP_BRIDGE 1 // 1 to make the bridge of the data 
#define SEND_DATA 1 // send data 
// #define GET_ROS_STICK 1 
// #define GET_CAM_VIO 1
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define MPU_TICK_RATE 10000
#define MPU 'm'
#define POLOLU 'p'
#define COMBINE_IMU 'c'
#define RC 'r'
#define RC_ros 'n'





#if UDP_BRIDGE ==1 

constexpr uint8_t IP_ADDRESS[4] = {192, 168,1, 199};
constexpr uint16_t PORT_NUMBER = 8888;
const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
Rtes rtesSocket(SOCKET_ADDRESS);
RtesSession* currentSession = nullptr;


std::vector<uint8_t> mpu6050_bytes = {};
std::vector<uint8_t> pololu_bytes = {};
std::vector<uint8_t> combine_bytes = {};
std::vector<uint8_t> rc_bytes = {};


void onConnect(RtesSession &session){
  currentSession = &session;

}

#endif

float* mpu6050_data = (float*)calloc(6, sizeof(float));
float* pololu_data = (float*)calloc(6, sizeof(float));
float* combine_data = (float*)calloc(13, sizeof(float));
int* rc_ch_data = (int*)calloc(16, sizeof(int));
std::vector<std::vector<float>> upsampleFusedIMUData(float* data, int numSamples, int upsampleFactor);



////////////////Instances of IMUs/////////////////////////////
PololuIMU pololuImu;     // Instance of Pololu IMU
MPU6050 mpuimu(0x68);    // Instance of MPU6050 with I2C address 0x68
IMUCombiner imuCombiner;
IntervalTimer imuTimer;  // IntervalTimer for IMU reading



////////////// RC channel count and names////////////////////////

AlfredoCRSF crsf;

///////////////////////////////////////convert to char//////////////////////


std::vector<uint8_t> floatsToBytes(float* pFloatArray, size_t size) {
    uint8_t byteArray[size * sizeof(float)];
    std::memcpy(byteArray, pFloatArray, size * sizeof(float));

    std::vector<uint8_t> bytes;

    for (size_t i = 0; i < sizeof(byteArray); i++)
        bytes.push_back(byteArray[i]);

    return bytes;
}


std::vector<uint8_t> intToBytes(int* pFloatArray, size_t size) {
    uint8_t byteArray[size * sizeof(int)];
    std::memcpy(byteArray, pFloatArray, size * sizeof(int));

    std::vector<uint8_t> bytes;

    for (size_t i = 0; i < sizeof(byteArray); i++)
        bytes.push_back(byteArray[i]);

    return bytes;
}



///////////////////////////////////////read imu data ////////////////////////////////
void readIMUs() {
    //need to check the relability of the imu data - need to think if this ok and the run time is ok also 
    // unsigned long currentTime = millis();
    // unsigned long timeTaken = currentTime - previousTime;

    mpuimu.updateData();
    memcpy(mpu6050_data, mpuimu.getData(), 6 * sizeof(float));
    memcpy(pololu_data, pololuImu.getSensorData(), 6 * sizeof(float));
    memcpy(combine_data, imuCombiner.combine_imu(mpu6050_data, pololu_data), 13 * sizeof(float));
    crsf.update();
    save_ch();

}

/////////////////////////////////////////////////////////////////setup//////////////////////////////////////////////////////
void setup(){
    Serial.begin(115200);
    while (!Serial);
    pololuImu.begin();
    mpuimu.init();

    #if UDP_BRIDGE == 1 
    rtesSocket.begin(1);
    rtesSocket.onConnection(onConnect);
    #endif


    imuTimer.begin(readIMUs, MPU_TICK_RATE);

 ///////activate crsf//////

    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!crsfSerial) {
        while (1) {
        Serial.println("Invalid crsfSerial configuration");
        }
    }

    crsf.begin(crsfSerial);
}



// Assuming you store fused IMU data for 100 Hz readings
int numSamples = 100;  // Example: Buffer of 100 readings
int upsampleFactor = 20;  // From 100 Hz to 2000 Hz

unsigned long previousTime = 0;

void loop() {
    unsigned long currentTime = millis();
    unsigned long timeTaken = currentTime - previousTime;

    // std::vector<std::vector<float>> interpolatedIMUData = upsampleFusedIMUData(combine_data, numSamples, upsampleFactor);

    #if UDP_BRIDGE ==1 
    rtesSocket.process();
    #endif

    #if SEND_DATA == 1 && UDP_BRIDGE ==1

        if (currentSession != nullptr) {
            std::vector<uint8_t> bytes;

            bytes = floatsToBytes(mpu6050_data, 6);
            mpu6050_bytes.assign(bytes.begin(), bytes.end());

            bytes = floatsToBytes(pololu_data, 6);
            pololu_bytes.assign(bytes.begin(), bytes.end());

            bytes = floatsToBytes(combine_data, 13);
            combine_bytes.assign(bytes.begin(), bytes.end());

            bytes = intToBytes(rc_ch_data,16);
            rc_bytes.assign(bytes.begin(), bytes.end());


            if (mpu6050_bytes.size() != 0) currentSession->emitRaw(MPU, mpu6050_bytes);
            if (pololu_bytes.size() != 0) currentSession->emitRaw(POLOLU, pololu_bytes);
            if (combine_bytes.size() != 0) currentSession->emitRaw(COMBINE_IMU, combine_bytes);
            if (rc_bytes.size() != 0) currentSession->emitRaw(RC, rc_bytes);

        }
    #endif

    Serial.print("time at milesec: ");
    Serial.println(timeTaken);

    previousTime = currentTime;

    // std::vector<std::vector<float>> interpolatedIMUData = upsampleFusedIMUData(combine_data, numSamples, upsampleFactor);


    // //need to check it 
    //   currentSession->on(RC_ros, [](const char* data) {
    //         Serial.print("Received RC from ROS: ");
    //         // You can process the received data here
    //         Serial.println(data);
    //     });

    // delay(1);







// float naor = random(0,100);
// // Serial.println(naor);

// if (currentSession == nullptr)
//   return;




//  char b[50];
//  std::sprintf(b, "%.2f", naor);
//   currentSession->emit(POLOLU, b);


 }

/////////////////////////////////////////////////////////onReceiveRcChannels//////////////////////////////////////////////////////////


void save_ch() {
    for (int i = 0; i < 16; i++) {
        rc_ch_data[i] = crsf.getChannel(i);  // Store each RC channel value into rc_ch_data
    }
}


// float cubicInterpolate(float y0, float y1, float y2, float y3, float mu) {
//     float a0, a1, a2, a3, mu2;

//     mu2 = mu * mu;
//     a0 = y3 - y2 - y0 + y1;
//     a1 = y0 - y1 - a0;
//     a2 = y2 - y0;
//     a3 = y1;

//     return (a0 * mu * mu2 + a1 * mu2 + a2 * mu + a3);
// }



// std::vector<std::vector<float>> upsampleFusedIMUData(float* data, int numSamples, int upsampleFactor) {
//     std::vector<std::vector<float>> interpolatedData;

//     // Example assumes 13 elements in each data point for fused IMU data
//     for (int i = 0; i < 13; i++) {
//         std::vector<float> axisInterpolated;
//         for (int j = 0; j < numSamples - 3; j++) { // For each set of 4 points
//             for (int k = 0; k < upsampleFactor; k++) { // Interpolate upsampleFactor points between each set
//                 float t = float(k) / upsampleFactor;  // Interpolation parameter (0 <= t <= 1)
//                 float interpolatedValue = cubicInterpolate(
//                     data[j * 13 + i],  // y0
//                     data[(j + 1) * 13 + i],  // y1
//                     data[(j + 2) * 13 + i],  // y2
//                     data[(j + 3) * 13 + i],  // y3
//                     t);
//                 axisInterpolated.push_back(interpolatedValue);
//             }
//         }
//         interpolatedData.push_back(axisInterpolated);
//     }

//     return interpolatedData;
// }