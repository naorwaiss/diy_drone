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


#define UDP_BRIDGE 1
#define SEND_DATA 1
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define MPU_TICK_RATE 10000
#define MPU 'm'
#define POLOLU 'p'
#define COMBINE_IMU 'c'
#define RC 'r'





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





////////////////Instances of IMUs/////////////////////////////
PololuIMU pololuImu;     // Instance of Pololu IMU
MPU6050 mpuimu(0x68);    // Instance of MPU6050 with I2C address 0x68
IMUCombiner imuCombiner;

// IMUCombiner imuCombiner(mpuimu, pololuImu);

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


unsigned long previousTime = 0;


///////////////////////////////////////read imu data ////////////////////////////////
void readIMUs() {
    unsigned long currentTime = millis();
    unsigned long timeTaken = currentTime - previousTime;

    mpuimu.updateData();
    memcpy(mpu6050_data, mpuimu.getData(), 6 * sizeof(float));
    memcpy(pololu_data, pololuImu.getSensorData(), 6 * sizeof(float));
    memcpy(combine_data, imuCombiner.combine_imu(mpu6050_data, pololu_data), 13 * sizeof(float));
    crsf.update();
    save_ch();
    // memccpy(rc_ch,crsf.getChannel(),16*sizeof(int));
    Serial.print("time at milesec: ");
    Serial.println(timeTaken);
    
    previousTime = currentTime;

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






void loop() {

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
    
    delay(10);







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