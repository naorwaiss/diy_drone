#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "PololuIMU.hpp"
#include "src/Rtes/Rtes.h"
#include <stdio.h>
#include <imuFilter.h>
 

#include <iostream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <iomanip>
#include <cmath>





#define UDP_BRIDGE 1 // 1 to make the bridge of the data 
#define SEND_DATA 1 // send data 
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define MPU_TICK_RATE 10000
#define POLOLU 'p'
#define EUILER 'e'
#define RC 'r'
#define RC_ros 'n'
#define Qurtirion 'q'






#if UDP_BRIDGE ==1 

constexpr uint8_t IP_ADDRESS[4] = {192, 168,1, 199};
constexpr uint16_t PORT_NUMBER = 8888;
const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
Rtes rtesSocket(SOCKET_ADDRESS);
RtesSession* currentSession = nullptr;

std::vector<uint8_t> quterion_bytes = {};
std::vector<uint8_t> euiler_bytes = {};
std::vector<uint8_t> pololu_bytes = {};
std::vector<uint8_t> rc_bytes = {};


void onConnect(RtesSession &session){
  currentSession = &session;

}

#endif

float* pololu_data = (float*)calloc(6, sizeof(float));
float* euiler = (float*)calloc(3, sizeof(float));
float* quartirion_data = (float*)calloc(4, sizeof(float));
int* rc_ch_data = (int*)calloc(16, sizeof(int));
std::vector<std::vector<float>> upsampleFusedIMUData(float* data, int numSamples, int upsampleFactor);



////////////////Instances of IMUs/////////////////////////////
PololuIMU pololuImu;     // Instance of Pololu IMU

Madgwick filter; // Create Madgwick filter instance


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



/////////////////////////////////////////////////////////////////setup//////////////////////////////////////////////////////
void setup(){
    Serial.begin(115200);
    while (!Serial);
    pololuImu.begin();

    #if UDP_BRIDGE == 1 
    rtesSocket.begin(1);
    rtesSocket.onConnection(onConnect);
    #endif



 ///////activate crsf//////

    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!crsfSerial) {
        while (1) {
        Serial.println("Invalid crsfSerial configuration");
        }
    }

    crsf.begin(crsfSerial);
}




void updateMadgwick(float ax, float ay, float az, float gx, float gy, float gz) {
    // Update the Madgwick filter with IMU data (gyroscope and accelerometer)
    filter.updateIMU(gx, gy, gz, ax, ay, az);  // Madgwick filter update method
}



unsigned long current_time = 0;  // Global variable to store the loop start time

void loopRate(int freq) {
  // Regulate the main loop rate to the specified frequency in Hz
  float invFreq = 1.0 / freq * 1000000.0;  // Convert frequency to time interval in microseconds
  unsigned long checker = micros();
  
  // Wait until the appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}






void loop() {
    current_time = micros();
    memcpy(pololu_data, pololuImu.getSensorData(), 6 * sizeof(float)); // Assume this gives you ax, ay, az, gx, gy, gz


    //   // Convert accelerometer data to m/s²
    // pololu_data[0] *= 9.81;  // ax
    // pololu_data[1] *= 9.81;  // ay
    // pololu_data[2] *= 9.81;  // az

    // // Convert gyroscope data to rad/s
    // pololu_data[3] *= (M_PI / 180);  // gx
    // pololu_data[4] *= (M_PI / 180);  // gy
    // pololu_data[5] *= (M_PI / 180);  // gz
    
    // Serial.print(pololu_data[0]);
    // PP;
    // Serial.print(pololu_data[1]);
    // PP;
    // Serial.print(pololu_data[2]);
    // PP;
    // Serial.print(pololu_data[3]);
    // PP;
    // Serial.print(pololu_data[4]);
    // PP;
    // Serial.print(pololu_data[5]);
    // Serial.println(" ");


    updateMadgwick(pololu_data[0], pololu_data[1], pololu_data[2], pololu_data[3], pololu_data[4], pololu_data[5]);



    euiler[0] = (filter.getRoll())* 180.0 / M_PI;
    euiler[1] = (filter.getPitch()) * 180.0 / M_PI;
    euiler[2] = (filter.getYaw()) * 180.0 / M_PI;


    Serial.print(euiler[0]);
    Serial.print(" ,");
    Serial.print(euiler[1]);
    Serial.print(" ,");
    Serial.print(euiler[2]);
    Serial.println();
    

    eulerToQuaternion(euiler[0], euiler[1], euiler[2]);



    // Serial.print(quartirion_data[0]);
    // Serial.print(" ,");
    // Serial.print(quartirion_data[1]);
    // Serial.print(" ,");
    // Serial.print(quartirion_data[2]);
    // Serial.print();
    // Serial.print(quartirion_data[3]);
    // Serial.println();

    // Continue with the rest of your logic
    crsf.update();
    save_ch();

    #if UDP_BRIDGE ==1 
    rtesSocket.process();
    #endif

    #if SEND_DATA == 1 && UDP_BRIDGE ==1
        if (currentSession != nullptr) {
            std::vector<uint8_t> bytes;

            // Send Pololu IMU data
            bytes = floatsToBytes(pololu_data, 6);
            pololu_bytes.assign(bytes.begin(), bytes.end());

            // Send RC data
            bytes = intToBytes(rc_ch_data,16);
            rc_bytes.assign(bytes.begin(), bytes.end());

            // Send Euler angles data (roll, pitch, yaw)
            bytes = floatsToBytes(euiler, 3);
            euiler_bytes.assign(bytes.begin(), bytes.end());

            bytes = floatsToBytes(quartirion_data, 4);
            quterion_bytes.assign(bytes.begin(), bytes.end());



            if (pololu_bytes.size() != 0) currentSession->emitRaw(POLOLU, pololu_bytes);
            if (rc_bytes.size() != 0) currentSession->emitRaw(RC, rc_bytes);
            if (euiler_bytes.size() != 0) currentSession->emitRaw(EUILER, euiler_bytes);
            if (quterion_bytes.size() != 0) currentSession->emitRaw(Qurtirion, quterion_bytes);


        }
    #endif


loopRate(2000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default

}


/////////////////////////////////////////////////////////onReceiveRcChannels//////////////////////////////////////////////////////////


void save_ch() {
    for (int i = 0; i < 16; i++) {
        rc_ch_data[i] = crsf.getChannel(i);  // Store each RC channel value into rc_ch_data
    }
}


void eulerToQuaternion(float roll, float pitch, float yaw) {
    // Convert Euler angles (roll, pitch, yaw) to quaternion
    // Roll (x-axis), Pitch (y-axis), Yaw (z-axis)

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    quartirion_data[0] = cr * cp * cy + sr * sp * sy;
    quartirion_data[1] = sr * cp * cy - cr * sp * sy;
    quartirion_data[2] = cr * sp * cy + sr * cp * sy;
    quartirion_data[3] = cr * cp * sy - sr * sp * cy;

}

