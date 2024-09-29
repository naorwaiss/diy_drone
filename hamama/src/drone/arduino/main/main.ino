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


#define SEND_DATA 1
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define MPU_TICK_RATE 1000000
#define MPU 'm'
#define POLOLU 'p'
#define COMBINE_IMU 'c'





constexpr uint8_t IP_ADDRESS[4] = {192, 168,1, 199};
constexpr uint16_t PORT_NUMBER = 8888;
const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
Rtes rtesSocket(SOCKET_ADDRESS);
RtesSession* currentSession = nullptr;

float* mpu6050_data = (float*)calloc(6, sizeof(float));
float* pololu_data = (float*)calloc(6, sizeof(float));
float* combine_data = (float*)calloc(13, sizeof(float));

std::vector<uint8_t> mpu6050_bytes = {};
std::vector<uint8_t> pololu_bytes = {};
std::vector<uint8_t> combine_bytes = {};

void onConnect(RtesSession &session){
  currentSession = &session;

}








////////////////Instances of IMUs/////////////////////////////
PololuIMU pololuImu;     // Instance of Pololu IMU
MPU6050 mpuimu(0x68);    // Instance of MPU6050 with I2C address 0x68
IMUCombiner imuCombiner(&mpuimu, &pololuImu);

IntervalTimer imuTimer;  // IntervalTimer for IMU reading








////////////// RC channel count and names////////////////////////

AlfredoCRSF crsf;


///////////////////////////////////////convert to char//////////////////////



// char* Float_Char(float* array, int size) {
//     // Dynamically allocate memory for the char buffer
//     // Each float can take up to 12 characters, and we need space for '/' separators (size-1 slashes).
//     int buffer_size = size * 12 + (size );  // Adjust buffer size based on size of array
//     char* temp = (char*)malloc(buffer_size * sizeof(char));

//     if (temp == NULL) {
//         Serial.println("Memory allocation failed!");
//         return NULL;  // Return NULL if memory allocation fails
//     }

//     memset(temp, 0, buffer_size);
//     char* ptr = temp;
//     for (int i = 0; i < size; i++) {
//         int len = snprintf(ptr, 12, "%.2f", array[i]);
//         ptr += len;
//         if (i < size - 2) {
//             *ptr++ = '/';
//             *ptr++ = NULL
            
//         }
//     }
//     return temp;
// }


std::vector<uint8_t> floatsToBytes(float* pFloatArray, size_t size) {
    uint8_t byteArray[size * sizeof(float)];
    std::memcpy(byteArray, pFloatArray, size * sizeof(float));

    std::vector<uint8_t> bytes;

    for (size_t i = 0; i < sizeof(byteArray); i++)
        bytes.push_back(byteArray[i]);

    return bytes;
}



///////////////////////////////////////read imu data ////////////////////////////////
void readIMUs() {
    mpuimu.updateData();
    memcpy(mpu6050_data, mpuimu.getData(), 6 * sizeof(float));
    memcpy(pololu_data, pololuImu.getSensorData(), 6 * sizeof(float));
    memcpy(combine_data, imuCombiner.combine_imu(), 13 * sizeof(float));
    crsf.update();



    // printChannels();


    // for (size_t i = 0; i < 6; i++)
    // {
    //     std::cout << std::fixed << std::setprecision(18);
    //     std::cout << mpu6050_data[i] << std::endl;
    //     Serial.print(" ");
    // }
    // Serial.println("");
}

/////////////////////////////////////////////////////////////////setup//////////////////////////////////////////////////////
void setup(){
    Serial.begin(115200);
    while (!Serial);
    pololuImu.begin();
    mpuimu.init();
    rtesSocket.begin(1);
    rtesSocket.onConnection(onConnect);
    Serial.println("naor");

    imuTimer.begin(readIMUs, MPU_TICK_RATE);
 

    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!crsfSerial) {
        while (1) {
        Serial.println("Invalid crsfSerial configuration");
        }
    }

    crsf.begin(crsfSerial);
}






void loop() {

    #if SEND_DATA == 1
        rtesSocket.process();

        if (currentSession != nullptr) {
            std::vector<uint8_t> bytes;

            bytes = floatsToBytes(mpu6050_data, 6);
            mpu6050_bytes.assign(bytes.begin(), bytes.end());

            bytes = floatsToBytes(pololu_data, 6);
            pololu_bytes.assign(bytes.begin(), bytes.end());

            bytes = floatsToBytes(combine_data, 13);
            combine_bytes.assign(bytes.begin(), bytes.end());

            if (mpu6050_bytes.size() != 0) currentSession->emitRaw(MPU, mpu6050_bytes);
            if (pololu_bytes.size() != 0) currentSession->emitRaw(POLOLU, pololu_bytes);
            if (combine_bytes.size() != 0) currentSession->emitRaw(COMBINE_IMU, combine_bytes);
        }
    #endif
    
    delay(1000);
    // crsf->update(); // Update RC data

    // size_t numFloats = sizeof(global_combined_imu_data);

    // Serial.print(numFloats);
    
    // uint8_t mpuBytes[6 * sizeof(float)];
    // Serial.print(mpuBytes);
    // uint8_t pololuBytes[6 * sizeof(float)];
    // uint8_t combinBytes[12 * sizeof(float)];

    // char*myChar=new char[6*4];

    // memcpy(myChar,global_mpu6050_imu_data,6*4);

    // Serial.print(myChar);


    // Serial.println(result);






    // floatsToBytes(global_mpu6050_imu_data, 6, mpuBytes);
    // floatsToBytes(global_polulo_imu_data, 6, pololuBytes);
    // floatsToBytes(global_combined_imu_data, 12, combinBytes);

    // Serial.print(mpuBytes);


    // // Print MPU6050 data in hexadecimal format
    // Serial.print("mpu data: ");
    // for (int i = 0; i < sizeof(mpuBytes); i++) {  // sizeof(mpuBytes) gives 24
    //     Serial.print(mpuBytes[i], HEX);  // Print each byte in hexadecimal format
    //     Serial.print(" ");
    // }
    // Serial.println();



    // if (currentSession != nullptr) {
    //     currentSession->emit(MPU, mpuBytes);
    //     currentSession->emit(POLOLU, pololuBytes);
    //     currentSession->emit(POLOLU, combinBytes);

    // } 
    // else {
    //     Serial.println("Session not connected!");
    // }

    // Serial.print(mpu);



// /// logical check///


// float naor = random(0,100);
// // Serial.println(naor);

// if (currentSession == nullptr)
//   return;




//  char b[50];
//  std::sprintf(b, "%.2f", naor);
//   currentSession->emit(POLOLU, b);


 }

/////////////////////////////////////////////////////////onReceiveRcChannels//////////////////////////////////////////////////////////



    

void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}