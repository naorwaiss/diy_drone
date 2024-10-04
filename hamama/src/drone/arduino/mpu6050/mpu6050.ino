#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "src/Rtes/Rtes.h"
#include <stdio.h>
#include <imuFilter.h>

#include <iostream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <cmath>

#include <basicMPU6050.h>       // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <imuFilter.h>


// MPU6050 I2C Address
#define MPU6050_ADDRESS 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define PWR_MGMT_1 0x6B

// Register addresses for accelerometer and gyroscope data
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43

// Global variables
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define UDP_BRIDGE 1
#define SEND_DATA 1
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define MPU_TICK_RATE 10000
#define POLOLU 'p'
#define EUILER 'e'
#define RC 'r'
#define RC_ros 'n'
#define Qurtirion 'q'


// Gyro settings:
#define LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define ADDRESS_A0  LOW            // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
#define GAIN 0.5   
#define SD_ACCEL  0.2                   
                             
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 0;          // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0;          // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 0;

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;


#if UDP_BRIDGE == 1

constexpr uint8_t IP_ADDRESS[4] = {192, 168, 1, 199};
constexpr uint16_t PORT_NUMBER = 8888;
const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
Rtes rtesSocket(SOCKET_ADDRESS);
RtesSession* currentSession = nullptr;

std::vector<uint8_t> quterion_bytes = {};
std::vector<uint8_t> euiler_bytes = {};
std::vector<uint8_t> mpu6050_bytes = {};
std::vector<uint8_t> rc_bytes = {};

void onConnect(RtesSession &session) {
  currentSession = &session;
}

#endif

// Allocate memory for data arrays
float* mpu6050_data = (float*)calloc(6, sizeof(float));
float* euiler = (float*)calloc(3, sizeof(float));
float* quartirion_data = (float*)calloc(4, sizeof(float));
int* rc_ch_data = (int*)calloc(16, sizeof(int));

// IMU object
imuFilter fusion;

AlfredoCRSF crsf;

// Convert to bytes for transmission
std::vector<uint8_t> floatsToBytes(float* pFloatArray, size_t size) {
    uint8_t byteArray[size * sizeof(float)];
    std::memcpy(byteArray, pFloatArray, size * sizeof(float));
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < sizeof(byteArray); i++)
        bytes.push_back(byteArray[i]);

    return bytes;
}

std::vector<uint8_t> intToBytes(int* pIntArray, size_t size) {
    uint8_t byteArray[size * sizeof(int)];
    std::memcpy(byteArray, pIntArray, size * sizeof(int));
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < sizeof(byteArray); i++)
        bytes.push_back(byteArray[i]);

    return bytes;
}


/////////////////////////////////////////////////////////////////setup//////////////////////////////////////////////////////
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Wire.begin();
    Wire.setClock(4000000); // Increase I2C speed
    




    imu.setup();
    imu.setBias();

    fusion.setup( imu.ax(), imu.ay(), imu.az() );     

    #if UDP_BRIDGE == 1
    rtesSocket.begin(1);
    rtesSocket.onConnection(onConnect);
    #endif

    // Activate CRSF
    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
    if (!crsfSerial) {
        while (1) {
            Serial.println("Invalid CRSF serial configuration");
        }
    }
    crsf.begin(crsfSerial);
}


unsigned long previousMicros = 0;  // Stores the time of the previous loop iteration
float frequency = 0.0; 

/////////////////////////////////////////////////////////////////loop//////////////////////////////////////////////////////
void loop() {


    unsigned long currentMicros = micros();  // Get the current time in microseconds
    unsigned long iterationTime = currentMicros - previousMicros;  // Time difference between iterations
    previousMicros = currentMicros;  // Update the previous time

    if (iterationTime > 0) {
        frequency = 1000000.0 / iterationTime;  // Convert iteration time to frequency in Hz
    }

    Serial.println(frequency);

    mpu6050_data[0] = imu.gx();
    mpu6050_data[1] = imu.gy();
    mpu6050_data[2] = imu.gz();
    mpu6050_data[3] = imu.ax();
    mpu6050_data[4] = imu.ay();
    mpu6050_data[5] = imu.az();

    
    // // Print accelerometer and gyroscope data
    // Serial.print("Accel X: "); Serial.print(mpu6050_data[0]); PP;
    // Serial.print("Accel Y: "); Serial.print(mpu6050_data[1]); PP;
    // Serial.print("Accel Z: "); Serial.print(mpu6050_data[2]); PP;
    // Serial.print("Gyro X: "); Serial.print(mpu6050_data[3]); PP;
    // Serial.print("Gyro Y: "); Serial.print(mpu6050_data[4]); PP;
    // Serial.print("Gyro Z: "); Serial.println(mpu6050_data[5]);





    fusion.update( mpu6050_data[0], mpu6050_data[1], mpu6050_data[2], mpu6050_data[3], mpu6050_data[4], mpu6050_data[5], GAIN, SD_ACCEL );  

//     // Update Madgwick filter with IMU data
//     updateMadgwick(mpu6050_data[0], mpu6050_data[1], mpu6050_data[2], mpu6050_data[3], mpu6050_data[4], mpu6050_data[5]);

    // Calculate Euler angles from the Madgwick filter
    euiler[0] = fusion.roll();
    euiler[1] = fusion.pitch(); 
    euiler[2] = fusion.yaw() ;

    // Serial.print(euiler[0]); PP;
    // Serial.print(euiler[1]); PP;
    // Serial.println(euiler[2]); PP;
    
    eulerToQuaternion(euiler[0],euiler[1],euiler[2]);

    // Serial.print(quartirion_data[0]); PP;
    // Serial.print(quartirion_data[1]); PP;
    // Serial.print(quartirion_data[2]); PP;
    // Serial.println(quartirion_data[3]); PP;


    // Continue with your UDP, CRSF, and quaternion logic
    crsf.update();
    save_ch();

    #if UDP_BRIDGE == 1
    rtesSocket.process();
    #endif

    #if SEND_DATA == 1 && UDP_BRIDGE ==1

    if (currentSession != nullptr) {
        std::vector<uint8_t> bytes;

        bytes = floatsToBytes(mpu6050_data, 6);
        mpu6050_bytes.assign(bytes.begin(), bytes.end());


        bytes = floatsToBytes(quartirion_data, 4);
        quterion_bytes.assign(bytes.begin(), bytes.end());

        bytes = intToBytes(rc_ch_data,16);
        rc_bytes.assign(bytes.begin(), bytes.end());


        if (mpu6050_bytes.size() != 0) currentSession->emitRaw(POLOLU, mpu6050_bytes);
        if (mpu6050_bytes.size() != 0) currentSession->emitRaw(Qurtirion, quterion_bytes);

        if (rc_bytes.size() != 0) currentSession->emitRaw(RC, rc_bytes);

    }
    #endif






// delay(1);







}

void save_ch() {
    for (int i = 0; i < 16; i++) {
        rc_ch_data[i] = crsf.getChannel(i);  // Store each RC channel value into rc_ch_data
    }
}

void eulerToQuaternion(float roll, float pitch, float yaw) {
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
