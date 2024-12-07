#include <Arduino.h>
#include "Wire.h"
#include "Var_types.h"
#include "CompClass.h"
#include "src/Rtes/Rtes.h"


#include <iostream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <iomanip>
#include <cmath>

// IMU Sensor Libraries
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>

#include <AlfredoCRSF.h>


//IMU Data Conversion
// #define POL_GYRO_SENS 4.375/1000.0f // FS = 125
#define POL_GYRO_SENS 0.00875f // FS = 250
// #define POL_GYRO_SENS 0.0175f // FS = 500
#define POL_ACC_SENS 0.061/1000.0f // FS = 2g, 0.061 mg/LSB
// #define POL_ACC_SENS 0.122/1000.0f // FS = 4g, 0.122 mg/LSB
#define POL_MAG_SENS 1/6842.0f
// #define POL_MAG_SENS 1/2281.0f

//Unit Conversion
#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define deg2rad PI/180.0f

// UDP Communication:
#define UDP_BRIDGE 1 // 1 to make the bridge of the data 
#define SEND_DATA 1 // send data 
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define MPU_TICK_RATE 10000
#define MAG 'm'
#define POLOLU 'p'
#define euler 'e'
#define RC 'r'
#define RC_ros 'n'
#define Quaternion 'q'
#define Motor_basic 'b'


/*
------------------------------------------ Global Variables ------------------------------------------
*/

// Measurement struct
Measurement_t meas;
quat_t q_est;
vec3_t euler_angles;
// IMU Sensor Objects
LSM6 IMU;
LIS3MDL mag;
LPS baro;

CompFilter Pololu_filter(true);
AlfredoCRSF crsf;






float dt = 1/1100.0f;
float lastTime = 0;
float loopCount = 0;

/*
------------------------------------------ Prototypes ------------------------------------------
*/

// Function prototypes
void Update_Measurement();
void GyroMagCalibration();
void UDPSend2Py();
void convert_Measurment_to_byte();
void save_ch();
void calibrate_motor();
void writeMotorSpeed();

std::vector<uint8_t> floatsToBytes(float* pFloatArray, size_t size);
std::vector<uint8_t> intToBytes(int* pIntArray, size_t size);

/*
------------------------------------------ UDP Setup ------------------------------------------
*/
#if UDP_BRIDGE ==1

constexpr uint8_t IP_ADDRESS[4] = {192, 168,1, 199};
constexpr uint16_t PORT_NUMBER = 8888;
const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
Rtes rtesSocket(SOCKET_ADDRESS);
RtesSession* currentSession = nullptr;

std::vector<uint8_t> quaternion_bytes = {};
std::vector<uint8_t> euler_bytes = {};
std::vector<uint8_t> imu_bytes = {};
std::vector<uint8_t> mag_bytes = {};
std::vector<uint8_t> rc_bytes = {};

void onConnect(RtesSession &session){
  currentSession = &session;

}

#endif

float* imu_data = (float*)calloc(6, sizeof(float));
float* mag_data = (float*)calloc(3, sizeof(float));
float* euler_data = (float*)calloc(3, sizeof(float));
float* quaternion_data = (float*)calloc(4, sizeof(float));
int* rc_ch_data = (int*)calloc(16, sizeof(int));

std::vector<std::vector<float>> upsampleFusedIMUData(float* data, int numSamples, int upsampleFactor);



/*
------------------------------------------ Functions ------------------------------------------
*/
void setup() {
  Serial.begin(115200);
  // Seting pins 24 and 25 to be used as I2C
  Wire.begin();
  Wire.setClock(400000);


  /// start the connection 
  #if UDP_BRIDGE == 1 
  rtesSocket.begin(1);
  rtesSocket.onConnection(onConnect);
  #endif



  
  // Initialize IMU
  if (!IMU.init()){
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
    }
  if (!mag.init()){
    Serial.println("Failed to detect and initialize Magnetometer!");
    while (1);
    }
  // if (!baro.init()){
  //   Serial.println("Failed to detect and initialize Barometer!");
  //   while (1);
  //   }
  IMU.enableDefault(); // 1.66 kHz, 2g, 245 dps
  IMU.writeReg(LSM6::CTRL2_G, 0b10100000);  // 0b1010 for ODR 1.66 kHz, 0b0000 for 125 dps range
  IMU.writeReg(LSM6::CTRL1_XL, 0b10100000);  // 0b1010 for ODR 1.66 kHz, 0b0000 for 2g range
  // IMU.writeReg(LSM6::CTRL1_XL, 0b10110000); // Set ODR to 3.33 kHz, FS = ±2 g (if unchanged)
  // IMU.writeReg(LSM6::CTRL2_G, 0b10110000);  // Set ODR to 3.33 kHz, FS = 125 dps (if unchanged)

  mag.enableDefault();
  mag.writeReg(LIS3MDL::CTRL_REG1, 0b11100110); // 1 KHz, high performance mode
  mag.writeReg(LIS3MDL::CTRL_REG2,0x10); // +- 4 gauss
  // baro.enableDefault();


  // / activate crsf 
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  if (!crsfSerial) {
      while (1) {
      Serial.println("Invalid crsfSerial configuration");
      }
  }

  crsf.begin(crsfSerial);
  GyroMagCalibration();

}


void loop(){
    // loopCount++;

    // if (millis() - lastTime >= 1000) {
    //     // Print the loop rate
    //     Serial.print("Loop Rate: ");
    //     Serial.print(loopCount);
    //     Serial.println(" iterations/second");

    //     // Reset the count and last time
    //     loopCount = 0;
    //     lastTime = millis();
    // }
    // Update the measurement
    Update_Measurement();
    
    // Update the quaternion
    Pololu_filter.UpdateQ(&meas, dt);
        
    // // Get the Euler angles
    Pololu_filter.GetEulerRPYrad(&euler_angles, meas.initial_heading); 

    Pololu_filter.GetQuaternion(&q_est);
    crsf.update();

    UDPSend2Py();
    print_ch();
    q_print();



}


void Update_Measurement(){
    // Read IMU data
    IMU.read();
    mag.read();    
    meas.acc.x = IMU.a.x * POL_ACC_SENS;
    meas.acc.y = IMU.a.y * POL_ACC_SENS;

    meas.acc.z = IMU.a.z * POL_ACC_SENS;
    meas.gyro.x = IMU.g.x * POL_GYRO_SENS * deg2rad-meas.gyro_bias.x;
    meas.gyro.y = IMU.g.y * POL_GYRO_SENS * deg2rad-meas.gyro_bias.y;
    meas.gyro.z = IMU.g.z * POL_GYRO_SENS * deg2rad-meas.gyro_bias.z;
    meas.mag.x = mag.m.x * POL_MAG_SENS - meas.mag_bias.x;
    meas.mag.y = mag.m.y * POL_MAG_SENS - meas.mag_bias.y;
    meas.mag.z = mag.m.z * POL_MAG_SENS - meas.mag_bias.z;
    // meas.baro_data.pressure = baro.readPressureMillibars();
    // meas.baro_data.temperature = baro.readTemperatureC();
    // meas.baro_data.asl = baro.pressureToAltitudeMeters(meas.baro_data.pressure);
}


void GyroMagCalibration(){
    Serial.println("Starting Gyro calibration");
    int start_time = millis();
    int num_samples = 0;
    while (millis() - start_time < 10000){
      IMU.read();
      mag.read();
      float x = IMU.g.x * POL_GYRO_SENS * deg2rad;
      float y = IMU.g.y * POL_GYRO_SENS * deg2rad;
      float z = IMU.g.z * POL_GYRO_SENS * deg2rad;
      num_samples++;
      meas.gyro_bias.x += (x- meas.gyro_bias.x)/num_samples;
      meas.gyro_bias.y += (y- meas.gyro_bias.y)/num_samples;
      meas.gyro_bias.z += (z- meas.gyro_bias.z)/num_samples;

      meas.mag_bias.x += (mag.m.x * POL_MAG_SENS - meas.mag_bias.x)/num_samples;
      meas.mag_bias.y += (mag.m.y * POL_MAG_SENS - meas.mag_bias.y)/num_samples;
      meas.mag_bias.z += (mag.m.z * POL_MAG_SENS - meas.mag_bias.z)/num_samples;

      meas.acc_bias.x += (IMU.a.x * POL_ACC_SENS - meas.acc_bias.x)/num_samples;
      meas.acc_bias.y += (IMU.a.y * POL_ACC_SENS - meas.acc_bias.y)/num_samples;
      meas.acc_bias.z += (IMU.a.z * POL_ACC_SENS - meas.acc_bias.z)/num_samples;
    }
    meas.initial_mag.x = meas.mag_bias.x;
    meas.initial_mag.y = meas.mag_bias.y;
    meas.initial_mag.z = meas.mag_bias.z;
    meas.initial_heading = atan2f(meas.initial_mag.y, meas.initial_mag.x);

    // int start_time2 = millis();
    // int num_samples2 = 0;
    // int sum = 0;
    // while (millis() - start_time2 < 10000){
    //   IMU.read();
    //   float x = IMU.g.x * POL_GYRO_SENS * deg2rad;
    //   float y = IMU.g.y * POL_GYRO_SENS * deg2rad;
    //   float z = IMU.g.z * POL_GYRO_SENS * deg2rad;
    //   sum += (x-meas.gyro_bias.x)*(x-meas.gyro_bias.x) + (y-meas.gyro_bias.y)*(y-meas.gyro_bias.y) + (z-meas.gyro_bias.z)*(z-meas.gyro_bias.z);
    //   num_samples2++;
    // }
    // meas.gyro_drift = sqrtf(sum/num_samples2);

    // Serial.print("Gyro Drift: ");
    // Serial.println(meas.gyro_drift);
    Serial.println("Finished Gyro calibration");
    delay(2000);

}


/*
------------------------------------------ UDP Functions ------------------------------------------
*/

void UDPSend2Py(){
  convert_Measurment_to_byte();

    #if UDP_BRIDGE ==1 
    rtesSocket.process();
    #endif

    #if SEND_DATA == 1 && UDP_BRIDGE ==1
      if (currentSession != nullptr) {
        std::vector<uint8_t> bytes;

        // Send Pololu IMU data
        bytes = floatsToBytes(imu_data, 6);
        imu_bytes.assign(bytes.begin(), bytes.end());

        bytes = floatsToBytes(mag_data, 3);
        mag_bytes.assign(bytes.begin(), bytes.end());
        // // Send RC data
        bytes = intToBytes(rc_ch_data,16);
        rc_bytes.assign(bytes.begin(), bytes.end());

        // Send Euler angles data (roll, pitch, yaw)
        bytes = floatsToBytes(euler_data, 3);
        euler_bytes.assign(bytes.begin(), bytes.end());

        bytes = floatsToBytes(quaternion_data, 4);
        quaternion_bytes.assign(bytes.begin(), bytes.end());

        if (imu_bytes.size() != 0) currentSession->emitRaw(POLOLU, imu_bytes);
        if (mag_bytes.size() != 0) currentSession->emitRaw(MAG, mag_bytes);
        if (rc_bytes.size() != 0) currentSession->emitRaw(RC, rc_bytes);
        if (euler_bytes.size() != 0) currentSession->emitRaw(euler, euler_bytes);
        if (quaternion_bytes.size() != 0) currentSession->emitRaw(Quaternion, quaternion_bytes);

      }
    #endif
}

void convert_Measurment_to_byte(){
    imu_data[0] = meas.acc.x;
    imu_data[1] = meas.acc.y;
    imu_data[2] =  meas.acc.z;
    imu_data[3] = meas.gyro.x;
    imu_data[4] = meas.gyro.y;
    imu_data[5] = meas.gyro.z;
    mag_data[0] = meas.mag.x;
    mag_data[1] = meas.mag.y;
    mag_data[2] = meas.mag.z;
    quaternion_data[0] = q_est.w;
    quaternion_data[1] = q_est.x;
    quaternion_data[2] = q_est.y;
    quaternion_data[3] = q_est.z;
    euler_data[0] = euler_angles.x;
    euler_data[1] = euler_angles.y;
    euler_data[2] = euler_angles.z;
    save_ch();


}

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


void save_ch() {
    for (int i = 0; i < 17; i++) {
        rc_ch_data[i] = crsf.getChannel(i);  // Store each RC channel value into rc_ch_data
    }
}

// Function to start the motor 
void calibrate_motor(int pulseWidth,int motorPin) {
  // Send PWM pulse for continuous operation with a 20ms period
  for (int i = 0; i < 50; i++) {  // Repeat to maintain signal
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(motorPin, LOW);
    delayMicroseconds(20000 - pulseWidth);
  }
}

// Function that run on the main loop 
void writeMotorSpeed(int pulseWidth,int motorPin) {
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(motorPin, LOW);
    delayMicroseconds(20000 - pulseWidth);
  
  }




void print_ch() {
    for (int i = 0; i < 17; i++) {
        Serial.print(rc_ch_data[i]); Serial.print("/");   // Store each RC channel value into rc_ch_data
    }
    PP;
}



void imu_print(){
    for (size_t i = 0; i < 6; i++)
    {
      Serial.print(imu_data[i]); Serial.print("/");
    }
    PP ;
  
}


void q_print(){
    for (size_t i = 0; i < 4; i++)
    {
      Serial.print(quaternion_data[i]); Serial.print("/");
    }
    PP ;
}

void euler_print(){
    for (size_t i = 0; i < 3; i++)
    {
      Serial.print(euler_data[i]); Serial.print("/");
    }
    PP ;
}


















///////////////////////////old code ////////////////////////



// #include <Arduino.h>
// #include "Wire.h"
// #include "Var_types.h"
// #include "CompClass.h"
// #include "src/Rtes/Rtes.h"

// #include <iostream>
// #include <cstdint>
// #include <cstring>
// #include <vector>
// #include <iomanip>
// #include <cmath>

// // IMU Sensor Libraries
// #include <LSM6.h>
// #include <LIS3MDL.h>
// #include <LPS.h>

// #include <AlfredoCRSF.h>

// //IMU Data Conversion
// #define POL_GYRO_SENS 0.00875f // FS = 250
// // #define POL_GYRO_SENS 0.0175f // FS = 500
// #define POL_ACC_SENS 0.122/1000.0f // FS = 4g, 0.122 mg/LSB
// #define POL_MAG_SENS_4 1/6842.0f
// #define POL_MAG_SENS_12 1/2281.0f

// //Unit Conversion
// #define PI 3.14159265358979323846f
// #define rad2deg 180.0f/PI
// #define deg2rad PI/180.0f

// // UDP Communication:
// #define UDP_BRIDGE 1 // 1 to make the bridge of the data 
// #define SEND_DATA 1 // send data 
// #define PP Serial.println("")
// #define SPACE Serial.print(" /")
// #define crsfSerial Serial1  // Use Serial1 for the CRSF communication
// #define MPU_TICK_RATE 10000
// #define MAG 'm'
// #define POLOLU 'p'
// #define euler 'e'
// #define RC 'r'
// #define RC_ros 'n'
// #define Quaternion 'q'

// /*
// ------------------------------------------ Global Variables ------------------------------------------
// */

// // Measurement struct
// Measurement_t meas;
// quat_t q_est;
// vec3_t euler_angles;
// // IMU Sensor Objects
// LSM6 IMU;
// LIS3MDL mag;
// LPS baro;

// CompFilter Pololu_filter(true);
// AlfredoCRSF crsf;




// float dt = 0.001;
// float previous_time, current_time;

// /*
// ------------------------------------------ Prototypes ------------------------------------------
// */

// // Function prototypes
// void Update_Measurement();
// void GyroMagCalibration();
// void UDPSend2Py();
// void convert_Measurment_to_byte();
// void save_ch();
// std::vector<uint8_t> floatsToBytes(float* pFloatArray, size_t size);
// std::vector<uint8_t> intToBytes(int* pIntArray, size_t size);
// void getDesState();

// /*
// ------------------------------------------ UDP Setup ------------------------------------------
// */
// #if UDP_BRIDGE ==1

// constexpr uint8_t IP_ADDRESS[4] = {192, 168,1, 199};
// constexpr uint16_t PORT_NUMBER = 8888;
// const SocketAddress SOCKET_ADDRESS = SocketAddress(IP_ADDRESS, PORT_NUMBER);
// Rtes rtesSocket(SOCKET_ADDRESS);
// RtesSession* currentSession = nullptr;

// std::vector<uint8_t> quaternion_bytes = {};
// std::vector<uint8_t> euler_bytes = {};
// std::vector<uint8_t> imu_bytes = {};
// std::vector<uint8_t> mag_bytes = {};
// std::vector<uint8_t> rc_bytes = {};

// void onConnect(RtesSession &session){
//   currentSession = &session;

// }

// #endif

// float* imu_data = (float*)calloc(6, sizeof(float));
// float* mag_data = (float*)calloc(3, sizeof(float));
// float* euler_data = (float*)calloc(3, sizeof(float));
// float* quaternion_data = (float*)calloc(4, sizeof(float));
// int* rc_ch_data = (int*)calloc(16, sizeof(int));

// std::vector<std::vector<float>> upsampleFusedIMUData(float* data, int numSamples, int upsampleFactor);

// float thro_des = 0.0;
// float roll_des= 0.0;
// float pitch_des= 0.0;
// float yaw_des= 0.0;
// float roll_passthru = 0.0;
// float pitch_passthru = 0.0;
// float yaw_passthru = 0.0;
// float maxRoll = 0.3; //need to see wat did he do at his code 
// float maxPitch = 0.3;//need to see wat did he do at his code 
// float maxYaw = 0.3;//need to see wat did he do at his code 


// /*
// ------------------------------------------ Functions ------------------------------------------
// */
// void setup() {
//   Serial.begin(115200);
//   // Seting pins 24 and 25 to be used as I2C
//   Wire.begin();
//   Wire.setClock(400000);


//   /// start the connection 
//   #if UDP_BRIDGE == 1 
//   rtesSocket.begin(1);
//   rtesSocket.onConnection(onConnect);
//   #endif

  
//   // Initialize IMU
//   if (!IMU.init()){
//     Serial.println("Failed to detect and initialize IMU!");
//     while (1);
//     }
//   if (!mag.init()){
//     Serial.println("Failed to detect and initialize Magnetometer!");
//     while (1);
//     }

// /// activate crsf 
//   crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
//   if (!crsfSerial) {
//       while (1) {
//       Serial.println("Invalid crsfSerial configuration");
//       }
//   }

//   crsf.begin(crsfSerial);


//   IMU.enableDefault(); // 1.66 kHz, 2g, 245 dps
// // Set Gyroscope ODR to 1.66 kHz
//   IMU.writeReg(LSM6::CTRL2_G, 0b10010000);  // 0b1000 for ODR 1.66 kHz, 0b0001 for 250 dps range
//   // Set Accelerometer ODR to 1.66 kHz
//   IMU.writeReg(LSM6::CTRL1_XL, 0b10011000);  // 0b1000 for ODR 1.66 kHz, 0b0001 for 4g range

//   mag.enableDefault();
//   mag.writeReg(LIS3MDL::CTRL_REG1, 0b11100110); // 1 KHz, high performance mode
//   mag.writeReg(LIS3MDL::CTRL_REG2,0x10); // +- 12 gauss
//   // baro.enableDefault();
//   GyroMagCalibration();
//   previous_time = millis();

// }


// void loop(){
//     current_time = millis();
//     dt = (current_time - previous_time)/1000.0;
//     // Update the measurement
//     Update_Measurement();
//     // Update the quaternion
//     Pololu_filter.UpdateQ(&meas, dt);
//     // // Get the Euler angles
//     Pololu_filter.GetEulerRPYdeg(&euler_angles, meas.initial_heading); 
//     Pololu_filter.GetQuaternion(&q_est);
//     crsf.update();
//     UDPSend2Py();
//     getDesState();
//     previous_time = current_time;



// }


// void Update_Measurement(){
//     // Read IMU data
//     IMU.read();
//     mag.read();    
//     meas.acc.x = IMU.a.x * POL_ACC_SENS;
//     meas.acc.y = IMU.a.y * POL_ACC_SENS;
//     meas.acc.z = IMU.a.z * POL_ACC_SENS;
//     meas.gyro.x = IMU.g.x * POL_GYRO_SENS * deg2rad-meas.gyro_bias.x;
//     meas.gyro.y = IMU.g.y * POL_GYRO_SENS * deg2rad-meas.gyro_bias.y;
//     meas.gyro.z = IMU.g.z * POL_GYRO_SENS * deg2rad-meas.gyro_bias.z;
//     meas.mag.x = mag.m.x * POL_MAG_SENS_4;
//     meas.mag.y = mag.m.y * POL_MAG_SENS_4;
//     meas.mag.z = mag.m.z * POL_MAG_SENS_4;
//     // meas.baro_data.pressure = baro.readPressureMillibars();
//     // meas.baro_data.temperature = baro.readTemperatureC();
//     // meas.baro_data.asl = baro.pressureToAltitudeMeters(meas.baro_data.pressure);
// }


// void GyroMagCalibration(){
//     Serial.println("Starting Gyro calibration");
//     int start_time = millis();
//     int num_samples = 0;
//     while (millis() - start_time < 10000){
//       IMU.read();
//       mag.read();
//       float x = IMU.g.x * POL_GYRO_SENS * deg2rad;
//       float y = IMU.g.y * POL_GYRO_SENS * deg2rad;
//       float z = IMU.g.z * POL_GYRO_SENS * deg2rad;
//       num_samples++;
//       meas.gyro_bias.x += (x- meas.gyro_bias.x)/num_samples;
//       meas.gyro_bias.y += (y- meas.gyro_bias.y)/num_samples;
//       meas.gyro_bias.z += (z- meas.gyro_bias.z)/num_samples;

//       meas.mag_bias.x += (mag.m.x * POL_MAG_SENS_4 - meas.mag_bias.x)/num_samples;
//       meas.mag_bias.y += (mag.m.y * POL_MAG_SENS_4 - meas.mag_bias.y)/num_samples;
//       meas.mag_bias.z += (mag.m.z * POL_MAG_SENS_4 - meas.mag_bias.z)/num_samples;

//       meas.acc_bias.x += (IMU.a.x * POL_ACC_SENS - meas.acc_bias.x)/num_samples;
//       meas.acc_bias.y += (IMU.a.y * POL_ACC_SENS - meas.acc_bias.y)/num_samples;
//       meas.acc_bias.z += (IMU.a.z * POL_ACC_SENS - meas.acc_bias.z)/num_samples;
//     }
//     meas.initial_mag.x = meas.mag_bias.x;
//     meas.initial_mag.y = meas.mag_bias.y;
//     meas.initial_mag.z = meas.mag_bias.z;
//     meas.initial_heading = atan2f(meas.initial_mag.y, meas.initial_mag.x);


//     Serial.println("Finished Gyro calibration");
//     delay(2000);

// }


// /*
// ------------------------------------------ UDP Functions ------------------------------------------
// */

// void UDPSend2Py(){
//   convert_Measurment_to_byte();

//     #if UDP_BRIDGE ==1 
//     rtesSocket.process();
//     #endif

//     #if SEND_DATA == 1 && UDP_BRIDGE ==1
//       if (currentSession != nullptr) {
//         std::vector<uint8_t> bytes;

//         // Send Pololu IMU data
//         bytes = floatsToBytes(imu_data, 6);
//         imu_bytes.assign(bytes.begin(), bytes.end());

//         bytes = floatsToBytes(mag_data, 3);
//         mag_bytes.assign(bytes.begin(), bytes.end());
//         // // Send RC data
//         bytes = intToBytes(rc_ch_data,16);
//         rc_bytes.assign(bytes.begin(), bytes.end());

//         // Send Euler angles data (roll, pitch, yaw)
//         bytes = floatsToBytes(euler_data, 3);
//         euler_bytes.assign(bytes.begin(), bytes.end());

//         bytes = floatsToBytes(quaternion_data, 4);
//         quaternion_bytes.assign(bytes.begin(), bytes.end());

//         if (imu_bytes.size() != 0) currentSession->emitRaw(POLOLU, imu_bytes);
//         if (mag_bytes.size() != 0) currentSession->emitRaw(MAG, mag_bytes);
//         if (rc_bytes.size() != 0) currentSession->emitRaw(RC, rc_bytes);
//         if (euler_bytes.size() != 0) currentSession->emitRaw(euler, euler_bytes);
//         if (quaternion_bytes.size() != 0) currentSession->emitRaw(Quaternion, quaternion_bytes);

//       }
//     #endif
// }

// void convert_Measurment_to_byte(){
//     imu_data[0] = meas.acc.x;
//     imu_data[1] = meas.acc.y;
//     imu_data[2] =  meas.acc.z;
//     imu_data[3] = meas.gyro.x;
//     imu_data[4] = meas.gyro.y;
//     imu_data[5] = meas.gyro.z;
//     mag_data[0] = meas.mag.x;
//     mag_data[1] = meas.mag.y;
//     mag_data[2] = meas.mag.z;
//     quaternion_data[0] = q_est.w;
//     quaternion_data[1] = q_est.x;
//     quaternion_data[2] = q_est.y;
//     quaternion_data[3] = q_est.z;
//     euler_data[0] = euler_angles.x;
//     euler_data[1] = euler_angles.y;
//     euler_data[2] = euler_angles.z;
//     save_ch();


// }

// std::vector<uint8_t> floatsToBytes(float* pFloatArray, size_t size) {
//     uint8_t byteArray[size * sizeof(float)];
//     std::memcpy(byteArray, pFloatArray, size * sizeof(float));
//     std::vector<uint8_t> bytes;
//     for (size_t i = 0; i < sizeof(byteArray); i++)
//         bytes.push_back(byteArray[i]);

//     return bytes;
// }


// std::vector<uint8_t> intToBytes(int* pFloatArray, size_t size) {
//     uint8_t byteArray[size * sizeof(int)];
//     std::memcpy(byteArray, pFloatArray, size * sizeof(int));
//     std::vector<uint8_t> bytes;
//     for (size_t i = 0; i < sizeof(byteArray); i++)
//         bytes.push_back(byteArray[i]);

//     return bytes;
// }


// void save_ch() {
//     for (int i = 0; i < 17; i++) {
//         rc_ch_data[i] = crsf.getChannel(i);  // Store each RC channel value into rc_ch_data
//     }
// }


// void print_crsf_ch(){
//     for (size_t i = 0; i < 16; i++)
//     {
//        Serial.print(rc_ch_data[i]); Serial.print("/");
//     }
//     PP ;
    
// }

// void imu_print(){
//     for (size_t i = 0; i < 6; i++)
//     {
//       Serial.print(imu_data[i]); Serial.print("/");
//     }
//     PP ;
  
// }


// void q_print(){
//     for (size_t i = 0; i < 4; i++)
//     {
//       Serial.print(quaternion_data[i]); Serial.print("/");
//     }
//     PP ;
// }

// void euler_print(){
//     for (size_t i = 0; i < 3; i++)
//     {
//       Serial.print(euler_data[i]); Serial.print("/");
//     }
//     PP ;
// }



// //////// need to integrated this code ///////
// void getDesState() {



//   //DESCRIPTION: Normalizes desired control values to appropriate values
//   /*
//    * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
//    * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
//    * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
//    * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
//    * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
//    */
//   thro_des = (crsf.getChannel(3) - 1000.0)/1000.0; //Between 0 and 1
//   roll_des = (crsf.getChannel(1) - 1500.0)/500.0; //Between -1 and 1
//   pitch_des = (crsf.getChannel(2) - 1500.0)/500.0; //Between -1 and 1
//   yaw_des = (crsf.getChannel(4) - 1500.0)/500.0; //Between -1 and 1

//   roll_passthru = roll_des/2.0;
//   pitch_passthru = pitch_des/2.0;
//   yaw_passthru = yaw_des/2.0;
//   thro_des = constrain(thro_des, 0.0, 1.0);
//   roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll;
//   pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch;
//   yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw;
//   roll_passthru = constrain(roll_passthru, -0.5, 0.5);
//   pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
//   yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
//   Serial.print(thro_des);SPACE; Serial.print(roll_passthru);SPACE; Serial.print(pitch_passthru);SPACE; Serial.print(yaw_passthru); PP;



// }

  

  



// // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


