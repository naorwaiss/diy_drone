#include "src/MPU6050/MPU6050.h"
#include <Wire.h>
#include <SPI.h>
#include "src/Rtes/Rtes.h"
#include <cstring> // Add this line to include the C string library


MPU6050 mpu6050;

// IMU Scale Constants
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16


#define UDP_BRIDGE 1
#define SEND_DATA 1
#define PP Serial.println("")
#define crsfSerial Serial1  // Use Serial1 for the CRSF communication
#define POLOLU 'p'
#define EUILER 'e'
#define RC 'r'
#define RC_ros 'n'
#define Qurtirion 'q'




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


// Magnetometer calibration parameters
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

// IMU calibration parameters
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY = 0.0;
float GyroErrorZ = 0.0;





// Allocate memory for data arrays
float* mpu6050_data = (float*)calloc(6, sizeof(float)); //a.x/a.y/a.z/g.x/g.y/g.z
float* mpu6050_data_prev = (float*)calloc(6, sizeof(float)); //a.x/a.y/a.z/g.x/g.y/g.z
float* euiler = (float*)calloc(3, sizeof(float)); 
float* q_data = (float*)calloc(4, sizeof(float));
int* rc_ch_data = (int*)calloc(16, sizeof(int));





// IMU Variables
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU_prev, pitch_IMU_prev;



// Timing variables
unsigned long current_time, prev_time, print_counter;
float dt = 0;

// Filter constants
float B_accel = 0.05; // Example values, you should adjust these as necessary
float B_gyro = 0.05;
float B_mag = 0.05;
float B_madgwick = 0.1; // This is a tuning parameter for the Madgwick algorithm


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





void setup() {
    Serial.begin(500000); // USB serial
    delay(500);
    IMUinit();

    #if UDP_BRIDGE == 1
        rtesSocket.begin(1);
        rtesSocket.onConnection(onConnect);
    #endif

}





void loop() {
  // Keep track of elapsed time
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;
  getIMUdata(); // Retrieve gyro, accelerometer, and magnetometer data from the IMU
  Madgwick6DOF(mpu6050_data[3], -mpu6050_data[4], -mpu6050_data[5], -mpu6050_data[0], mpu6050_data[1], mpu6050_data[2], dt); // Update roll_IMU, pitch_IMU, and yaw_IMU

    #if UDP_BRIDGE == 1
    rtesSocket.process();
    #endif

    #if SEND_DATA == 1 && UDP_BRIDGE ==1

    if (currentSession != nullptr) {
        std::vector<uint8_t> bytes;

        bytes = floatsToBytes(mpu6050_data, 6);
        mpu6050_bytes.assign(bytes.begin(), bytes.end());


        bytes = floatsToBytes(q_data, 4);
        quterion_bytes.assign(bytes.begin(), bytes.end());

        bytes = intToBytes(rc_ch_data,16);
        rc_bytes.assign(bytes.begin(), bytes.end());


        if (mpu6050_bytes.size() != 0) currentSession->emitRaw(POLOLU, mpu6050_bytes);
        if (mpu6050_bytes.size() != 0) currentSession->emitRaw(Qurtirion, quterion_bytes);

        if (rc_bytes.size() != 0) currentSession->emitRaw(RC, rc_bytes);

    }
    #endif


    printQuaterion();
  // Simulate 2000Hz loop
  loopRate(2000);
}






// Placeholder function for loopRate
void loopRate(unsigned long rate) {
  delayMicroseconds(1000000 / rate);
}

// Placeholder for Madgwick inverse square root function (needed for Madgwick filter)
float invSqrt(float x) {
  return 1.0f / sqrt(x);
}

// Existing IMU initialization
void IMUinit() {
    Wire.begin();
    Wire.setClock(1000000); // Increase I2C speed
    mpu6050.initialize();
    
    if (!mpu6050.testConnection()) {
        Serial.println("MPU6050 initialization unsuccessful");
        while(1) {} // Infinite loop if initialization fails
    }

    // Set full-scale gyro and accelerometer range
    mpu6050.setFullScaleGyroRange(GYRO_FS_SEL_1000); // You can change this
    mpu6050.setFullScaleAccelRange(ACCEL_FS_SEL_8); // You can change this
}

// Existing function to get IMU data
void getIMUdata() {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

  // Read the IMU values
  mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

  // Convert raw data to g's and deg/sec, and correct for error
  mpu6050_data[0] = (AcX / 16384.0) - AccErrorX; // Adjust scale factor as needed
  mpu6050_data[1] = (AcY / 16384.0) - AccErrorY;
  mpu6050_data[2] = (AcZ / 16384.0) - AccErrorZ;

  mpu6050_data[3] = (GyX / 131.0) - GyroErrorX;
  mpu6050_data[4] = (GyY / 131.0) - GyroErrorY;
  mpu6050_data[5] = (GyZ / 131.0) - GyroErrorZ;
}

// Madgwick filter for 6DOF IMU data (gyro and accelerometer only)
// Madgwick filter for 6DOF IMU data (gyro and accelerometer only)
// Madgwick filter for 6DOF IMU data (gyro and accelerometer only)
// Madgwick filter for 6DOF IMU data (gyro and accelerometer only)
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f; // Convert to radians
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q_data[1] * gx - q_data[2] * gy - q_data[3] * gz);
    qDot2 = 0.5f * (q_data[0] * gx + q_data[2] * gz - q_data[3] * gy);
    qDot3 = 0.5f * (q_data[0] * gy - q_data[1] * gz + q_data[3] * gx);
    qDot4 = 0.5f * (q_data[0] * gz + q_data[1] * gy - q_data[2] * gx);

    // Check if accelerometer input is valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to reduce computation
        _2q0 = 2.0f * q_data[0];
        _2q1 = 2.0f * q_data[1];
        _2q2 = 2.0f * q_data[2];
        _2q3 = 2.0f * q_data[3];
        _4q0 = 4.0f * q_data[0];
        _4q1 = 4.0f * q_data[1];
        _4q2 = 4.0f * q_data[2];
        _8q1 = 8.0f * q_data[1];
        _8q2 = 8.0f * q_data[2];
        q0q0 = q_data[0] * q_data[0];
        q1q1 = q_data[1] * q_data[1];
        q2q2 = q_data[2] * q_data[2];
        q3q3 = q_data[3] * q_data[3];

        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q_data[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q_data[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q_data[3] - _2q1 * ax + 4.0f * q2q2 * q_data[3] - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalize step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    } else {
        Serial.println("Invalid accelerometer input detected.");
    }

    // **Store Previous Quaternion Values (before updating them)**
    float prev_q0 = q_data[0];
    float prev_q1 = q_data[1];
    float prev_q2 = q_data[2];
    float prev_q3 = q_data[3];

    // **Update Quaternion Values** by integrating rate of change of quaternion
    q_data[0] += qDot1 * invSampleFreq;
    q_data[1] += qDot2 * invSampleFreq;
    q_data[2] += qDot3 * invSampleFreq;
    q_data[3] += qDot4 * invSampleFreq;

    // Normalize quaternion
    recipNorm = invSqrt(q_data[0] * q_data[0] + q_data[1] * q_data[1] + q_data[2] * q_data[2] + q_data[3] * q_data[3]);
    if (recipNorm == 0) {
        Serial.println("Quaternion normalization failed: Division by zero.");
        return;
    }
    q_data[0] *= recipNorm;
    q_data[1] *= recipNorm;
    q_data[2] *= recipNorm;
    q_data[3] *= recipNorm;

    // Debug: Print quaternion values
    Serial.print("Prev Q0: ");
    Serial.print(prev_q0);
    Serial.print(", Prev Q1: ");
    Serial.print(prev_q1);
    Serial.print(", Prev Q2: ");
    Serial.print(prev_q2);
    Serial.print(", Prev Q3: ");
    Serial.println(prev_q3);

    Serial.print("New Q0: ");
    Serial.print(q_data[0]);
    Serial.print(", New Q1: ");
    Serial.print(q_data[1]);
    Serial.print(", New Q2: ");
    Serial.print(q_data[2]);
    Serial.print(", New Q3: ");
    Serial.println(q_data[3]);

    // Compute angles
    euiler[0] = atan2(q_data[0] * q_data[1] + q_data[2] * q_data[3], 0.5f - q_data[1] * q_data[1] - q_data[2] * q_data[2]) * 57.29577951f; // Roll in degrees
    euiler[1] = -asin(constrain(-2.0f * (q_data[1] * q_data[3] - q_data[0] * q_data[2]), -1.0f, 1.0f)) * 57.29577951f; // Pitch in degrees
    euiler[2] = atan2(q_data[1] * q_data[2] + q_data[0] * q_data[3], 0.5f - q_data[2] * q_data[2] - q_data[3] * q_data[3]) * 57.29577951f; // Yaw in degrees
}







////print statuse /////


void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(mpu6050_data[3]);
    Serial.print(F(" GyroY:"));
    Serial.print(mpu6050_data[4]);
    Serial.print(F(" GyroZ:"));
    Serial.println(mpu6050_data[5]);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(mpu6050_data[0]);
    Serial.print(F(" AccY:"));
    Serial.print(mpu6050_data[1]);
    Serial.print(F(" AccZ:"));
    Serial.println(mpu6050_data[2]);
  }
}


void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(euiler[0]);
    Serial.print(F(" pitch:"));
    Serial.print(euiler[1]);
    Serial.print(F(" yaw:"));
    Serial.println(euiler[2]);
  }
}

void printQuaterion() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Q0:"));
    Serial.print(q_data[0]);
    Serial.print(F(" Q1:"));
    Serial.print(q_data[1]);
    Serial.print(F(" Q2:"));
    Serial.println(q_data[2]);
    Serial.print(F(" Q3:"));
    Serial.println(q_data[3]);
  }
}



void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}
