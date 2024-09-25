#include "CRSFforArduino.hpp"
#include <Wire.h>
#include "PololuIMU.hpp"
#include "MPU6050.h"
#include <TimerOne.h>
#include "UdpPublisher.hpp" // Include the UDP publisher class
#include "IMUCombiner.hpp"






////////////////Instances of IMUs/////////////////////////////
PololuIMU pololuImu;     // Instance of Pololu IMU
MPU6050 mpuimu(0x68);    // Instance of MPU6050 with I2C address 0x68
UdpPublisher udpPublisher;  // Instance of the UDP publisher class
IMUCombiner imuCombiner(&mpuimu, &pololuImu);









////////////// RC channel count and names////////////////////////

CRSFforArduino *crsf = nullptr;

int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
const char *rcChannelNames[] = {
    "A", "E", "T", "R", "Aux1", "Aux2", "Aux3", "Aux4",
    "Aux5", "Aux6", "Aux7", "Aux8", "Aux9", "Aux10", "Aux11", "Aux12"
};

int32_t global_axis_data[crsfProtocol::RC_CHANNEL_COUNT]; // Global variable to store axis data
const unsigned long updateInterval = 20;  // 20 milliseconds for 50 Hz updates
unsigned long lastUpdateTime = 0;

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);



///////////////////////////////////////read imu data ////////////////////////////////

void readIMUs() {
    // Read data from Pololu IMU
    float* pololuData = pololuImu.getSensorData();  // Get pointer to Pololu sensor data

    // Read data from MPU6050
    mpuimu.updateData();  // Update the MPU6050 sensor data
    float* mpuData = mpuimu.getData();  // Get pointer to MPU6050 sensor data

    float* combinedData = imuCombiner.combine_imu();
    imuCombiner.combine_imu();







    // // Print Pololu IMU sensor data
    // Serial.println("Pololu IMU Sensor Data:");
    // Serial.print("Accel X: "); Serial.println(pololuData[0]);
    // Serial.print("Accel Y: "); Serial.println(pololuData[1]);
    // Serial.print("Accel Z: "); Serial.println(pololuData[2]);
    // Serial.print("Gyro X: "); Serial.println(pololuData[3]);
    // Serial.print("Gyro Y: "); Serial.println(pololuData[4]);
    // Serial.print("Gyro Z: "); Serial.println(pololuData[5]);
    // Serial.print("Magnet X: "); Serial.println(pololuData[6]);
    // Serial.print("Magnet Y: "); Serial.println(pololuData[7]);
    // Serial.print("Magnet Z: "); Serial.println(pololuData[8]);
    // Serial.print("Pressure (hPa): "); Serial.println(pololuData[9]);

    // // Print MPU6050 sensor data
    // Serial.println("MPU6050 Sensor Data:");
    // Serial.print("Accel X: "); Serial.println(mpuData[0]);
    // Serial.print("Accel Y: "); Serial.println(mpuData[1]);
    // Serial.print("Accel Z: "); Serial.println(mpuData[2]);
    // Serial.print("Gyro X: "); Serial.println(mpuData[3]);
    // Serial.print("Gyro Y: "); Serial.println(mpuData[4]);
    // Serial.print("Gyro Z: "); Serial.println(mpuData[5]);

    // Serial.println();  // Empty line for better readability


      //   for (int i = 0; i < 13; i++) {
      // Serial.print("Combined Data ["); Serial.print(i); Serial.print("]: "); Serial.println(combinedData[i]);
    // }

}






/////////////////////////////////////////////////////////////////setup//////////////////////////////////////////////////////
void setup(){
    Serial.begin(115200);
    pololuImu.begin();

    mpuimu.init();

    Timer1.initialize(1000000);  // 1 second
    Timer1.attachInterrupt(readIMUs);  // Attach the readIMUs function to the timer




    // Initialize CRSF
    crsf = new CRSFforArduino();
    if (!crsf->begin()) {
        Serial.println("CRSF for Arduino initialization failed!");
        while (1) { delay(10); }
    }

    // Set callback for RC channel updates
    crsf->setRcChannelsCallback(onReceiveRcChannels);




}


void loop() {
    crsf->update(); // Update data at the RC

    // Set the data arrays in the UdpPublisher instance inside the loop
    udpPublisher.setRcArray(global_axis_data, rcChannelCount);
    udpPublisher.setMpu6050Array(mpuimu.getData(), 6);  // Get latest MPU6050 data (Accel X, Y, Z; Gyro X, Y, Z)
    udpPublisher.setPololuArray(pololuImu.getSensorData(), 10);  // Get latest Pololu IMU data

    // Print data using UdpPublisher class
    udpPublisher.printData();  // This function will print all the data

    delay(10);  // Add a small delay to avoid flooding the serial monitor
}

/////////////////////////////////////////////////////////onReceiveRcChannels//////////////////////////////////////////////////////////




void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels) {
    // if (rcChannels->failsafe == false) {
       
    //         // Process RC data
            for (int i = 0; i < rcChannelCount; i++) {
                int32_t channel_value = crsf->rcToUs(crsf->getChannel(i + 1));
                global_axis_data[i] = channel_value;  // Update global axis data
            }

            // Optional: Print RC data for debugging
            for (int i = 0; i < rcChannelCount; i++) {
                Serial.print(global_axis_data[i]);
                Serial.print("\t");
            }
            Serial.println();

          #if USE_MICRO_ROS == 1

            // Publish RC data through micro-ROS
            if (microROSAgent.isConnected()) {
                microROSAgent.publishRCData(global_axis_data, crsfProtocol::RC_CHANNEL_COUNT);
            }
            #endif

        }
    