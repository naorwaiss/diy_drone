/*
Written by: Amit Gedj
Date: 15.10.24

Description: This file contains the definition of the different types used in the project.
              It also contains the definition of the different rates used in the project.
              The RATE_DO_EXECUTE macro is used to execute a certain code at a certain rate.
              The different rates are defined in the RATE_XXX_HZ macros.
              The different types are defined in the different structs.
              The different structs are:
                  - vec3_t: a 3D vector
                  - quat_t: a quaternion
                  - baro_t: a barometer measurement
                  - Measurement_t: a struct containing the different measurements
                  - flowMeasurement_t: a struct containing the flow measurements
                  - tofMeasurement_t: a struct containing the TOF measurements
                  - heightMeasurement_t: a struct containing the height measurements
                  - attitude_t: a struct containing the attitude angles
                  - state_t: a struct containing the state of the drone
                  - StabStep_t: a type used to count the number of stabilization steps
*/



#ifndef VAR_TYPES_H
#define VAR_TYPES_H

#include <Arduino.h>

// Frequencies to be used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ
#define RATE_HL_COMMANDER RATE_100_HZ
#define RATE_SUPERVISOR RATE_25_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

typedef struct{
    float x;
    float y;
    float z;
} vec3_t;


typedef struct{
    float x;
    float y;
    float z;
    float w;
} quat_t;

typedef struct baro_s {
  float pressure;           // mbar
  float temperature;        // degree Celcius
  float asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct{
    vec3_t acc;
    vec3_t acc_bias = {0.0, 0.0, 0.0};
    vec3_t gyro;
    vec3_t gyro_bias = {0.0, 0.0, 0.0};
    float gyro_drift = {0.0};
    vec3_t mag;
    vec3_t mag_bias = {0.0, 0.0, 0.0};
    vec3_t initial_mag = {0.0, 0.0, 0.0};
    float initial_heading = 0.0;
    baro_t baro_data;
}Measurement_t;

typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  // Accumulated pixel count x
      float dpixely;  // Accumulated pixel count y
    };
    float dpixel[2];  // Accumulated pixel count
  };
  float stdDevX;      // Measurement standard deviation
  float stdDevY;      // Measurement standard deviation
  float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;

/** LiDAR TOF measurement **/
typedef struct tofMeasurement_s {
  uint32_t timestamp;
  float distance;
  float stdDev;
} tofMeasurement_t;

/** Absolute height measurement **/
typedef struct heightMeasurement_s {
  uint32_t timestamp;
  float height;
  float stdDev;
} heightMeasurement_t;

typedef struct attitude_s{
    float roll;
    float pitch;
    float yaw;
}attitude_t;

typedef struct state_s{
    attitude_t attitude_angles;
    quat_t attitudeQuaternion;
    float height;
    vec3_t velocity;
    vec3_t position;
    vec3_t acceleration;
}state_t;

typedef uint32_t StabStep_t;


#endif