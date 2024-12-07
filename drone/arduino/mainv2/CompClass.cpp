#include <Arduino.h>
#include "CompClass.h"
#include "Var_types.h"


//static float baseZaccel = 1.0; // Base acceleration in the Z direction

void CompFilter::UpdateQ(Measurement_t* meas, float dt){
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy, _8qx, _8qy, qwqw, qxqx, qyqy, qzqz;
    float _2qwmx, _2qwmy, _2qwmz, _2qxmx, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3, _2q0q1, _2q0q3;
    float hx, hy, _2bx, _2bz, _4bx, _4bz;

    // Initial Filtering - Not a must nut it helps.
    InitialFiltering(meas);


    // Model based time propagation
    qDot1 = 0.5f * (-q.x * meas->gyro.x - q.y * meas->gyro.y - q.z * meas->gyro.z);
    qDot2 = 0.5f * (q.w * meas->gyro.x + q.y * meas->gyro.z - q.z * meas->gyro.y);
    qDot3 = 0.5f * (q.w * meas->gyro.y - q.x * meas->gyro.z + q.z * meas->gyro.x);
    qDot4 = 0.5f * (q.w * meas->gyro.z + q.x * meas->gyro.y - q.y * meas->gyro.x);
    
    float BETA = calculateDynamicBeta(*meas);
    //float BETA = DEFAULT_BETA;

    if(!(meas->acc.x == 0.0 && meas->acc.y ==0.0 && meas->acc.z ==0.0)) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(meas->acc.x * meas->acc.x + meas->acc.y * meas->acc.y + meas->acc.z * meas->acc.z);
        meas->acc.x *= recipNorm;
        meas->acc.y *= recipNorm;
        meas->acc.z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2qw = 2.0f * q.w;
        _2qx = 2.0f * q.x;
        _2qy = 2.0f * q.y;
        _2qz = 2.0f * q.z;
        _4qw = 4.0f * q.w;
        _4qx = 4.0f * q.x;
        _4qy = 4.0f * q.y;
        _8qx = 8.0f * q.x;
        _8qy = 8.0f * q.y;
        qwqw = q.w * q.w;
        qxqx = q.x * q.x;
        qyqy = q.y * q.y;
        qzqz = q.z * q.z;

        // Gradient decent algorithm corrective step
        s0 = _4qw * qyqy + _2qy * meas->acc.x + _4qw * qxqx - _2qx * meas->acc.y;
        s1 = _4qx * qzqz - _2qz * meas->acc.x + 4.0f * qwqw * q.x - _2qw * meas->acc.y - _4qx + _8qx * qxqx + _8qx * qyqy + _4qx * meas->acc.z;
        s2 = 4.0f * qwqw * q.y + _2qw * meas->acc.x + _4qy * qzqz - _2qz * meas->acc.y - _4qy + _8qy * qxqx + _8qy * qyqy + _4qy * meas->acc.z;
        s3 = 4.0f * qxqx * q.z - _2qx * meas->acc.x + 4.0f * qyqy * q.z - _2qy * meas->acc.y;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;


        // Apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;


    }
    if(USE_MAG &&( meas->mag.x != 0.0 && meas->mag.y != 0.0 && meas->mag.z != 0.0) && (gyroNorm > HIGH_MOTION)){
    //if(USE_MAG &&( meas->mag.x != 0.0 && meas->mag.y != 0.0 && meas->mag.z != 0.0)){

        // Normalise magnetometer measurement
        recipNorm = invSqrt(meas->mag.x * meas->mag.x + meas->mag.y * meas->mag.y + meas->mag.z * meas->mag.z);
        meas->mag.x *= recipNorm;
        meas->mag.y *= recipNorm;
        meas->mag.z *= recipNorm;

        _2qwmx = 2.0f * q.w * meas->mag.x;
        _2qwmy = 2.0f * q.w * meas->mag.y;
        _2qwmz = 2.0f * q.w * meas->mag.z;
        _2qxmx = 2.0f * q.x * meas->mag.x;
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _2q0q2 = 2.0f * q.w * q.y;
        _2q0q1 = 2.0f * q.w * q.x;
        _2q0q3 = 2.0f * q.w * q.z;
        _2q2q3 = 2.0f * q.y * q.z;
        q0q0 = q.w * q.w;
        q0q1 = q.w * q.x;
        q0q2 = q.w * q.y;
        q0q3 = q.w * q.z;
        q1q1 = q.x * q.x;
        q1q2 = q.x * q.y;
        q1q3 = q.x * q.z;
        q2q2 = q.y * q.y;
        q2q3 = q.y * q.z;
        q3q3 = q.z * q.z;

        // Reference direction of Earth's magnetic field
        hx = meas->mag.x * q0q0 - _2qwmy * q.z + _2qwmz * q.y + meas->mag.x * q1q1 + _2qxmx * q.y + meas->mag.x * q2q2 - meas->mag.x * q3q3;
        hy = _2qwmx * q.z + meas->mag.y * q0q0 - _2qwmz * q.x + _2q0 * meas->mag.y * q1q1 - meas->mag.y * q2q2 + meas->mag.y * q3q3 + _2qxmx * q.z;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2qwmx * q.y + _2q0 * meas->mag.z * q1q1 + _2q0 * meas->mag.z * q2q2 + meas->mag.z * q3q3 + _2qxmx * q.y - meas->mag.z * q0q0;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = - _2bz * q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - meas->mag.z) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - meas->mag.x);
        s1 = - 4.0f * q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - meas->mag.z) + _2bz * q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - meas->mag.z) + (_2bx * q.y + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - meas->mag.x);
        s2 = - 4.0f * q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - meas->mag.z) + (-_2bx * q.y + _2bz * q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - meas->mag.z) + (_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - meas->mag.x);
        s3 = (-_4bx * q.y * (2.0f * q1q2 - _2q0q3 - meas->mag.z) + _4bz * q.z * (2.0f * q1q3 - _2q0q2 - meas->mag.x) - _4bx * q.x * (2.0f * q2q3 - _2q0q1 - meas->mag.y)) + (-_4bx * q.z + _4bz * q.x) * (_4bx * (0.5f - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - meas->mag.x);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;


        // Apply feedback step
        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;

    }


    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * dt;
    q.x += qDot2 * dt;
    q.y += qDot3 * dt;
    q.z += qDot4 * dt;

    // Normalise quaternion in order to get unit length quaternion
    recipNorm = invSqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    estimatedGravityDir(&gravX, &gravY, &gravZ);

}



// Returning the estimated quaternion
void CompFilter::GetQuaternion(quat_t* q_){
    q_->x = q.x;
    q_->y = q.y;
    q_->z = q.z;
    q_->w = q.w;

}

void CompFilter::GetEulerRPYrad(vec3_t* rpy, float initial_heading){
    float gx = gravX;
    float gy = gravY;
    float gz = gravZ;

    if (gx > 1) gx = 1;
    if (gx < -1) gx = -1;

    // Currently returend in radians, can be converted to degrees by multiplying by rad2deg
    rpy->z = atan2f(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    rpy->y = asinf(gx);
    // rpy->y = atan2f(2 * (q.w * q.y - q.x * q.z), 1 - 2 * (q.y * q.y + q.z * q.z))
    rpy->x = atan2f(gy, gz);
}

void CompFilter::GetEulerRPYdeg(vec3_t* rpy, float initial_heading){
    float gx = gravX;
    float gy = gravY;
    float gz = gravZ;
    if (gx > 1) gx = 1;
    if (gx < -1) gx = -1;

    rpy->z = atan2f(2*(q.w*q.z + q.x*q.y), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) * rad2deg;
    //     if (rpy->z < 0) {
    //     rpy->z += 360.0f;
    // }
    rpy->y = asinf(gx) * rad2deg;
    // rpy->y = atan2f(2 * (q.w * q.y - q.x * q.z), 1 - 2 * (q.y * q.y + q.z * q.z)) * rad2deg;
    rpy->x = atan2f(gy, gz) * rad2deg;
}

//---------------------------------------------------------------------------------------------------
// Inverse square root function:
float CompFilter::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float CompFilter::GetAccZ(float ax, float ay, float az) {
    return (ax*gravX + ay*gravY + az*gravZ);
}

void CompFilter::estimatedGravityDir(float* gx, float* gy, float*gz){
    *gx = 2*(q.x*q.z - q.y*q.w);
    *gy = 2*(q.y*q.z + q.x*q.w);
    *gz = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
}
float CompFilter::calculateDynamicBeta(Measurement_t meas) {
    // Compute the norm (magnitude) of the gyroscope vector
    gyroNorm = sqrtf(meas.gyro.x * meas.gyro.x + meas.gyro.y * meas.gyro.y + meas.gyro.z * meas.gyro.z);

    // Adapt Beta based on gyroscope norm
    if (gyroNorm < LOW_MOTION) {
        // System is likely stable or slow-moving, increase Beta for more correction
        // Serial.println("Low Motion");
        return HIGH_BETA;
    } else if (gyroNorm > HIGH_MOTION) {
        // System is moving fast, reduce Beta to rely more on gyroscope
        // Serial.println("High Motion");
        return LOW_BETA;
    } else {
        // Default case, normal correction
        // Serial.println("Default Motion");
        return DEFAULT_BETA;
    }
}


void CompFilter::InitialFiltering(Measurement_t* meas){
     // Apply Low-pass Filter to Accel
    accFiltered.x = (1 - ALPHA_LPF) * accFiltered.x + ALPHA_LPF * meas->acc.x;
    accFiltered.y = (1 - ALPHA_LPF) * accFiltered.y + ALPHA_LPF * meas->acc.y;
    accFiltered.z = (1 - ALPHA_LPF) * accFiltered.z + ALPHA_LPF * meas->acc.z;
    meas->acc.x = accFiltered.x;
    meas->acc.y = accFiltered.y;
    meas->acc.z = accFiltered.z;

    // Apply High-pass Filter to Gyro
    static vec3_t gyroPrev = {0.0, 0.0, 0.0};
    gyroFiltered.x = ALPHA_HPF * (gyroFiltered.x + meas->gyro.x - gyroPrev.x);
    gyroFiltered.y = ALPHA_HPF * (gyroFiltered.y + meas->gyro.y - gyroPrev.y);
    gyroFiltered.z = ALPHA_HPF * (gyroFiltered.z + meas->gyro.z - gyroPrev.z);
    meas->gyro.x = gyroFiltered.x;
    meas->gyro.y = gyroFiltered.y;
    meas->gyro.z = gyroFiltered.z;
    gyroPrev.x = meas->gyro.x;
    gyroPrev.y = meas->gyro.y;
    gyroPrev.z = meas->gyro.z;

    if (USE_MAG){
        // Apply Low-pass Filter to Mag
        magFiltered.x = (1 - ALPHA_LPF) * magFiltered.x + ALPHA_LPF * meas->mag.x;
        magFiltered.y = (1 - ALPHA_LPF) * magFiltered.y + ALPHA_LPF * meas->mag.y;
        magFiltered.z = (1 - ALPHA_LPF) * magFiltered.z + ALPHA_LPF * meas->mag.z;
        meas->mag.x = magFiltered.x;
        meas->mag.y = magFiltered.y;
        meas->mag.z = magFiltered.z;
    }
}
