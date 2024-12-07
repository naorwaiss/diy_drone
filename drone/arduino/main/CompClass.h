#ifndef COMPCLASS_H
#define COMPCLASS_H

#include <Arduino.h>
#include "Var_types.h"

#define PI 3.14159265358979323846f
#define rad2deg 180.0f/PI
#define LOW_MOTION 0.001*125.0f
#define HIGH_MOTION 0.008*125.0f
#define HIGH_BETA 0.4f
#define LOW_BETA 0.03f
#define DEFAULT_BETA 0.05f
#define ALPHA_LPF 0.25f
#define ALPHA_HPF 0.75f

class CompFilter {
    public:
        CompFilter(bool _MAG = 1) : USE_MAG(_MAG) {}
        bool USE_MAG;


        quat_t q = {0.0, 0.0, 0.0, 1.0};
        // Params for HPF and LPF:
        vec3_t accFiltered = {0.0, 0.0, 0.0};
        vec3_t gyroFiltered = {0.0, 0.0, 0.0};
        vec3_t magFiltered = {0.0, 0.0, 0.0};
        float gyroNorm = 0.0;
        float drift = 0.0;
        float driftRate = 0.005;
        float gravX , gravY, gravZ; // Unit vector in the direction of the estimated gravity
        
        
        void UpdateQ(Measurement_t* , float );
        void InitialFiltering(Measurement_t* );
        float calculateDynamicBeta(Measurement_t );
        float invSqrt(float x);
        void GetQuaternion(quat_t* q_);
        void GetEulerRPYrad(vec3_t* rpy , float initial_heading);
        void GetEulerRPYdeg(vec3_t* rpy, float initial_heading);
        void estimatedGravityDir(float* gx, float* gy, float*gz);
        float GetAccZ(float ax, float ay, float az);
};



#endif






