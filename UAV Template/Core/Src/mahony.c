#include "mahony.h"

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// IMU algorithm update (6DOF - Accelerometer and Gyroscope only)
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= (M_PI / 180.0f);
    gy *= (M_PI / 180.0f);
    gz *= (M_PI / 180.0f);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) && !((gx == 0.0f) && (gy == 0.0f) && (gz == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKiDef > 0.0f) {
            integralFBx += twoKiDef * halfex * dt;
            integralFBy += twoKiDef * halfey * dt;
            integralFBz += twoKiDef * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
