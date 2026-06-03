#ifndef MAHONY_H
#define MAHONY_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Mahony Filter Parameters
#define twoKpDef  (2.0f * 1.0f) // 2 * proportional gain
#define twoKiDef  (2.0f * 0.0f) // 2 * integral gain

extern float q0, q1, q2, q3; // Quaternions

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

#endif // MAHONY_H
