#ifndef PID_H
#define PID_H

#include <stdint.h>

// PID Controller එක සඳහා දත්ත ව්‍යුහය
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;              // Feedforward Gain
    float integral;
    float prev_measured;   // For Derivative on Measurement
    float prev_setpoint;   // For Feedforward
    float prev_derivative; // D-term Low-Pass Filter state
    float out_max;
    float out_min;
} PID_Controller;

// Function Prototype
void Reset_PID_Integrals(PID_Controller *pid_roll, PID_Controller *pid_pitch, PID_Controller *pid_yaw);
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt, float tpa_factor);
float PID_Compute_Angular(PID_Controller *pid, float setpoint, float measured, float dt, float tpa_factor);

#endif // PID_H
