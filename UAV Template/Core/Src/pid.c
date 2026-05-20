#include "pid.h"

// Reset PID Integrals to prevent windup
void Reset_PID_Integrals(PID_Controller *pid_roll, PID_Controller *pid_pitch, PID_Controller *pid_yaw) {
    pid_roll->integral = 0.0f;
    pid_pitch->integral = 0.0f;
    pid_yaw->integral = 0.0f;
}

// PID Controller computation function implementation
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt) {
    if (dt == 0.0f) return 0.0f; // Prevent division by zero
    
    float error = setpoint - measured;
    
    // Proportional term
    float P_out = pid->Kp * error;
    
    // Integral term with Anti-Windup (very important for drones!)
    pid->integral += error * dt;
    
    // Clamp the integral to prevent it from growing infinitely while on the ground
    float i_max = 400.0f; 
    if(pid->integral > i_max) pid->integral = i_max;
    else if(pid->integral < -i_max) pid->integral = -i_max;
    
    float I_out = pid->Ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float D_out = pid->Kd * derivative;
    
    // Total output
    float output = P_out + I_out + D_out;
    
    // Constrain output
    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_error = error;
    
    return output;
}

// Angular PID Controller computation (handles 360-degree wrap-around)
float PID_Compute_Angular(PID_Controller *pid, float setpoint, float measured, float dt) {
    if (dt == 0.0f) return 0.0f; 
    
    // Calculate shortest path error
    float error = setpoint - measured;
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    
    // Proportional term
    float P_out = pid->Kp * error;
    
    // Integral term with Anti-Windup
    pid->integral += error * dt;
    float i_max = 400.0f; 
    if(pid->integral > i_max) pid->integral = i_max;
    else if(pid->integral < -i_max) pid->integral = -i_max;
    
    float I_out = pid->Ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float D_out = pid->Kd * derivative;
    
    // Total output
    float output = P_out + I_out + D_out;
    
    // Constrain output
    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_error = error;
    
    return output;
}
