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
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float D_out = pid->Kd * derivative;
    
    // Dynamic Anti-Windup (Conditional Integration)
    // Check if output is saturated and error is pushing in the same direction
    float current_output = P_out + (pid->Ki * pid->integral) + D_out;
    uint8_t saturated = 0;
    
    if (current_output >= pid->out_max && error > 0.0f) saturated = 1;
    if (current_output <= pid->out_min && error < 0.0f) saturated = 1;
    
    // Only integrate if not saturated
    if (!saturated) {
        pid->integral += error * dt;
    }
    
    // Clamp the integral as a final safety net
    float i_max = 400.0f; 
    if(pid->integral > i_max) pid->integral = i_max;
    else if(pid->integral < -i_max) pid->integral = -i_max;
    
    float I_out = pid->Ki * pid->integral;
    
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
    
    // Derivative term (shortest path wrap-around)
    float diff = error - pid->prev_error;
    if (diff > 180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    float derivative = diff / dt;
    float D_out = pid->Kd * derivative;
    
    // Dynamic Anti-Windup (Conditional Integration)
    float current_output = P_out + (pid->Ki * pid->integral) + D_out;
    uint8_t saturated = 0;
    
    if (current_output >= pid->out_max && error > 0.0f) saturated = 1;
    if (current_output <= pid->out_min && error < 0.0f) saturated = 1;
    
    if (!saturated) {
        pid->integral += error * dt;
    }
    
    // Hard clamp just in case
    float i_max = 400.0f; 
    if(pid->integral > i_max) pid->integral = i_max;
    else if(pid->integral < -i_max) pid->integral = -i_max;
    
    float I_out = pid->Ki * pid->integral;
    
    // Total output
    float output = P_out + I_out + D_out;
    
    // Constrain output
    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_error = error;
    
    return output;
}
