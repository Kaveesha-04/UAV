#include "pid.h"

// Reset PID Integrals to prevent windup
void Reset_PID_Integrals(PID_Controller *pid_roll, PID_Controller *pid_pitch, PID_Controller *pid_yaw) {
    pid_roll->integral = 0.0f;
    pid_pitch->integral = 0.0f;
    pid_yaw->integral = 0.0f;
}

// PID Controller computation function implementation
float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt, float tpa_factor) {
    if (dt == 0.0f) return 0.0f; // Prevent division by zero
    
    float error = setpoint - measured;
    
    // Proportional term (with TPA)
    float P_out = (pid->Kp * tpa_factor) * error;
    
    // Derivative on Measurement (with TPA and EMA Low-Pass Filter)
    float raw_derivative = -(measured - pid->prev_measured) / dt;
    float derivative = (0.3f * raw_derivative) + (0.7f * pid->prev_derivative);
    pid->prev_derivative = derivative;
    float D_out = (pid->Kd * tpa_factor) * derivative;

    // Feedforward term
    float raw_ff = (setpoint - pid->prev_setpoint) / dt;
    float FF_out = pid->Kf * raw_ff;
    
    // Dynamic Anti-Windup (Conditional Integration)
    // Check if output is saturated and error is pushing in the same direction
    float current_output = P_out + (pid->Ki * pid->integral) + D_out + FF_out;
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
    float output = P_out + I_out + D_out + FF_out;
    
    // Constrain output
    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_measured = measured;
    pid->prev_setpoint = setpoint;
    
    return output;
}

// Angular PID Controller computation (handles 360-degree wrap-around)
float PID_Compute_Angular(PID_Controller *pid, float setpoint, float measured, float dt, float tpa_factor) {
    if (dt == 0.0f) return 0.0f; 
    
    // Calculate shortest path error
    float error = setpoint - measured;
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;
    
    // Proportional term
    float P_out = (pid->Kp * tpa_factor) * error;
    
    // Derivative on Measurement (shortest path wrap-around with EMA Low-Pass Filter)
    float meas_diff = measured - pid->prev_measured;
    if (meas_diff > 180.0f) meas_diff -= 360.0f;
    else if (meas_diff < -180.0f) meas_diff += 360.0f;
    
    float raw_derivative = -meas_diff / dt;
    float derivative = (0.3f * raw_derivative) + (0.7f * pid->prev_derivative);
    pid->prev_derivative = derivative;
    float D_out = (pid->Kd * tpa_factor) * derivative;

    // Feedforward term (shortest path wrap-around)
    float sp_diff = setpoint - pid->prev_setpoint;
    if (sp_diff > 180.0f) sp_diff -= 360.0f;
    else if (sp_diff < -180.0f) sp_diff += 360.0f;
    
    float raw_ff = sp_diff / dt;
    float FF_out = pid->Kf * raw_ff;
    
    // Dynamic Anti-Windup (Conditional Integration)
    float current_output = P_out + (pid->Ki * pid->integral) + D_out + FF_out;
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
    float output = P_out + I_out + D_out + FF_out;
    
    // Constrain output
    if(output > pid->out_max) output = pid->out_max;
    else if(output < pid->out_min) output = pid->out_min;
    
    pid->prev_measured = measured;
    pid->prev_setpoint = setpoint;
    
    return output;
}
