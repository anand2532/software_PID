#include "pid_controller.h"

void pid_init(PIDController *pid, float Kp, float Ki, float Kd, float setpoint, float output_min, float output_max) {
    // Initialize PID controller parameters
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;  // Start with zero accumulated error
    pid->prev_error = 0.0f;  // No previous error at start
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    // Note: The choice of Kp, Ki, and Kd values is crucial for system performance.
    // - Too high values can lead to instability or oscillations.
    // - Too low values can result in slow response or steady-state errors.
    // Proper tuning often requires experimentation or advanced methods like Ziegler-Nichols.
}

float pid_compute(PIDController *pid, float measured_value, float dt) {
    // Calculate the error: difference between setpoint and measured value
    float error = pid->setpoint - measured_value;
    
    // Proportional term: direct response to current error
    // A larger Kp increases the speed of the control system response
    float P = pid->Kp * error;
    
    // Integral term: addresses accumulated error over time
    // Helps eliminate steady-state error, but can cause overshoot
    pid->integral += error * dt;
    float I = pid->Ki * pid->integral;
    
    // Derivative term: predicts future error based on rate of change
    // Helps reduce overshoot and settling time, but sensitive to noise
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    
    // Calculate total output: sum of P, I, and D terms
    float output = P + I + D;
    
    // Apply output limits (anti-windup technique)
    // This prevents integral windup and ensures the output stays within practical limits
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    // Save current error for next iteration's derivative calculation
    pid->prev_error = error;
    
    return output;
}

float simulate_process(float current_temp, float heater_power, float ambient_temp, float dt) {
    // Simple thermal process simulation
    // This model assumes:
    // 1. Heat is added to the system by the heater
    // 2. Heat is lost to the environment based on the temperature difference
    // 3. The system has a certain heat capacity
    
    float heat_transfer_coeff = 0.1f;  // Rate of heat loss to environment
    float heat_capacity = 1000.0f;  // System's resistance to temperature change
    
    // Calculate heat added by the heater
    float heat_added = heater_power * dt;
    
    // Calculate heat lost to the environment
    // Heat loss is proportional to the temperature difference
    float heat_lost = heat_transfer_coeff * (current_temp - ambient_temp) * dt;
    
    // Calculate the net temperature change
    float temp_change = (heat_added - heat_lost) / heat_capacity;
    
    // Return the new temperature
    return current_temp + temp_change;
}