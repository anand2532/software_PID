#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * PID Controller Overview:
 * 
 * A PID (Proportional-Integral-Derivative) controller is a control loop feedback
 * mechanism widely used in industrial control systems. The controller attempts 
 * to minimize the error between a measured process variable and a desired setpoint
 * by calculating and then outputting a corrective action that can adjust the process
 * accordingly.
 *
 * The PID controller calculation involves three separate parameters:
 * 1. Proportional (P): Depends on the present error
 * 2. Integral (I): Depends on the accumulation of past errors
 * 3. Derivative (D): Is a prediction of future errors, based on current rate of change
 *
 * The weighted sum of these three actions is used to adjust the process via a control 
 * element such as the power supplied to a heating element or the position of a control valve.
 */

// PID controller structure
typedef struct {
    float Kp;           // Proportional gain: Determines the reaction to the current error
    float Ki;           // Integral gain: Determines the reaction based on the sum of recent errors
    float Kd;           // Derivative gain: Determines the reaction based on the rate of error change
    float setpoint;     // Desired value that the process should achieve
    float integral;     // Sum of all errors over time, used for the integral term
    float prev_error;   // Previous error, used to calculate the derivative term
    float output_min;   // Minimum allowed output value (for output clamping)
    float output_max;   // Maximum allowed output value (for output clamping)
} PIDController;

/**
 * Initializes the PID controller with given parameters.
 *
 * @param pid Pointer to the PIDController structure
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param setpoint Desired process value
 * @param output_min Minimum output value
 * @param output_max Maximum output value
 */
void pid_init(PIDController *pid, float Kp, float Ki, float Kd, float setpoint, float output_min, float output_max);

/**
 * Computes the PID control action.
 *
 * @param pid Pointer to the PIDController structure
 * @param measured_value Current value of the process variable
 * @param dt Time step (in seconds)
 * @return Computed control action
 */
float pid_compute(PIDController *pid, float measured_value, float dt);

/**
 * Simulates a simple process (e.g., a heating system).
 *
 * @param current_temp Current temperature of the system
 * @param heater_power Power input to the heater
 * @param ambient_temp Ambient temperature
 * @param dt Time step (in seconds)
 * @return New temperature after the time step
 */
float simulate_process(float current_temp, float heater_power, float ambient_temp, float dt);

#endif // PID_CONTROLLER_H