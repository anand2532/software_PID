#include <stdio.h>
#include "pid_controller.h"

int main() {
    // Initialize PID controller
    PIDController pid;
    float setpoint = 25.0f;  // Desired temperature (°C)
    float Kp = 10.0f;  // Proportional gain
    float Ki = 0.1f;   // Integral gain
    float Kd = 1.0f;   // Derivative gain
    float output_min = 0.0f;   // Minimum heater power
    float output_max = 100.0f; // Maximum heater power
    
    // Initialize the PID controller with the above parameters
    pid_init(&pid, Kp, Ki, Kd, setpoint, output_min, output_max);
    
    // Simulation parameters
    float current_temp = 20.0f;  // Starting temperature
    float ambient_temp = 18.0f;  // Ambient temperature
    float dt = 0.1f;             // Time step (seconds)
    int num_steps = 1000;        // Number of simulation steps
    
    // Print CSV header
    // printf("Time(s),Temperature(°C),Heater Power(%)\n");
    printf("Time(s),Temperature(°C),Heater Power(%%)\n");  // Note the escaped %%
    
    // Run simulation
    for (int i = 0; i < num_steps; i++) {
        // Calculate current time
        float time = i * dt;
        
        // Compute PID control action (heater power)
        float heater_power = pid_compute(&pid, current_temp, dt);
        
        // Simulate the process for one time step
        current_temp = simulate_process(current_temp, heater_power, ambient_temp, dt);
        
        // Output results
        printf("%.1f,%.2f,%.2f\n", time, current_temp, heater_power);
        
        // Simulation analysis:
        // 1. Initial phase: The heater power will be high as the system tries to reach the setpoint quickly.
        // 2. Approaching setpoint: The heater power will decrease as the temperature nears the setpoint.
        // 3. Maintaining setpoint: The heater power will stabilize to counteract heat loss to the environment.
        // 4. Disturbances: If ambient temperature changes, the system will adjust the heater power to maintain the setpoint.
        
        // The effectiveness of the PID controller can be evaluated by observing:
        // - Rise time: How quickly the system approaches the setpoint
        // - Overshoot: Whether the temperature exceeds the setpoint
        // - Settling time: How long it takes to stabilize around the setpoint
        // - Steady-state error: The final difference between the temperature and setpoint
    }
    
    return 0;
}