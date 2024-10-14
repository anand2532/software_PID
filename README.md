# PID Controller Simulation

This project implements a simple PID (Proportional-Integral-Derivative) controller in C, along with a basic simulation of a temperature control system. It serves as an educational tool to understand the principles of PID control and its application in process control systems.

## Project Structure

- `pid_controller.h`: Header file containing the PID controller structure and function prototypes.
- `pid_controller.c`: Implementation of the PID controller and simulation functions.
- `main.c`: Main program that sets up and runs the PID controller simulation.
- `Makefile`: Build script for compiling the project.

## Compilation

To compile the project, simply run:

```
make
```

This will create an executable named `pid_controller`.

## Running the Simulation

To run the simulation and output the results to a CSV file, use:

```
./pid_controller > output.csv
```

The output file will contain columns for Time, Temperature, and Heater Power.

## Cleaning Up

To remove compiled objects and the executable, run:

```
make clean
```

## Customization

You can modify the PID parameters, setpoint, and simulation parameters in the `main.c` file to experiment with different control scenarios. Key parameters to adjust include:

- `Kp`, `Ki`, `Kd`: PID gains
- `setpoint`: Desired temperature
- `current_temp`: Initial temperature
- `ambient_temp`: Ambient temperature
- `dt`: Time step
- `num_steps`: Number of simulation steps

## Understanding the Results

The simulation output allows you to observe:

1. How quickly the system reaches the setpoint (rise time)
2. Whether the temperature overshoots the setpoint
3. How long it takes for the temperature to stabilize (settling time)
4. The final steady-state error (if any)

By adjusting the PID parameters, you can see how they affect the system's behavior and performance.

## Further Development

Possible enhancements to this project could include:

1. Implementing more advanced PID algorithms (e.g., with anti-windup)
2. Adding a user interface for real-time parameter adjustment
3. Incorporating more complex process models
4. Implementing auto-tuning algorithms for PID parameters

