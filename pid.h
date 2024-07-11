#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{ 
    /* Controller gains */
     float Kp;
     float Ki;
     float Kd;

     /* Derivative low-pass filter time constant */
     float tau;

     /* output limits */
     float limMin;
     float limMax;

     /* Integrator limit*/
     float limMinInt;
     float limMaxInt;

     /* Sample time (in Second)*/
     float T;

     /* Controller "memoru" */
     float integrator;
     float prevError;
     float differentiator;
     float prevMeasurement;

     /* Controller output */
     float out;
    
} PIDController;


void PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
#endif

