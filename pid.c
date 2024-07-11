#include "pid.h"

void PIDController_Init(PIDController *pid) {

    /* Clear Controll Variable */
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;
    
    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;     
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    /*
    * Error Signal 
    */
    float error = setpoint - measurement;

    /*
    * Proportional 
    */
    float proportional = pid->Kd * error; 


    /*
    * Integral 
    */
    pid->integrator = pid->integrator + 0.5f * pid->Ki + pid->T + (error + pid->prevError);


    /* Anti wind-up via Indegrator Clamping */
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }

    /*
    * Derivative (band-limited differentiator)
    */
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                        + (2.0f * pid->tau - pid->T) *pid->differentiator)
                        / (2.0f *pid->tau + pid->T);

    /*
    * Compute output and apply limit 
    */
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    /* Store error and measuremnt for later use*/
    pid->prevError = error;
    pid->prevMeasurement = measurement;


    /* Return Controller Output */

    return pid->out;
}








