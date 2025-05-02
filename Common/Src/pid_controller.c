#include "pid_controller.h"
#include <math.h>

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float T, 
              float max_output, float min_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->T = T;
    pid->max_output = max_output;
    pid->min_output = min_output;
    
    /* Default integral limits to output limits */
    pid->max_integral = max_output;
    pid->min_integral = min_output;
    
    /* Reset runtime variables */
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    
    pid->is_initialized = true;
}

void PID_Reset(PID_Controller* pid) {
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

void PID_SetIntegralLimits(PID_Controller* pid, float max, float min) {
    pid->max_integral = max;
    pid->min_integral = min;
}

float PID_Calculate(PID_Controller* pid, float setpoint, float measurement) {
    if (!pid->is_initialized) {
        return 0.0f;
    }
    
    float error = setpoint - measurement;
    
    /* Proportional term */
    float proportional = pid->Kp * error;
    
    /* Integral term */
    pid->integral += pid->Ki * error * pid->T;
    
    /* Anti-windup for integral term */
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < pid->min_integral) {
        pid->integral = pid->min_integral;
    }
    
    /* Derivative term (using backward difference) */
    float derivative = pid->Kd * (error - pid->prev_error) / pid->T;
    pid->prev_error = error;
    
    /* Compute total output */
    float output = proportional + pid->integral + derivative;
    
    /* Output clamping */
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }
    
    return output;
}

void PID_UpdateParameters(PID_Controller* pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void PID_SetSampleTime(PID_Controller* pid, float T) {
    pid->T = T;
}
