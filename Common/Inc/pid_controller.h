#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

/* PID Controller structure definition */
typedef struct PID_Controller
{
    /* Configuration parameters */
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float T;            // Sampling time (seconds)
    float max_output;   // Output upper limit
    float min_output;   // Output lower limit
    float max_integral; // Integral term upper limit
    float min_integral; // Integral term lower limit
    
    /* Runtime variables */
    float prev_error;   // Previous error value
    float integral;     // Integral term accumulation
    
    /* Status flag */
    bool is_initialized; // Initialization status
} PID_Controller;

/**
 * @brief Initialize PID controller
 * @param pid Pointer to PID controller
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param T Sampling time (seconds)
 * @param max_output Output upper limit
 * @param min_output Output lower limit
 */
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float T, 
              float max_output, float min_output);

/**
 * @brief Reset PID controller internal state
 * @param pid Pointer to PID controller
 */
void PID_Reset(PID_Controller* pid);

/**
 * @brief Set integral term limits
 * @param pid Pointer to PID controller
 * @param max Integral upper limit
 * @param min Integral lower limit
 */
void PID_SetIntegralLimits(PID_Controller* pid, float max, float min);

/**
 * @brief Compute PID output
 * @param pid Pointer to PID controller
 * @param setpoint Desired value
 * @param measurement Current process value
 * @return PID controller output
 */
float PID_Calculate(PID_Controller* pid, float setpoint, float measurement);

/**
 * @brief Update PID tuning parameters
 * @param pid Pointer to PID controller
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void PID_UpdateParameters(PID_Controller* pid, float Kp, float Ki, float Kd);

/**
 * @brief Set sampling time
 * @param pid Pointer to PID controller
 * @param T Sampling time (seconds)
 */
void PID_SetSampleTime(PID_Controller* pid, float T);

#endif /* PID_CONTROLLER_H */
