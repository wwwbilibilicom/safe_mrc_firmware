#include "trajectory.h"
#include <math.h>
#include <stdlib.h>

/**
 * @brief Calculate cubic spline interpolation coefficients
 * @param trajectory Trajectory structure pointer
 * @param start_pos Starting position
 * @param end_pos Ending position
 * 
 * @note Calculate cubic spline interpolation coefficients satisfying boundary conditions
 * @note Boundary conditions: S(0) = start_pos, S(1) = end_pos, S'(0) = 0, S'(1) = 0
 * @note Cubic polynomial form: S(t) = a*t^3 + b*t^2 + c*t + d
 */
static void CalculateCubicCoefficients(Trajectory *trajectory, float start_pos, float end_pos)
{
    float delta_pos = -end_pos + start_pos;

    // Cubic polynomial S(t) = a*t^3 + b*t^2 + c*t + d
    // Satisfy S(0) = start_pos, S(1) = end_pos
    // Satisfy S'(0) = 0, S'(1) = 0
    // Solution:
    // a = 2*delta_pos
    // b = -3*delta_pos
    // c = 0
    // d = start_pos

    trajectory->a = 2.0f * delta_pos;
    trajectory->b = -3.0f * delta_pos;
    trajectory->c = 0.0f;
    trajectory->d = start_pos;
}

/**
 * @brief Initialize trajectory planner
 * @param trajectory Trajectory structure pointer
 * @param current_position Current position
 * @param target_position Target position
 * 
 * @note This function will calculate cubic spline interpolation coefficients for smooth trajectory planning
 * @note Dynamically adjust step count based on position difference to ensure smooth trajectory
 */
void Trajectory_Init(Trajectory *trajectory, float current_position, float target_position)
{
    trajectory->current_position = current_position;
    trajectory->target_position = target_position;
    trajectory->interpolated_position = current_position;

    // Calculate cubic spline coefficients
    CalculateCubicCoefficients(trajectory, current_position, target_position);

    // Calculate step count, dynamically adjust based on position difference
    float delta = fabsf(target_position - current_position);
    trajectory->total_steps = (int)(delta / STEP_ANGLE);
    if (trajectory->total_steps < 1)
    {
        trajectory->total_steps = 1;
    }

    trajectory->step_size = 1.0f / (float)trajectory->total_steps;
    trajectory->current_step = 0;
}

/**
 * @brief Get interpolated target position
 * @param trajectory Trajectory structure pointer
 * @return Interpolated target position
 * 
 * @note Calculate cubic spline interpolated position based on current step
 * @note Use cubic polynomial: S(t) = a*t^3 + b*t^2 + c*t + d
 */
float Trajectory_GetTargetPosition(Trajectory *trajectory)
{
    if (trajectory->current_step <= trajectory->total_steps)
    {
        float t = (float)trajectory->current_step * trajectory->step_size; // Interpolation factor t from 0 to 1
        trajectory->interpolated_position = trajectory->a * t * t * t +
                                            trajectory->b * t * t +
                                            trajectory->c * t +
                                            trajectory->d;
    }
    return trajectory->interpolated_position;
}

/**
 * @brief Update interpolation status
 * @param trajectory Trajectory structure pointer
 * 
 * @note Update current step count and advance trajectory execution progress
 * @note Stop updating when reaching total step count
 */
void Trajectory_Update(Trajectory *trajectory)
{
    if (trajectory->current_step <= trajectory->total_steps)
    {
        trajectory->current_step++;
    }
}

/**
 * @brief Check if trajectory is complete
 * @param trajectory Trajectory structure pointer
 * @return 1: trajectory complete, 0: trajectory incomplete
 * 
 * @note Determine if trajectory execution is finished by comparing current step and total steps
 */
int Trajectory_IsComplete(Trajectory *trajectory)
{
    return trajectory->current_step > trajectory->total_steps;
}
