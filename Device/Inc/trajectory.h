#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

#define STEP_ANGLE 0.01f // Minimum angle for each motor movement, unit: rad

#pragma pack(1)
typedef struct
{
    float target_position;       // Target position
    float current_position;      // Current position
    float interpolated_position; // Interpolated target position

    // Cubic spline coefficients
    float a, b, c, d;

    // Interpolation step count and progress
    int current_step;
    int total_steps;
    float step_size;
} Trajectory;
#pragma pack()

/**
 * @brief Initialize trajectory planner
 * @param trajectory Trajectory structure pointer
 * @param current_position Current position
 * @param target_position Target position
 * 
 * @note This function will calculate cubic spline interpolation coefficients for smooth trajectory planning
 */
void Trajectory_Init(Trajectory *trajectory, float current_position, float target_position);

/**
 * @brief Get interpolated target position
 * @param trajectory Trajectory structure pointer
 * @return Interpolated target position
 * 
 * @note Calculate cubic spline interpolated position based on current step
 */
float Trajectory_GetTargetPosition(Trajectory *trajectory);

/**
 * @brief Update interpolation status
 * @param trajectory Trajectory structure pointer
 * 
 * @note Update current step count and advance trajectory execution progress
 */
void Trajectory_Update(Trajectory *trajectory);

/**
 * @brief Check if trajectory is complete
 * @param trajectory Trajectory structure pointer
 * @return 1: trajectory complete, 0: trajectory incomplete
 * 
 * @note Determine if trajectory execution is finished by comparing current step and total steps
 */
int Trajectory_IsComplete(Trajectory *trajectory);

#endif // TRAJECTORY_H
