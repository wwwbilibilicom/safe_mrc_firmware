#include "trajectory.h"
#include <math.h>
#include <stdlib.h>

// 计算三次样条系数，确保两端斜率为0
static void CalculateCubicCoefficients(Trajectory *trajectory, float start_pos, float end_pos)
{
    float delta_pos = -end_pos + start_pos;

    // 三次多项式 S(t) = a*t^3 + b*t^2 + c*t + d
    // 满足 S(0) = start_pos, S(1) = end_pos
    // 满足 S'(0) = 0, S'(1) = 0
    // 解得:
    // a = 2*delta_pos
    // b = -3*delta_pos
    // c = 0
    // d = start_pos

    trajectory->a = 2.0f * delta_pos;
    trajectory->b = -3.0f * delta_pos;
    trajectory->c = 0.0f;
    trajectory->d = start_pos;
}

// 初始化轨迹
void Trajectory_Init(Trajectory *trajectory, float current_position, float target_position)
{
    trajectory->current_position = current_position;
    trajectory->target_position = target_position;
    trajectory->interpolated_position = current_position;

    // 计算三次样条系数
    CalculateCubicCoefficients(trajectory, current_position, target_position);

    // 计算步数，根据位置差值动态调整
    float delta = fabsf(target_position - current_position);
    trajectory->total_steps = (int)(delta / STEP_ANGLE);
    if (trajectory->total_steps < 1)
    {
        trajectory->total_steps = 1;
    }

    trajectory->step_size = 1.0f / (float)trajectory->total_steps;
    trajectory->current_step = 0;
}

// 获取插值后的目标位置
float Trajectory_GetTargetPosition(Trajectory *trajectory)
{
    if (trajectory->current_step <= trajectory->total_steps)
    {
        float t = (float)trajectory->current_step * trajectory->step_size; // 插值因子 t 从 0 到 1
        trajectory->interpolated_position = trajectory->a * t * t * t +
                                            trajectory->b * t * t +
                                            trajectory->c * t +
                                            trajectory->d;
    }
    return trajectory->interpolated_position;
}

// 更新插值状态
void Trajectory_Update(Trajectory *trajectory)
{
    if (trajectory->current_step <= trajectory->total_steps)
    {
        trajectory->current_step++;
    }
}

// 检查是否轨迹完成
int Trajectory_IsComplete(Trajectory *trajectory)
{
    return trajectory->current_step > trajectory->total_steps;
}
