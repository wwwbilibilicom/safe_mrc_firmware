#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdint.h>

#define STEP_ANGLE 0.01f // 每次电机移动的最小角度，单位：rad

#pragma pack(1)
typedef struct
{
    float target_position;       // 目标位置
    float current_position;      // 当前的位置
    float interpolated_position; // 插值后的目标位置

    // 三次样条系数
    float a, b, c, d;

    // 插值步数和进度
    int current_step;
    int total_steps;
    float step_size;
} Trajectory;
#pragma pack()
// 初始化轨迹
void Trajectory_Init(Trajectory *trajectory, float current_position, float target_position);

// 获取插值后的目标位置
float Trajectory_GetTargetPosition(Trajectory *trajectory);

// 更新插值状态
void Trajectory_Update(Trajectory *trajectory);

// 检查是否轨迹完成
int Trajectory_IsComplete(Trajectory *trajectory);

#endif // TRAJECTORY_H
