#include "filter.h"
#include "stdlib.h"

#define TICK_TIMER HAL_GetTick()

/**
 * 创建移动平均滤波器上下文
 *
 * 该函数初始化一个移动平均滤波器的上下文结构体，为其分配缓冲区并设置初始值。
 * 它适用于在滤波器中处理连续的样本数据，通过计算一段时间内的平均值来平滑数据。
 *
 * @param context 指向移动平均滤波器上下文结构体的指针
 * @param filter_size 滤波器的大小，即用于计算平均值的样本数量
 * @param sample_time 样本的时间间隔，用于计算新样本是否应该被加入到滤波器中
 */
void moving_average_create(movingAverage_t *context,
                           uint16_t filter_size,
                           uint16_t sample_time)
{
    // 释放之前可能分配的缓冲区，以避免内存泄漏
    free(context->buffer);

    // 初始化滤波器的大小，缓冲区大小，以及所有其他相关变量
    context->size = filter_size;
    // 分配足够空间用于存储滤波器大小的浮点数样本
    context->buffer = (float *)malloc(filter_size * sizeof(float));
    context->index = 0;                 // 当前样本索引
    context->sum = 0;                   // 用于计算平均值的总和
    context->fill = 0;                  // 当前缓冲区的填充程度
    context->filtered = 0;              // 已经经过滤波处理的样本数量
    context->sample_time = sample_time; // 样本的时间间隔
    context->last_time = 0;             // 上次处理样本的时间戳
}

/**
 * @brief Applies a moving average filter to the given input.
 *
 * This function smooths the input signal by storing recent input values in a circular buffer and calculating their average.
 * The moving average filter helps reduce noise in the input data.
 *
 * @param context Pointer to the moving average filter context structure. The context structure contains configuration and state information for the filter.
 * @param filter_input The value that needs to be filtered.
 */
void moving_average_filter(movingAverage_t *context,
                           float filter_input)
{
    // Update the filter state instead of checking if the sampling time has elapsed.
    // if ((TICK_TIMER - context->last_time) > context->sample_time)
    // {
    //     context->last_time = TICK_TIMER;
    // }

    // If the buffer is full, remove the old input value from the start of the buffer.
    if (context->fill)
    {
        context->sum -= context->buffer[context->index];
    }

    // Add the new input value to the buffer and update the sum.
    context->buffer[context->index] = filter_input;
    context->sum += context->buffer[context->index];

    // Move the buffer index to the next position.
    context->index++;
    // If the index exceeds the buffer size, reset it to 0 to implement a circular buffer.
    if (context->index >= context->size)
    {
        context->index = 0;
    }

    // If the buffer is not yet full, continue filling the buffer.
    if (context->fill < context->size)
    {
        context->fill++;
    }

    // Calculate the moving average and update the filtered output.
    context->filtered = (float)(context->sum / context->fill);
    // }
}

void FirstOrder_KalmanFilter_Init(FirstOrderKalmanFilter *KalmanFilter, float Q, float R)
{
    KalmanFilter->LastP = 0.02;
    KalmanFilter->Now_P = 0;
    KalmanFilter->out = 0;
    KalmanFilter->Kg = 0;
    KalmanFilter->Q = Q;
    KalmanFilter->R = R;
}

float FirstOrder_KalmanFilter_Update(FirstOrderKalmanFilter *KalmanFilter, float input)
{
    // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    KalmanFilter->Now_P = KalmanFilter->LastP + KalmanFilter->Q;
    // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    KalmanFilter->Kg = KalmanFilter->Now_P / (KalmanFilter->Now_P + KalmanFilter->R);
    // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    KalmanFilter->out = KalmanFilter->out + KalmanFilter->Kg * (input - KalmanFilter->out); // 因为这一次的预测值就是上一次的输出值
    // 更新协方差方程: 本次的系统协方差付给 KalmanFilter->LastP 威下一次运算准备。
    KalmanFilter->LastP = (1 - KalmanFilter->Kg) * KalmanFilter->Now_P;
    return KalmanFilter->out;
}




#define SAMPLE_RATE 1000.0f //采样频率Hz
#define PI 3.1415926f


void HighPassFilter_Init(Filter *filter,float SampleRate,float CutoffFreq){
    float RC=1.0f/(2.0f*PI*CutoffFreq);
    float dt=1.0f/SampleRate;
    float alpha=RC/(RC+dt);

    filter->b0 = alpha;
    filter->b1 = -alpha;
    filter->a1 = alpha;
    filter->x1 = 0.0f;  //历史输入值
    filter->y1 = 0.0f;  //历史输出值
}

void LowPassFilter_Init(Filter *filter,float SampleRate,float CutoffFreq){
    float RC=1.0f/(2.0f*PI*CutoffFreq);
    float dt=1.0f/SampleRate;
    float alpha=dt/(RC+dt);

    filter->b0 = alpha;
    filter->b1 = 0.0f;
    filter->a1 = 1.0f - alpha;
    filter->x1 = 0.0f;
    filter->y1 = 0.0f;
}

float ApplyFilter(Filter* filter, float input) {
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->a1 * filter->y1;  //一阶IIR滤波公式

    // 更新历史值
    filter->x1 = input;
    filter->y1 = output;

    return output;
}

void BandPassFilter_Init(float highCutoffFreq, float lowCutoffFreq, float sampleRate, BandPassFilter *filter)
{
    HighPassFilter_Init(&filter->highPassFilter, sampleRate, highCutoffFreq);
    LowPassFilter_Init(&filter->lowPassFilter, sampleRate, lowCutoffFreq);
}

float BandPassFilter_Update(BandPassFilter * filter, float input)
{

    float highpass_output = ApplyFilter(&filter->highPassFilter, input);
    float filtered_output = ApplyFilter(&filter->lowPassFilter, highpass_output);

    return filtered_output;
}
