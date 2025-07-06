#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define FILTER_NUM 8 // //滑动平均滤波数值个�??

#include "stdint.h"

#pragma pack(1)
    typedef struct 
    {
        float *buffer;        /**< Data buffer pointer */
        uint16_t size;        /**< Size of filter buffer */
        uint16_t index;       /**< Current location in buffer */
        uint16_t fill;        /**< Buffer filled level */
        float sum;            /**< Cumulative value of buffer */
        float filtered;       /**< Filtered output */
        uint16_t sample_time; /**< data sampling time interval */
        uint32_t last_time;   /**< last sampled time */
        // uint8_t update_flag;
    } movingAverage_t;
#pragma pack()


    /**
     * @brief filter object initialization
     *
     * @param context [in] instance of filter object
     * @param filter_size [in] size of filter buffer
     * @param sample_time [in] filter sampling time in ms
     */
    void moving_average_create(movingAverage_t *context, uint16_t filter_size, uint16_t sample_time);

    /**
     * @brief filter process function
     *
     * @param context [in] instance of filter object
     * @param input [in] data sample to filter
     */
    float moving_average_filter(movingAverage_t *context, float input);


#pragma pack(1)
typedef struct
{
    /* data */
    float LastP; // 上一次估计的协方�?
    float Now_P; // 当前估�?�的协方�?
    float out; // 输出
    float Kg; // 卡尔曼�?�益
    float Q; // 过程�?声协方差
    float R; // 测量�?声协方差
}FirstOrderKalmanFilter;
#pragma pack()

#pragma pack(1)

    typedef struct 
    {
        float a1;
        float b0,b1;
        float x1;
        float y1;
    }Filter;    //IIR滤波
#pragma pack()
    
#pragma pack(1)

    typedef struct 
    {
        Filter lowPassFilter;
        Filter highPassFilter;
    }BandPassFilter;
#pragma pack()

void FirstOrder_KalmanFilter_Init(FirstOrderKalmanFilter *KalmanFilter,float Q,float R);
float FirstOrder_KalmanFilter_Update(FirstOrderKalmanFilter *KalmanFilter,float input);
void HighPassFilter_Init(Filter *filter,float SampleRate,float CutoffFreq);
void LowPassFilter_Init(Filter *filter,float SampleRate,float CutoffFreq);
void BandPassFilter_Init(float highCutoffFreq, float lowCutoffFreq, float sampleRate, BandPassFilter *filter);
float BandPassFilter_Update(BandPassFilter * filter, float input);

/**
 * @brief Simple first-order low-pass filter structure
 *
 * This structure holds the state for a simple exponential smoothing low-pass filter.
 * The filter equation is: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
 */
#pragma pack(1)
typedef struct {
    float alpha;      /**< Smoothing factor (0 < alpha < 1) */
    float prev;       /**< Previous output value */
    uint8_t has_prev; /**< Flag to indicate if prev is valid */
} SimpleLowPassFilter;
#pragma pack()

/**
 * @brief Initialize a simple first-order low-pass filter using sample rate and cutoff frequency
 *
 * This function computes the smoothing factor (alpha) based on the sample rate and cutoff frequency.
 *
 * @param filter Pointer to the SimpleLowPassFilter structure
 * @param sample_rate Sampling rate in Hz
 * @param cutoff_freq Cutoff frequency in Hz
 */
void SimpleLowPassFilter_InitByCutoff(SimpleLowPassFilter *filter, float sample_rate, float cutoff_freq);

/**
 * @brief (Deprecated) Initialize a simple first-order low-pass filter with alpha
 *
 * @param filter Pointer to the SimpleLowPassFilter structure
 * @param alpha Smoothing factor (0 < alpha < 1)
 */
void SimpleLowPassFilter_Init(SimpleLowPassFilter *filter, float alpha);

/**
 * @brief Update the low-pass filter with a new input value
 *
 * @param filter Pointer to the SimpleLowPassFilter structure
 * @param input New input value to be filtered
 * @return Filtered output value
 */
float SimpleLowPassFilter_Update(SimpleLowPassFilter *filter, float input);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */
