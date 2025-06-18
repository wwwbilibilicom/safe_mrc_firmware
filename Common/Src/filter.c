#include "filter.h"
#include "stdlib.h"

#define TICK_TIMER HAL_GetTick()

/**
 * Create moving average filter context
 *
 * This function initializes a moving average filter context structure, allocates buffer and sets initial values.
 * It is suitable for processing continuous sample data in filters, smoothing data by calculating the average over a period of time.
 *
 * @param context Pointer to the moving average filter context structure
 * @param filter_size Size of the filter, i.e., the number of samples used to calculate the average
 * @param sample_time Time interval between samples, used to determine if a new sample should be added to the filter
 */
void moving_average_create(movingAverage_t *context,
                           uint16_t filter_size,
                           uint16_t sample_time)
{
    // Free previously allocated buffer to avoid memory leaks
    free(context->buffer);

    // Initialize filter size, buffer size, and all other related variables
    context->size = filter_size;
    // Allocate sufficient space for storing filter_size float samples
    context->buffer = (float *)malloc(filter_size * sizeof(float));
    context->index = 0;                 // Current sample index
    context->sum = 0;                   // Sum used to calculate average
    context->fill = 0;                  // Current buffer fill level
    context->filtered = 0;              // Number of samples that have been filtered
    context->sample_time = sample_time; // Time interval between samples
    context->last_time = 0;             // Timestamp of last processed sample
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
float moving_average_filter(movingAverage_t *context,
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
    return context->filtered;
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
    // Prediction covariance equation: k-th moment system estimation covariance = k-1 moment system covariance + process noise covariance
    KalmanFilter->Now_P = KalmanFilter->LastP + KalmanFilter->Q;
    // Kalman gain equation: Kalman gain = k-th moment system estimation covariance / (k-th moment system estimation covariance + observation noise covariance)
    KalmanFilter->Kg = KalmanFilter->Now_P / (KalmanFilter->Now_P + KalmanFilter->R);
    // Update optimal value equation: k-th moment state variable optimal value = state variable prediction value + Kalman gain * (measurement value - state variable prediction value)
    KalmanFilter->out = KalmanFilter->out + KalmanFilter->Kg * (input - KalmanFilter->out); // Because this prediction value is the last output value
    // Update covariance equation: assign this system covariance to KalmanFilter->LastP for next calculation preparation.
    KalmanFilter->LastP = (1 - KalmanFilter->Kg) * KalmanFilter->Now_P;
    return KalmanFilter->out;
}




#define SAMPLE_RATE 1000.0f // Sampling frequency Hz
#define PI 3.1415926f


void HighPassFilter_Init(Filter *filter,float SampleRate,float CutoffFreq){
    float RC=1.0f/(2.0f*PI*CutoffFreq);
    float dt=1.0f/SampleRate;
    float alpha=RC/(RC+dt);

    filter->b0 = alpha;
    filter->b1 = -alpha;
    filter->a1 = alpha;
    filter->x1 = 0.0f;  // Historical input value
    filter->y1 = 0.0f;  // Historical output value
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
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->a1 * filter->y1;  // First-order IIR filter formula

    // Update historical values
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
