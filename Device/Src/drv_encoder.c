/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "stdio.h"
#include "usart.h"
#include "drv_encoder.h"
#include "tim.h"
#include "stdint.h"



#define ENCODER_PI 3.14159265358979323846f

/**
 * @brief Initializes the Encoder device.
 *
 * This function initializes the Encoder device by starting the timer interrupt, creating a binary semaphore,
 * initializing a first-order Kalman filter, and setting initial values for the capture index and flag.
 *
 * @return Returns 1 if the initialization is successful.
 */
int drv_encoder_init(Device_encoder_t *Encoder_dev, TIM_HandleTypeDef *Encoder_tim, uint32_t Encoder_ch)
{
    Encoder_dev->htim = Encoder_tim;
    Encoder_dev->Channel = Encoder_ch;
    FirstOrder_KalmanFilter_Init(&(Encoder_dev->Encoder_KF), 0.001f, 0.033f);
    SimpleLowPassFilter_InitByCutoff(&Encoder_dev->lowPassFilter, 10000.0f, 100.0f); // the frequency of pwm is 66kHz, which means the sample frequency is 66kHz
    moving_average_create(&(Encoder_dev->movingAverage), 100, 1);
    BandPassFilter_Init(30.0f, 130.0f, 10000.0f, &(Encoder_dev->bandPassFilter));
    Encoder_dev->Encoder_Duty.CapIndex = First_Rrising;
    Encoder_dev->Encoder_Duty.CapFlag = 0;
    // HAL_TIM_IC_Start_IT(Encoder_dev->htim, Encoder_dev->Channel); // Start the timer interrupt for input capture
    // Encoder_PWM_Start_ReadAngle(Encoder_dev);
    // Encoder_PWM_CalibrateZero(Encoder_dev);

    Encoder_SPI_CalibrateZero(Encoder_dev);

    printf("Encoder initialized successfully!\n");
    return 1;
}

static void Encoder_SPI_CalibrateZero(Device_encoder_t *Encoder_dev) {
    float sum = 0;
    for (int i = 0; i < 1000; ++i) {
        Encoder_SPI_ReadAngle_WithWait(Encoder_dev);
        sum += Encoder_dev->raw_angle;
    }
    Encoder_dev->zero_offset = sum / 1000.0f;
    Encoder_dev->round_count = 0;
    Encoder_dev->raw_continuous_angle = 0;
}

void Encoder_PWM_CalibrateZero(Device_encoder_t *Encoder_dev) {
    float sum = 0;
    for (int i = 0; i < 1000; ++i) {
        while (Encoder_dev->Encoder_Duty.CapFlag == 0)
        {
        }
        Encoder_dev->Encoder_Duty.CapFlag = 0;
        Encoder_Calculate_From_Capture(Encoder_dev);
        sum += Encoder_dev->raw_angle;
    }
    Encoder_dev->zero_offset = sum / 1000.0f;
    Encoder_dev->round_count = 0;
    Encoder_dev->raw_continuous_angle = 0;
}


/**
 * @brief Capture the duty cycle of the signal
 *
 * This function captures the duty cycle of the signal by reading the captured values from the timer.
 *
 * @param Encoder_dev Pointer to the Encoder device structure
 * @param htim Pointer to the timer handle
 */
void CaptureDutyCycle(Device_encoder_t *Encoder_dev)
{
    
    switch (Encoder_dev->Encoder_Duty.CapIndex)
    {
    case First_Rrising:
        Encoder_dev->Encoder_Duty.CapFlag = 0;
        // Encoder_dev->Encoder_Duty.CapVal[0] = HAL_TIM_ReadCapturedValue(Encoder_dev->htim, Encoder_dev->Channel);
        // printf("Capture CapVal0 is :%d\n",Encoder_dev->Encoder_Duty.CapVal[0]);
        Encoder_dev->Encoder_Duty.CapIndex = Falling;
        __HAL_TIM_SET_COUNTER(Encoder_dev->htim, 0); // Reset counter, start counting from beginning
        __HAL_TIM_SET_CAPTUREPOLARITY(Encoder_dev->htim, Encoder_dev->Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
        break;

    case Falling:
        Encoder_dev->Encoder_Duty.CapVal[1] = HAL_TIM_ReadCapturedValue(Encoder_dev->htim, Encoder_dev->Channel);
        // printf("Capture CapVal1 is :%d\n",Encoder_dev->Encoder_Duty.CapVal[1]);
        Encoder_dev->Encoder_Duty.CapIndex = Second_Rising;
        __HAL_TIM_SET_CAPTUREPOLARITY(Encoder_dev->htim, Encoder_dev->Channel, TIM_INPUTCHANNELPOLARITY_RISING);
        break;

    case Second_Rising:
        Encoder_dev->Encoder_Duty.CapVal[2] = HAL_TIM_ReadCapturedValue(Encoder_dev->htim, Encoder_dev->Channel);
        // printf("Capture CapVal2 is :%d\n",Encoder_dev->Encoder_Duty.CapVal[2]);
        HAL_TIM_IC_Stop_IT(Encoder_dev->htim, Encoder_dev->Channel);
        Encoder_dev->Encoder_Duty.CapIndex = First_Rrising;
        Encoder_dev->Encoder_Duty.CapFlag = 1;
        // uint64_t last_capture_time = Encoder_dev->CurrentSampleTime;
        Encoder_dev->PreviousSampleTime = Encoder_dev->CurrentSampleTime;
        Encoder_dev->CurrentSampleTime = getHighResTime_ns();
        // uint64_t deltaTime_ns = Encoder_dev->CurrentSampleTime - last_capture_time;
        // if (deltaTime_ns > 0) {
        //     Encoder_dev->real_freq = 1/(deltaTime_ns  * 1e-9f);
        // } else {
        //     Encoder_dev->real_freq = 0.0f;
        // }
        __HAL_TIM_SET_COUNTER(Encoder_dev->htim, 0); // Reset counter, start counting from beginning
        break;

    default:
        Error_Handler();
        break;
    }
}

/**
 * @brief Read encoder angle through SPI
 * @param Encoder_dev Encoder device structure pointer
 * 
 * @note Suitable for encoder chips that support SPI communication, such as KTH78xx series
 * @note Directly read the raw angle value from the encoder chip
 */
void Encoder_SPI_ReadAngle_WithWait(Device_encoder_t *Encoder_dev){
    Encoder_dev->previous_raw_angle = Encoder_dev->raw_angle;
    Encoder_dev->PreviousSampleTime = Encoder_dev->CurrentSampleTime;
    Encoder_dev->raw_angle = KTH78_ReadAngle();
    Encoder_dev->CurrentSampleTime = getHighResTime_ns();
}
void Encoder_SPI_ReadAngle_WithoutWait(Device_encoder_t *Encoder_dev){
    Encoder_dev->raw_angle = (float)(((Encoder_dev->spi_databack[0] << 8) + Encoder_dev->spi_databack[1]) * 360.0f/65536.0f);
}

void Encoder_SPI_ExchangeData(Device_encoder_t *Encoder_dev)
{
    KTH78_ExchangeData_WithoutWait(Encoder_dev->spi_databack);
}

void Encoder_PWM_Start_ReadAngle(Device_encoder_t *Encoder_dev){
    /* Restart capture */
    HAL_TIM_IC_Start_IT(Encoder_dev->htim, Encoder_dev->Channel);
}
static void Encoder_UpdateContinuousAngle(Device_encoder_t *Encoder_dev){
    float delta = Encoder_dev->raw_angle - Encoder_dev->previous_raw_angle;
    // 跳变处理
    if (delta > 180.0f) {
        Encoder_dev->round_count -= 1;
    } else if (delta < -180.0f) {
        Encoder_dev->round_count += 1;
    }
    Encoder_dev->raw_continuous_angle = Encoder_dev->raw_angle-Encoder_dev->zero_offset + Encoder_dev->round_count * 360.0f;
}

static void Encoder_Calculate_From_Capture(Device_encoder_t *Encoder_dev){
    uint32_t period, high_time;

    // record the time of this sample
    // Encoder_dev->PreviousSampleTime = Encoder_dev->CurrentSampleTime;
    //Encoder_dev->CurrentSampleTime = getHighResTime_ns();

    period = Encoder_dev->Encoder_Duty.CapVal[2];
    high_time = Encoder_dev->Encoder_Duty.CapVal[1];
    // printf("period: %d, high_time: %d\n", period, high_time);

    
    /* Store the values */
    Encoder_dev->Encoder_Duty.Period = period;
    Encoder_dev->Encoder_Duty.HighTime = high_time;
    
    /* Calculate duty cycle with safety checks */
    if (period > 0)
    {
        float duty = (float)high_time / (float)period;
        /* Limit duty cycle to valid range */
        if (duty > 1.0f) duty = 1.0f;
        if (duty < 0.0f) duty = 0.0f;
        Encoder_dev->Encoder_Duty.Duty = duty;
    }
    else
    {
        /* Handle invalid period */
        Encoder_dev->Encoder_Duty.Duty = 0.0f;
    }
    // record last angle
    Encoder_dev->previous_raw_angle = Encoder_dev->raw_angle;

    /* Calculate angle */
    Encoder_dev->raw_angle = Encoder_dev->Encoder_Duty.Duty * 360.0f;

    Encoder_PWM_Start_ReadAngle(Encoder_dev);
}

/**
 * @brief Calibrates and filters the Encoder device.
 *
 * This function calculates the period, high time of the pulse, and duty cycle of the Encoder device.
 * It also applies a first-order Kalman filter to the raw angle.
 */
void Encoder_Calibrate_n_Filter(Device_encoder_t *Encoder_dev)
{
    // Encoder_Calculate_From_Capture(Encoder_dev);
    Encoder_SPI_ReadAngle_WithWait(Encoder_dev);

    Encoder_UpdateContinuousAngle(Encoder_dev);
    
    /* Apply Kalman filter */
    // Encoder_dev->filtered_angle = FirstOrder_KalmanFilter_Update(&Encoder_dev->Encoder_KF, Encoder_dev->raw_continuous_angle);
    // Encoder_dev->filtered_angle = moving_average_filter(&Encoder_dev->movingAverage, Encoder_dev->raw_continuous_angle);
    Encoder_dev->filtered_angle = SimpleLowPassFilter_Update(&Encoder_dev->lowPassFilter, Encoder_dev->raw_continuous_angle);
    // convert to radians
    Encoder_dev->PreviousEncoderValRad = Encoder_dev->CurrentEncoderValRad;
    Encoder_dev->CurrentEncoderValRad = Encoder_dev->filtered_angle * ENCODER_PI / 180.0f;


    CalAngularVelocity(Encoder_dev);
    Encoder_dev->filtered_anguvel = BandPassFilter_Update(&Encoder_dev->bandPassFilter, Encoder_dev->AngularVelocity);
}

// Encoder parameters
#define ENCODER_RESOLUTION 4096 // Encoder resolution (pulses per revolution)
#define SAMPLE_INTERVAL_MS 10   // Sampling interval (milliseconds)
#define PI 3.14159f

/**
 * @brief Calculate the angular velocity of the encoder (rad/s)
 *
 * This function computes the angular velocity (speed of rotation) of the encoder in radians per second.
 * It uses the difference between the current and previous encoder positions (in radians) and the time interval between samples (in nanoseconds).
 * The function also handles the angle wrap-around (jump) at 0/360 degrees (i.e., 0/2π radians) to avoid spikes in the calculated speed.
 *
 * @param Encoder_dev Pointer to the encoder device structure (Device_encoder_t*), which must contain:
 *   - CurrentEncoderValRad: Current filtered encoder angle in radians
 *   - PreviousEncoderValRad: Previous filtered encoder angle in radians
 *   - CurrentSampleTime: Current sample timestamp in nanoseconds
 *   - PreviousSampleTime: Previous sample timestamp in nanoseconds
 *   - AngularVelocity: Output, calculated angular velocity (rad/s)
 *   - real_freq: Output, actual sampling frequency (Hz)
 *
 * @note
 *   - The encoder angle values must be in radians.
 *   - The time values must be in nanoseconds (ns).
 *   - This function automatically handles angle wrap-around (e.g., from 359° to 0°) to prevent speed spikes.
 *   - If the time interval is zero or negative, the output will be set to zero to avoid division by zero.
 *
 * Example usage:
 *   CalAngularVelocity(&encoder);
 */
void CalAngularVelocity(Device_encoder_t *Encoder_dev){
    uint64_t deltaTime_ns=Encoder_dev->CurrentSampleTime-Encoder_dev->PreviousSampleTime;
    float deltaEncoderValue=Encoder_dev->CurrentEncoderValRad-Encoder_dev->PreviousEncoderValRad;
    // 使用弧度单位处理跳变
    // float encoder_2pi = 2.0f * ENCODER_PI;
    // if(deltaEncoderValue > encoder_2pi / 2.0f){
    //     deltaEncoderValue -= encoder_2pi;
    // }
    // else if(deltaEncoderValue < -encoder_2pi / 2.0f){
    //     deltaEncoderValue += encoder_2pi;
    // }
    //calculate the angular velocity
    if (deltaTime_ns > 0) {
        // 角速度 = 角度变化 / 时间变化（单位：rad/s）
        Encoder_dev->AngularVelocity = deltaEncoderValue / ((float)deltaTime_ns * 1e-9f);
        Encoder_dev->real_freq = 1/(deltaTime_ns  * 1e-9f);
    } else {
        Encoder_dev->AngularVelocity = 0.0f;
        Encoder_dev->real_freq = 0.0f;
    }
}

/**
 * @brief Get encoder position
 * @param Encoder_dev Encoder device structure pointer
 * @return Current encoder position (degrees)
 * 
 * @note Returns the filtered angle value
 */
float Encoder_GetPos(Device_encoder_t *Encoder_dev)
{
    return Encoder_dev->filtered_angle;
}


