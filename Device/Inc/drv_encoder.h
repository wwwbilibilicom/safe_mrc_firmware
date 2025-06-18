#ifndef __DRV_ENCODER_H__
#define __DRV_ENCODER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "filter.h"
#include "stdint.h"
#ifdef FREERTOS
#include "cmsis_os.h"
#endif
#include <stm32h7xx_hal_tim.h>

    typedef enum
    {
        First_Rrising,
        Falling,
        Second_Rising
    } Capture_Process_State;

#pragma pack(1)
    typedef struct
    {
        /* data */
        uint32_t CapVal[3];

        Capture_Process_State CapIndex;

        uint8_t CapFlag;
        uint32_t Period;
        uint32_t HighTime;
        float Duty;
#ifdef FREERTOS
        SemaphoreHandle_t xEncoderGetPosSemap;
#endif
    } PWM_DUTY_Capture;
#pragma pack()

#pragma pack(1)

    typedef struct
    {
        TIM_HandleTypeDef *htim;
        uint32_t Channel;

        PWM_DUTY_Capture Encoder_Duty;
        FirstOrderKalmanFilter Encoder_KF;
        movingAverage_t movingAverage;
        float raw_angle;  // 编码器未处理角度，单位：度
        float filtered_angle;  // 编码器滤波后角度，单位：度
        float initial_angle;  // 编码器初始角度，单位：度
        float diff_angle;  // 编码器角度差，单位：度

        float CurrentEncoderValRad;  // 当前编码器值
        float PreviousEncoderValRad;// 上一次编码器值
        uint32_t CurrentTime;
        uint32_t PreviousTime;
        float AngularVelocity; // 角速度，rad/s
        float filtered_anguvel;
        BandPassFilter bandPassFilter;

    } Device_encoder_t;
#pragma pack()

    int drv_encoder_init(Device_encoder_t *Encoder_dev, TIM_HandleTypeDef *Encoder_tim, uint32_t Encoder_ch);
    float Encoder_GetPos(Device_encoder_t *Encoder_dev);
    void Encoder_Calibrate_n_Filter(Device_encoder_t *Encoder_dev);
    void CaptureDutyCycle(Device_encoder_t *Encoder_dev);
    void CalAngularVelocity(Device_encoder_t *Encoder_dev);
    void Encoder_SPI_ReadAngle(Device_encoder_t *Encoder_dev);

    // extern Device_encoder_t Encoder_dev;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DRV_ENCODER_H__ */
