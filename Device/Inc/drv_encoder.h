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
#include "sys_clock.h"
#include "kth78xx.h"

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

        float real_freq;

        PWM_DUTY_Capture Encoder_Duty;
        FirstOrderKalmanFilter Encoder_KF;
        float raw_angle;  // Encoder unprocessed angle, unit: degrees
        float previous_raw_angle; // Encoder unprocessed angle, unit: degrees
        float zero_offset;      // initial angle in degrees
        int32_t round_count;    // round count
        float raw_continuous_angle; // raw continuous angle unit: degrees
        float filtered_angle;  // Encoder filtered angle, unit: degrees
        
        float CurrentEncoderValRad;  // Current encoder value in rad
        float PreviousEncoderValRad;// Previous encoder value in rad
        uint64_t CurrentSampleTime;
        uint64_t PreviousSampleTime;
        float AngularVelocity; // Angular velocity, rad/s
        float filtered_anguvel;

        BandPassFilter bandPassFilter;
        SimpleLowPassFilter lowPassFilter;
        movingAverage_t movingAverage;
        
        uint8_t spi_databack[2];

    } Device_encoder_t;
#pragma pack()

    /**
     * @brief Initialize encoder device
     * @param Encoder_dev Encoder device structure pointer
     * @param Encoder_tim Timer handle pointer
     * @param Encoder_ch Encoder channel number
     * @return 0: success, -1: failure
     * 
     * @note This function will initialize encoder timer, filter and related parameters
     */
    int drv_encoder_init(Device_encoder_t *Encoder_dev, TIM_HandleTypeDef *Encoder_tim, uint32_t Encoder_ch);

    void Encoder_PWM_CalibrateZero(Device_encoder_t *Encoder_dev);

    static void Encoder_SPI_CalibrateZero(Device_encoder_t *Encoder_dev);

    
    /**
     * @brief Get encoder position
     * @param Encoder_dev Encoder device structure pointer
     * @return Current encoder position (degrees)
     * 
     * @note Returns filtered angle value
     */
    float Encoder_GetPos(Device_encoder_t *Encoder_dev);
    
    static void Encoder_Calculate_From_Capture(Device_encoder_t *Encoder_dev);

    static void Encoder_UpdateContiuousAngle(Device_encoder_t *Encoder_dev);

    /**
     * @brief Calibrate and filter encoder
     * @param Encoder_dev Encoder device structure pointer
     * 
     * @note This function will calibrate encoder initial angle and apply filter
     */
    void Encoder_Calibrate_n_Filter(Device_encoder_t *Encoder_dev);
    
    /**
     * @brief Capture PWM duty cycle
     * @param Encoder_dev Encoder device structure pointer
     * 
     * @note Used to capture PWM signal duty cycle from encoder output
     */
    void CaptureDutyCycle(Device_encoder_t *Encoder_dev);
    
    /**
     * @brief Calculate angular velocity
     * @param Encoder_dev Encoder device structure pointer
     * 
     * @note Calculate angular velocity based on encoder position change, unit: rad/s
     */
    void CalAngularVelocity(Device_encoder_t *Encoder_dev);
    
    /**
     * @brief Read encoder angle via SPI
     * @param Encoder_dev Encoder device structure pointer
     * 
     * @note Suitable for encoder chips that support SPI communication, such as KTH78xx series
     */
    void Encoder_SPI_ReadAngle_WithWait(Device_encoder_t *Encoder_dev);

    void Encoder_PWM_Start_ReadAngle(Device_encoder_t *Encoder_dev);

    void Encoder_SPI_ReadAngle_WithoutWait(Device_encoder_t *Encoder_dev);

    void Encoder_SPI_ExchangeData(Device_encoder_t *Encoder_dev);

//    uint32_t getCurrentTime(void);
//    uint64_t getHighResTime_ns(void);

    // extern Device_encoder_t Encoder_dev;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DRV_ENCODER_H__ */
