#include "stdio.h"
#include "usart.h"
#include "drv_encoder.h"
#include "tim.h"
#include "stdint.h"
#include "kth78xx.h"
#ifdef FREERTOS
#include "cmsis_os.h"
#endif

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
    HAL_TIM_IC_Start_IT(Encoder_dev->htim, Encoder_dev->Channel); // Start the timer interrupt for input capture
    FirstOrder_KalmanFilter_Init(&(Encoder_dev->Encoder_KF), 0.001f, 0.033f);
    // moving_average_create(&(Encoder_dev->movingAverage), 100, 1);
    BandPassFilter_Init(30.0f, 150.0f, 1000.0f, &(Encoder_dev->bandPassFilter));
    Encoder_dev->Encoder_Duty.CapIndex = First_Rrising;
    Encoder_dev->Encoder_Duty.CapFlag = 0;

    printf("Encoder init success!\n");
    return 1;
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
void Encoder_SPI_ReadAngle(Device_encoder_t *Encoder_dev){
    Encoder_dev->raw_angle = KTH78_ReadAngle();
}

/**
 * @brief Calibrates and filters the Encoder device.
 *
 * This function calculates the period, high time of the pulse, and duty cycle of the Encoder device.
 * It also applies a first-order Kalman filter to the raw angle.
 */
void Encoder_Calibrate_n_Filter(Device_encoder_t *Encoder_dev)
{
    uint32_t period, high_time;

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
    
    /* Calculate angle */
    Encoder_dev->raw_angle = Encoder_dev->Encoder_Duty.Duty * 360.0f;
    
    /* Apply Kalman filter */
    Encoder_dev->filtered_angle = FirstOrder_KalmanFilter_Update(&Encoder_dev->Encoder_KF, Encoder_dev->raw_angle);
    // Encoder_dev->filtered_angle = moving_average_filter(&Encoder_dev->movingAverage, Encoder_dev->raw_angle);
    
    /* Restart capture */
    HAL_TIM_IC_Start_IT(Encoder_dev->htim, Encoder_dev->Channel);
}

// Encoder parameters
#define ENCODER_RESOLUTION 4096 // Encoder resolution (pulses per revolution)
#define SAMPLE_INTERVAL_MS 10   // Sampling interval (milliseconds)
#define PI 3.14159f

/**
 * @brief Get current time
 * @return Current system time (milliseconds)
 * 
 * @note Use HAL_GetTick() to get system running time
 */
uint32_t getCurrentTime(){
    return HAL_GetTick();
}

/**
 * @brief Calculate angular velocity
 * @param Encoder_dev Encoder device structure pointer
 * 
 * @note Calculate angular velocity based on encoder position change, unit is rad/s
 * @note Use differential method to calculate angular velocity, including angle jump handling
 * @note Sampling interval is 10ms to ensure calculation accuracy
 */
void CalAngularVelocity(Device_encoder_t *Encoder_dev){
    Encoder_dev->CurrentTime=getCurrentTime();
    Encoder_dev->CurrentEncoderValRad=(Encoder_dev->filtered_angle)*PI/180;

    uint32_t deltaTime=Encoder_dev->CurrentTime-Encoder_dev->PreviousTime;

    if(deltaTime>=SAMPLE_INTERVAL_MS){

        float deltaEncoderValue=Encoder_dev->CurrentEncoderValRad-Encoder_dev->PreviousEncoderValRad;
        if(deltaEncoderValue>ENCODER_RESOLUTION/2){
            deltaEncoderValue-=ENCODER_RESOLUTION;
        }
        else if(deltaEncoderValue<-ENCODER_RESOLUTION/2){
            deltaEncoderValue+=ENCODER_RESOLUTION;
        }
        //calculate the angular velocity
        Encoder_dev->AngularVelocity=(float)deltaEncoderValue/ENCODER_RESOLUTION*360.0f/(deltaTime/1000.0f);
        Encoder_dev->PreviousEncoderValRad=Encoder_dev->CurrentEncoderValRad;
        Encoder_dev->PreviousTime=Encoder_dev->CurrentTime;
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


