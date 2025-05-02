#include "stdio.h"
#include "usart.h"
#include "drv_encoder.h"
#include "tim.h"
#include "stdint.h"
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
    FirstOrder_KalmanFilter_Init(&(Encoder_dev->Encoder_KF), 0.001f, 0.133f);
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
#ifdef FREERTOS
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif
    switch (Encoder_dev->Encoder_Duty.CapIndex)
    {
    case First_Rrising:
        Encoder_dev->Encoder_Duty.CapFlag = 0;
        Encoder_dev->Encoder_Duty.CapVal[0] = HAL_TIM_ReadCapturedValue(Encoder_dev->htim, Encoder_dev->Channel);
        // printf("Capture CapVal0 is :%d\n",Encoder_dev->Encoder_Duty.CapVal[0]);
        Encoder_dev->Encoder_Duty.CapIndex = Falling;
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
#ifdef FREERTOS
        xSemaphoreGiveFromISR(Encoder_dev->Encoder_Duty.xEncoderGetPosSemap, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
#endif
        break;

    default:
        Error_Handler();
        break;
    }
}

/**
 * @brief Get the position information of the Encoder module.
 *
 * This function determines the position of the Encoder module by calculating its duty cycle. The duty cycle is calculated by measuring the high time of the pulse and its period.
 * If the CapFlag flag is set, indicating that new position data is available, the function will update the duty cycle calculation and start a timer to obtain the next position update.
 *
 * @return The position of the Encoder module in degrees.
 */
__attribute__((weak)) float Encoder_GetPos(Device_encoder_t *Encoder_dev)
{
    /* Check if new position data is available */

    if (Encoder_dev->Encoder_Duty.CapFlag)
    {
        /* Clear the new data flag */
        Encoder_dev->Encoder_Duty.CapFlag = 0;

        /* Calculate the period */

        if (Encoder_dev->Encoder_Duty.CapVal[2] > Encoder_dev->Encoder_Duty.CapVal[0])
        {
            Encoder_dev->Encoder_Duty.Period = Encoder_dev->Encoder_Duty.CapVal[2] - Encoder_dev->Encoder_Duty.CapVal[0];
        }
        else
        {
            Encoder_dev->Encoder_Duty.Period = Encoder_dev->Encoder_Duty.CapVal[2] + 0xffff + 1 - Encoder_dev->Encoder_Duty.CapVal[0];
        }

        /* Calculate the high time of the pulse */
        if (Encoder_dev->Encoder_Duty.CapVal[1] >= Encoder_dev->Encoder_Duty.CapVal[0])
        {
            Encoder_dev->Encoder_Duty.HighTime = Encoder_dev->Encoder_Duty.CapVal[1] - Encoder_dev->Encoder_Duty.CapVal[0];
        }
        else
        {
            Encoder_dev->Encoder_Duty.HighTime = 0xFFFF + 1 - Encoder_dev->Encoder_Duty.CapVal[0] + Encoder_dev->Encoder_Duty.CapVal[1];
        }

        /* Calculate the duty cycle */

        Encoder_dev->Encoder_Duty.Duty = (float)Encoder_dev->Encoder_Duty.HighTime / (float)Encoder_dev->Encoder_Duty.Period;
        Encoder_dev->raw_angle = (float)(Encoder_dev->Encoder_Duty.Duty * 360.0f);
        Encoder_dev->filtered_angle = FirstOrder_KalmanFilter_Update(&Encoder_dev->Encoder_KF, Encoder_dev->raw_angle);
        /* Start a timer to capture the next position update */
        // HAL_TIM_IC_Start_IT(Encoder_dev->htim, Encoder_dev->Channel);
        /* Return the position information in degrees */

        // return Encoder_dev->filtered_angle;
        return Encoder_dev->raw_angle;
    }

    // return Encoder_dev->filtered_angle;
    return Encoder_dev->raw_angle;
}



/**
 * @brief Calibrates and filters the Encoder device.
 *
 * This function calculates the period, high time of the pulse, and duty cycle of the Encoder device.
 * It also applies a first-order Kalman filter to the raw angle.
 */
void Encoder_Calibrate_n_Filter(Device_encoder_t *Encoder_dev)
{
    /* Calculate the period */

    if (Encoder_dev->Encoder_Duty.CapVal[2] > Encoder_dev->Encoder_Duty.CapVal[0])
    {
        Encoder_dev->Encoder_Duty.Period = Encoder_dev->Encoder_Duty.CapVal[2] - Encoder_dev->Encoder_Duty.CapVal[0];
    }
    else
    {
        Encoder_dev->Encoder_Duty.Period = Encoder_dev->Encoder_Duty.CapVal[2] + 0xffff + 1 - Encoder_dev->Encoder_Duty.CapVal[0];
    }

    /* Calculate the high time of the pulse */
    if (Encoder_dev->Encoder_Duty.CapVal[1] >= Encoder_dev->Encoder_Duty.CapVal[0])
    {
        Encoder_dev->Encoder_Duty.HighTime = Encoder_dev->Encoder_Duty.CapVal[1] - Encoder_dev->Encoder_Duty.CapVal[0];
    }
    else
    {
        Encoder_dev->Encoder_Duty.HighTime = 0xFFFF + 1 - Encoder_dev->Encoder_Duty.CapVal[0] + Encoder_dev->Encoder_Duty.CapVal[1];
    }

    /* Calculate the duty cycle */

    Encoder_dev->Encoder_Duty.Duty = (float)Encoder_dev->Encoder_Duty.HighTime / (float)Encoder_dev->Encoder_Duty.Period;
    Encoder_dev->raw_angle = (float)(Encoder_dev->Encoder_Duty.Duty * 360.0f);
    Encoder_dev->filtered_angle = FirstOrder_KalmanFilter_Update(&Encoder_dev->Encoder_KF, Encoder_dev->raw_angle);

    HAL_TIM_IC_Start_IT(Encoder_dev->htim, Encoder_dev->Channel);
}

//编码器参数
#define ENCODER_RESOLUTION 4096 //编码器分辨率(每圈脉冲数)
#define SAMPLE_INTERVAL_MS 10   //采样间隔(毫秒)
#define PI 3.14159f

//获取当前时间
uint32_t getCurrentTime(){
    return HAL_GetTick();
}

//calculate angular velocity
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


