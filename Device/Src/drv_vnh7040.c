/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "drv_vnh7040.h"
#include "stdio.h"

#define PWM_FREQ 20000
#define SUPPLY_VOLTAGE 12.0f // Supply voltage
#define ADC_OVER_SAMPLING_RATIO 16 // ADC oversampling ratio
#define COIL_RESISTANCE 3.2f // Coil resistance in ohms
uint16_t prescaler = 9 - 1;       // Set the prescaler
uint64_t tim_clk_freq = 72000000; // Set the timer clock frequency


/**
 * @brief Initialize VNH7040 motor driver chip
 * @param dev_name Device name
 * @param device VNH7040 device structure pointer
 * @param INA INA pin GPIO port
 * @param INA_PIN INA pin number
 * @param INB INB pin GPIO port
 * @param INB_PIN INB pin number
 * @param PWM_tim PWM timer handle pointer
 * @param PWM_channel PWM channel number
 * @param ADC_handle ADC handle pointer
 * 
 * @note This function will initialize VNH7040's GPIO, PWM and ADC configuration
 * @note Start PWM output and configure ADC calibration and DMA transmission
 */
void drv_VNH7040_init(const uint8_t  *dev_name, Device_VNH7040_t *device, GPIO_TypeDef *INA, uint16_t INA_PIN,
                GPIO_TypeDef *INB, uint16_t INB_PIN, TIM_HandleTypeDef *PWM_tim, uint32_t PWM_channel)
{
    device->PWM_tim = PWM_tim; // PWM timer handle
    device->PWM_channel = PWM_channel;         // PWM channel
    device->INA = INA;         // INA GPIO
    device->INA_PIN = INA_PIN; // INA PIN

    device->INB = INB;         // INB GPIO
    device->INB_PIN = INB_PIN; // INB PIN

    HAL_TIM_PWM_Start(PWM_tim, PWM_channel);
    Set_PWM_Param(PWM_tim, PWM_channel, PWM_FREQ, 0);
    /* OUTA enable OUTB disable*/
    HAL_GPIO_WritePin(INA, INA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(INB, INB_PIN, GPIO_PIN_RESET);

    printf("Device VNH7040(%s) initialized successfully!\n", dev_name);
}

/**
 * @brief Set PWM parameters
 * @param htim Timer handle pointer
 * @param Channel PWM channel number
 * @param freq PWM frequency (Hz)
 * @param duty Duty cycle (0.0-100.0)
 * 
 * @note This function will configure the timer's PWM frequency and duty cycle
 * @note Calculate PWM parameters based on timer clock frequency and prescaler
 */
void Set_PWM_Param(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t freq, float duty)
{
    float pwm_freq_arr = (tim_clk_freq * 1.0f) / (prescaler + 1) / freq * 1.0f - 1; // Calculate PWM frequency
    float pwm_duty_pulse = duty * 1.0f / 100.0f * (pwm_freq_arr + 1);                 // Calculate PWM duty cycle pulse

    __HAL_TIM_SET_PRESCALER(htim, prescaler);                      // Set the timer prescaler
    __HAL_TIM_SetAutoreload(htim, (uint16_t)pwm_freq_arr);         // Set the timer auto-reload value
    __HAL_TIM_SetCompare(htim, Channel, (uint16_t)pwm_duty_pulse); // Set the timer compare value
}

/**
 * @brief Set VNH7040 output voltage
 * @param device VNH7040 device structure pointer
 * @param voltage Target voltage value (-12V to +12V)
 * 
 * @note This function will set PWM duty cycle based on target voltage
 * @note Positive voltage enables OUTA, negative voltage enables OUTB
 * @note Voltage will be limited within the power supply voltage range
 */
void VNH7040_Set_Voltage(Device_VNH7040_t *device, float voltage)
{
    float duty;
    if(voltage > SUPPLY_VOLTAGE)
    {
        voltage = SUPPLY_VOLTAGE;
    }
    if (voltage < -SUPPLY_VOLTAGE)
    {
        voltage = -SUPPLY_VOLTAGE;
    }
    
    if (voltage >= 0.0f)
    {
        duty = (float)((voltage / SUPPLY_VOLTAGE) * 100);

        Set_PWM_Param(device->PWM_tim, device->PWM_channel, PWM_FREQ, duty);
        /* OUTA enable OUTB disable*/
        HAL_GPIO_WritePin(device->INA, device->INA_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(device->INB, device->INB_PIN, GPIO_PIN_RESET);
    }
    else if (voltage < 0.0f)
    {
        duty = (float)(-(voltage / SUPPLY_VOLTAGE) * 100);

        Set_PWM_Param(device->PWM_tim, device->PWM_channel, PWM_FREQ, duty);
        /* OUTA disable OUTB enable*/
        HAL_GPIO_WritePin(device->INA, device->INA_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(device->INB, device->INB_PIN, GPIO_PIN_SET);
    }
}


