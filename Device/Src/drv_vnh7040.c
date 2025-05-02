#include "drv_vnh7040.h"
#include "stdio.h"

#define PWM_FREQ 20000
#define SUPPLY_VOLTAGE 12.0f // Supply voltage
#define ADC_OVER_SAMPLING_RATIO 16 // ADC oversampling ratio
#define COIL_RESISTANCE 3.2f // Coil resistance in ohms
uint16_t prescaler = 9 - 1;       // Set the prescaler
uint64_t tim_clk_freq = 72000000; // Set the timer clock frequency

void drv_VNH7040_init(  uint8_t const *dev_name, Device_VNH7040_t *device, GPIO_TypeDef *INA, uint16_t INA_PIN,
                GPIO_TypeDef *INB, uint16_t INB_PIN, TIM_HandleTypeDef *PWM_tim, uint32_t PWM_channel, ADC_HandleTypeDef *ADC_handle)
{
    device->PWM_tim = PWM_tim; // PWM timer handle
    device->PWM_channel = PWM_channel;         // PWM channel
    device->INA = INA;         // INA GPIO
    device->INA_PIN = INA_PIN; // INA PIN

    device->INB = INB;         // INB GPIO
    device->INB_PIN = INB_PIN; // INB PIN

    device->ADC_handle = ADC_handle; // ADC handle

    HAL_TIM_PWM_Start(PWM_tim, PWM_channel);
    Set_PWM_Param(PWM_tim, PWM_channel, PWM_FREQ, 0);
    /* OUTA enable OUTB disable*/
    HAL_GPIO_WritePin(INA, INA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(INB, INB_PIN, GPIO_PIN_RESET);

    HAL_ADCEx_Calibration_Start(ADC_handle, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(ADC_handle,(uint32_t *)&device->ADC_BUFFER,1);

    printf("device VNH7040(%s) init success!\n", dev_name);
}


void Set_PWM_Param(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t freq, float duty)
{
    float pwm_freq_arr = (tim_clk_freq * 1.0f) / (prescaler + 1) / freq * 1.0f - 1; // Calculate PWM frequency
    float pwm_duty_pulse = duty * 1.0f / 100.0f * (pwm_freq_arr + 1);                 // Calculate PWM duty cycle pulse

    __HAL_TIM_SET_PRESCALER(htim, prescaler);                      // Set the timer prescaler
    __HAL_TIM_SetAutoreload(htim, (uint16_t)pwm_freq_arr);         // Set the timer auto-reload value
    __HAL_TIM_SetCompare(htim, Channel, (uint16_t)pwm_duty_pulse); // Set the timer compare value
}

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

void VNH7070_Multisense_ADC_process(Device_VNH7040_t *device)
{
    device->actual_voltage = COIL_RESISTANCE*(float)device->ADC_BUFFER * 3.3f / (4095 * ADC_OVER_SAMPLING_RATIO);
}
