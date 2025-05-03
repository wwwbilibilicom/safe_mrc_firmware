#ifndef __DRV_VNH7040_H__
#define __DRV_VNH7040_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "main.h"
#include "adc.h"

#pragma pack(1)
typedef struct Device_VNH7040_t
{
    const uint8_t *device_name;                                    // Pointer to the device name
    TIM_HandleTypeDef *PWM_tim;
    TIM_HandleTypeDef *loop_htim;
    uint32_t PWM_channel; // PWM
    GPIO_TypeDef *INA;
    uint32_t INA_PIN;
    ADC_HandleTypeDef *ADC_handle;
    GPIO_TypeDef *INB;
    uint32_t INB_PIN;
    float des_voltage; // Desired voltage of the device
    float actual_voltage; // Actual voltage of the device
} Device_VNH7040_t;
#pragma pack()

void drv_VNH7040_init(const uint8_t *dev_name, Device_VNH7040_t *device,
                  GPIO_TypeDef *INA, uint16_t INA_PIN,
                  GPIO_TypeDef *INB, uint16_t INB_PIN,
                  TIM_HandleTypeDef *PWM_tim, uint32_t PWM_channel, 
                  ADC_HandleTypeDef *ADC_handle);
void Set_PWM_Param(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t freq, float duty);
void VNH7070_Multisense_ADC_process(Device_VNH7040_t *device);
void VNH7040_Set_Voltage(Device_VNH7040_t *device, float voltage);

#ifdef __cplusplus
}
#endif

#endif
