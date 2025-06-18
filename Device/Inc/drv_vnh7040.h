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
 * @note This function will initialize VNH7040 GPIO, PWM and ADC configuration
 */
void drv_VNH7040_init(const uint8_t *dev_name, Device_VNH7040_t *device,
                  GPIO_TypeDef *INA, uint16_t INA_PIN,
                  GPIO_TypeDef *INB, uint16_t INB_PIN,
                  TIM_HandleTypeDef *PWM_tim, uint32_t PWM_channel, 
                  ADC_HandleTypeDef *ADC_handle);

/**
 * @brief Set PWM parameters
 * @param htim Timer handle pointer
 * @param Channel PWM channel number
 * @param freq PWM frequency (Hz)
 * @param duty Duty cycle (0.0-100.0)
 * 
 * @note This function will configure timer PWM frequency and duty cycle
 */
void Set_PWM_Param(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t freq, float duty);

/**
 * @brief VNH7040 multi-sensor ADC processing
 * @param device VNH7040 device structure pointer
 * 
 * @note This function will read ADC value and update actual voltage value
 */
void VNH7070_Multisense_ADC_process(Device_VNH7040_t *device);

/**
 * @brief Set VNH7040 output voltage
 * @param device VNH7040 device structure pointer
 * @param voltage Target voltage value
 * 
 * @note This function will set PWM duty cycle according to target voltage
 */
void VNH7040_Set_Voltage(Device_VNH7040_t *device, float voltage);

#ifdef __cplusplus
}
#endif

#endif
