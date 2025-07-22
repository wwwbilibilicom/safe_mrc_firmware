/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __DRV_LED_H__
#define __DRV_LED_H__
#include "main.h"

typedef enum
{
    HIGH_LEVEL,
    LOW_LEVEL
}LED_OPEN_LEVEL;


#pragma  pack(1)
typedef struct
{
    const char * name;
    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
    LED_OPEN_LEVEL open_level;
}device_led_t;
#pragma  pack()

/**
 * @brief Initialize LED device
 * @param led_dev LED device structure pointer
 * @param name LED device name
 * @param port GPIO port
 * @param pin GPIO pin number
 * @param open_level LED turn-on level (HIGH_LEVEL or LOW_LEVEL)
 * 
 * @note This function will configure LED GPIO parameters and turn-on level
 */
void drv_led_init(device_led_t * led_dev,const char * name, GPIO_TypeDef * port, uint16_t pin, LED_OPEN_LEVEL open_level);

/**
 * @brief Turn on LED
 * @param led_dev LED device structure pointer
 * 
 * @note Set corresponding GPIO level according to LED's open_level
 */
void led_on(device_led_t * led_dev);

/**
 * @brief Turn off LED
 * @param led_dev LED device structure pointer
 * 
 * @note Set corresponding GPIO level according to LED's open_level
 */
void led_off(device_led_t * led_dev);

/**
 * @brief Toggle LED state
 * @param led_dev LED device structure pointer
 * 
 * @note Switch LED from current state to opposite state
 */
void led_toggle(device_led_t * led_dev);

#endif // __DRV_LED_H__
