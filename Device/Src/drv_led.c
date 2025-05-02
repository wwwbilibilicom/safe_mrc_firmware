//
// Created by ustc on 24-9-20.
//

#include "drv_led.h"
#include "stdio.h"

/**
 * @brief Initialize a LED device with specific parameters.
 *
 * This function initializes a LED device with a given name, port, pin, and open level.
 * It sets the GPIO pin state based on the open level.
 *
 * @param led_dev Pointer to the LED device structure to be initialized.
 * @param name Name of the LED device.
 * @param port GPIO port to which the LED is connected.
 * @param pin GPIO pin number to which the LED is connected.
 * @param open_level Open level of the LED (HIGH_LEVEL or LOW_LEVEL).
 */
void drv_led_init(device_led_t * led_dev,const char * name, GPIO_TypeDef * port, uint16_t pin, LED_OPEN_LEVEL open_level)
{
    led_dev->name = name;
    led_dev->GPIO_Port = port;
    led_dev->GPIO_Pin = pin;
    led_dev->open_level = open_level;
    if(led_dev->open_level == HIGH_LEVEL)
    {
        HAL_GPIO_WritePin(led_dev->GPIO_Port, led_dev->GPIO_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(led_dev->GPIO_Port, led_dev->GPIO_Pin, GPIO_PIN_SET);
    }
    printf("LED %s initialized.\n", led_dev->name);
}


/**
 * @brief Turn on the LED device.
 *
 * This function turns on the LED device by setting the GPIO pin state based on the open level.
 *
 * @param led_dev Pointer to the LED device structure to be turned on.
 */
void led_on(device_led_t * led_dev)
{
    if(led_dev->open_level == HIGH_LEVEL)
    {
        HAL_GPIO_WritePin(led_dev->GPIO_Port, led_dev->GPIO_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(led_dev->GPIO_Port, led_dev->GPIO_Pin, GPIO_PIN_RESET);
    }
}

/**
 * @brief Toggle the state of the LED device.
 *
 * This function toggles the state of the LED device by inverting the GPIO pin state.
 *
 * @param led_dev Pointer to the LED device structure to be toggled.
 */
void led_off(device_led_t * led_dev)
{
    if(led_dev->open_level == HIGH_LEVEL)
    {
        HAL_GPIO_WritePin(led_dev->GPIO_Port, led_dev->GPIO_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(led_dev->GPIO_Port, led_dev->GPIO_Pin, GPIO_PIN_SET);
    }
}

void led_toggle(device_led_t * led_dev)
{
    HAL_GPIO_TogglePin(led_dev->GPIO_Port, led_dev->GPIO_Pin);
}
