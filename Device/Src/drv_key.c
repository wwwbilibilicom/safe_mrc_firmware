//
// Created by ustc on 24-9-20.
//

#include "drv_key.h"
#include "stdio.h"



/**
 * @brief Initialize a key device with specific parameters.
 *
 * This function initializes a key device with a given name and GPIO port and pin.
 * It sets the initial state of the key to KEY_UP.
 *
 * @param dev_key Pointer to the key device structure to be initialized.
 * @param name Name of the key device.
 * @param GPIO_Port GPIO port to which the key is connected.
 * @param GPIO_Pin GPIO pin number to which the key is connected.
 */
void drv_key_init(device_key_t * dev_key, const char * name, GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin)
{
    dev_key->state = KEY_UP;
    dev_key->name = name;
    dev_key->GPIO_Port = GPIO_Port;
    dev_key->GPIO_Pin = GPIO_Pin;
    
    printf("key init ok\r\n");
}

/**
 * @brief Scan the state of the key device.
 * @usage: put this function in the timer interrupt or main loop.
 * This function scans the state of the key device and updates its state based on the GPIO pin state.
 * It handles key bounce and sets the key flag when the key is pressed.
 * if you want to use this function, you must call it in the main loop.
 *
 * @param dev_key Pointer to the key device structure to be scanned.
 */
void Keyscan(device_key_t * dev_key)
{
    switch(dev_key->state)
    {
        case KEY_UP:
        {
            if(HAL_GPIO_ReadPin(dev_key->GPIO_Port,dev_key->GPIO_Pin) == GPIO_PIN_RESET)
            {
                dev_key->state = KEY_BEBOUNCE;
            }
            break;
        }
        case KEY_BEBOUNCE:
        {
            if(HAL_GPIO_ReadPin(dev_key->GPIO_Port,dev_key->GPIO_Pin) == GPIO_PIN_RESET)
            {
                dev_key->state = KEY_WAIT_RELEASE;
            }
            else
            {
                dev_key->state = KEY_UP;
            }
            break;
        }
        case KEY_WAIT_RELEASE:
        {
            if(HAL_GPIO_ReadPin(dev_key->GPIO_Port,dev_key->GPIO_Pin) == GPIO_PIN_SET)
            {
                dev_key->state = KEY_UP;
                dev_key->key_flag = 1;
            }
            break;

        }
        default:break;
    }
}
