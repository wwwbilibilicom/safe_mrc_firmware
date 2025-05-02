//
// Created by ustc on 24-9-20.
//

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

void drv_led_init(device_led_t * led_dev,const char * name, GPIO_TypeDef * port, uint16_t pin, LED_OPEN_LEVEL open_level);
void led_on(device_led_t * led_dev);
void led_off(device_led_t * led_dev);
void led_toggle(device_led_t * led_dev);

#endif // __DRV_LED_H__
