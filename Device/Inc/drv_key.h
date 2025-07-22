/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __DRV_KEY_H__
#define __DRV_KEY_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

typedef enum
{
    KEY_UP = 0,
    KEY_BEBOUNCE,
    KEY_WAIT_RELEASE,
}KEY_STATE;

#pragma pack(1)
typedef struct
{
    const char * name;
    KEY_STATE state;
    uint8_t key_flag;
    GPIO_TypeDef *GPIO_Port;
    uint16_t GPIO_Pin;
}device_key_t;
#pragma pack()
void drv_key_init(device_key_t * dev_key, const char * name,
                  GPIO_TypeDef * GPIO_Port, uint16_t GPIO_Pin);
void Keyscan(device_key_t * dev_key);


#ifdef __cplusplus
}
#endif


#endif // __DRV_KEY_H__

