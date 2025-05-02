#ifndef __MRC_COM_H__
#define __MRC_COM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "mrc_protocol.h"


#pragma pack(1)
    typedef struct MRC_Com_t
    {
        uint8_t id;

        MRC_Cmd_Protocol cmd_msg; // 8 bytes
        uint8_t cmd_msg_buffer[sizeof(MRC_Cmd_Protocol)]; // 8 bytes
        MRC_Fbk_Protocol fbk_msg;

        UART_HandleTypeDef *mrc_huart;

        uint8_t RxFlag;
        uint16_t RxLen; // Length of the received data
        uint16_t TxLen; // Length of the received data

        uint8_t cmd_correct; // 0: error, 1: correct
    } MRC_Com_t;
#pragma pack()

#ifdef __cplusplus
}
#endif

#endif

