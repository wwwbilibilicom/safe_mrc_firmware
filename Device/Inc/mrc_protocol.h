#ifndef __MRC_PROTOCOL_H__
#define __MRC_PROTOCOL_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef enum MRC_Mode
{
    FREE,
    FIX_LIMIT,
    ADAPTATION
} MRC_Mode;

#pragma pack(1)
    typedef struct MRC_Cmd_Protocol     //message buffer typedef for the command protocol
    {
        uint8_t head[2];                // 2 bytes 0xFE, 0xEE
        uint8_t id;                     // 1 bytes
        MRC_Mode mode;                  // 1 bytes
        uint16_t des_torque;            // 2 bytes
        uint16_t CRC16Data;             // 2 bytes
    } MRC_Cmd_Protocol;                 // 8 bytes 
#pragma pack()

#pragma pack(1)
    typedef struct MRC_Fbk_Protocol //message buffer typedef for the feedback protocol
    {
        uint8_t head[2];            // 2 bytes
        uint8_t id;                 // 1 bytes
        MRC_Mode mode;              // 1 bytes
        uint8_t collision_flag;             // 1 bytes 0x00: safely, 0x01: collision happened.
        uint32_t encoder_value;     // 4 bytes
        uint16_t present_torque;    // 2 bytes
        uint16_t CRC16Data;         // 2 bytes
    } MRC_Fbk_Protocol;             // 13 bytes
#pragma pack()

#ifdef __cplusplus
}
#endif

#endif /* __MRC_PROTOCOL_H__ */
