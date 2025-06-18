//
// Created by wenbo on 24-9-7.
//

#ifndef __DRV_MRC_H__
#define __DRV_MRC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "tim.h"
#include "usart.h"
#include "adc.h"
#include "main.h"
#include "drv_led.h"
#include "drv_key.h"
#include "drv_encoder.h"
#include "mrc_com.h"
#include "drv_vnh7040.h"
#include "pid_controller.h"

    typedef enum
    {
        Disengagement,
        Engagement,
        Demagnetization,
        Magnetization

    } MRC_State;

#pragma pack(1)

    /**
     * @brief Structure representing the MR device.
     *
     * This structure holds various parameters and function pointers related to the MR device.
     */
    typedef struct Device_MRC_t
    {
        MRC_Mode mode; // mode of the MR device
        MRC_State state_phase; // state_phase of the MR device
        uint8_t collision_flag; // 1 bytes 0x00: safely, 0x01: collision happened.
        MRC_Com_t com; // communication structure for the MRC device
        float des_torque;

        device_led_t LED1;
        device_led_t LED2;
        device_key_t KEY1;
        device_key_t KEY2;
        Device_encoder_t Encoder;
        Device_VNH7040_t VNH7040; // VNH7040 device structure
        const uint8_t *device_name;                                    // Pointer to the device name

        PID_Controller coil_pid; // PID controller for the coil

        int (*LOG_MSG)(const char *format, ...); // Function pointer to log messages
        uint8_t State_Update_Flag;               // Flag to update the state_phase
        uint8_t demagnetization_counter;         // Counter for demagnetization
        uint8_t magnetization_counter;
        uint8_t COLLISION_REACT_FLAG;            // flag to execute collision reaction
        uint8_t control_loop_flag;               // flag to execute control loop

    } Device_MRC_t;
#pragma pack()

    /**
     * @brief Initialize MRC device
     * @param dev_name Device name
     * @param MRC MRC device structure pointer
     * @param id Device ID
     * 
     * @note This function will initialize all MRC sub-devices, including LED, key, encoder, VNH7040, etc.
     * @note Also initializes the MRC communication module with DMA support
     */
    void MRC_Init(const uint8_t *dev_name, Device_MRC_t *MRC, uint8_t id);
    
    /**
     * @brief Set MRC coil voltage
     * @param MRC MRC device structure pointer
     * 
     * @note Calculate and set coil voltage based on target torque and PID controller
     */
    void MRC_set_voltage(Device_MRC_t *MRC);
    
    /**
     * @brief Collision detection
     * @param MRC1 First MRC device structure pointer
     * @param MRC2 Second MRC device structure pointer
     * @param param Detection parameter
     * @param threshold Detection threshold
     * 
     * @note Detect collision by comparing states of two MRC devices
     */
    void MRC_collision_detect(Device_MRC_t *MRC1, Device_MRC_t *MRC2, float param, float threshold);
    
    /**
     * @brief MRC communication process using new mrc_com module
     * @param MRC MRC device structure pointer
     * 
     * @note Handle communication with host computer using DMA mode
     * @note Extract command and send feedback automatically
     */
    void MRC_Com_Process(Device_MRC_t *MRC);
    
    /**
     * @brief Handle KEY1 response
     * @param MRC MRC device structure pointer
     * 
     * @note Handle response logic when KEY1 is pressed
     */
    void MRC_Key1_Reaction(Device_MRC_t *MRC);
    
    /**
     * @brief Handle KEY2 response
     * @param MRC MRC device structure pointer
     * 
     * @note Handle response logic when KEY2 is pressed
     */
    void MRC_Key2_Reaction(Device_MRC_t *MRC);

#ifdef __cplusplus
    }
#endif

#endif
