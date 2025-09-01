/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

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
#include "filter.h"
#include "mrc_statemachine.h"

// MRC control mode enumeration
typedef enum {
    MRC_VOLTAGE_CONTROL = 0,  // Voltage control mode
    MRC_CURRENT_CONTROL = 1   // Coil current control mode
} MRC_ControlMode;

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
        MRC_State state_phase; // state_phase of the MR device

        float collision_threshold; // collision_threshold of the MR device
        uint8_t COLLISION_REACT_FLAG;            // flag to execute collision reaction
        
        MRC_Com_t com; // communication structure for the MRC device

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
        uint8_t control_loop_flag;               // flag to execute control loop

        ADC_HandleTypeDef *ADC_handle; // ADC handle for the MRC device
        float coil_resistance; // Coil resistance for the MRC device
        float coil_inductance; // Coil inductance for the MRC device
        float coil_sample_time; // Coil sample time for the MRC device
        uint8_t coil_current_update_flag; // flag to update the coil current
        float actual_coil_current; // Actual coil current for the MRC device
        float filtered_coil_current; // Filtered coil current for the MRC device
        float des_coil_current; // Desired coil current for the MRC device
        SimpleLowPassFilter coil_current_filter; // Coil current filter
        MRC_ControlMode control_mode; // Control mode: voltage or current (default: voltage)
        MRC_StateMachine_t statemachine; // MRC state machine instance for mode management and safety

        uint16_t print_count;
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
     * @param MRC MRC device structure pointer
     * @param param Detection parameter
     * @param threshold Detection threshold
     * 
     * @note Detect collision by comparing states of the MRC device
     */
    void MRC_collision_detect(Device_MRC_t *MRC);
    
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
     * @note In voltage control mode, KEY1 decreases VNH7040.des_voltage.
     * @note In current control mode, KEY1 decreases des_coil_current.
     */
    void MRC_Key1_Reaction(Device_MRC_t *MRC);
    
    /**
     * @brief Handle KEY2 response
     * @param MRC MRC device structure pointer
     *
     * @note In voltage control mode, KEY2 increases VNH7040.des_voltage.
     * @note In current control mode, KEY2 increases des_coil_current.
     */
    void MRC_Key2_Reaction(Device_MRC_t *MRC);

    /**
     * @brief Get coil current
     * @param MRC MRC device structure pointer
     * 
     * @note Get coil current from ADC buffer
     */
    float MRC_Get_Coil_Current(Device_MRC_t *MRC);

    /**
     * @brief Update coil current
     * @param MRC MRC device structure pointer
     * 
     * @note Update coil current
     */
    float MRC_Update_Coil_Current(Device_MRC_t *MRC);

    /**
     * @brief Update coil current control (feedforward + PI)
     * @param MRC Pointer to MRC device structure
     * @param i_ref Desired coil current (A)
     * @param i_meas Measured coil current (A)
     * @param R_coil Coil resistance (Ohm, e.g. 4.22)
     * @param L_coil Coil inductance (H, e.g. 0.0014868)
     * @param Ts Sample time (s)
     * @return PWM duty cycle (0~1)
    */
    float MRC_CoilCurrentControl_Update(Device_MRC_t *MRC);

    int8_t MRC_SetMode(Device_MRC_t *mrc, MRC_Mode mode);

    void MRC_send_data(Device_MRC_t *MRC);

/**
 * @brief Lookup table entry for target-to-measured voltage compensation
 *        Used to correct for nonlinearity in H-bridge output at low voltages.
 *        Each entry maps a target voltage to the experimentally measured output voltage.
 */
typedef struct {
    float target_voltage;      /**< Target (commanded) voltage, V */
    float measured_voltage;    /**< Measured output voltage, V */
} VoltageLUTEntry;

/**
 * @brief Lookup table for voltage compensation (see README for data source)
 *        For v_target <= 5V, use this table for interpolation; for v_target > 5V, use v_target directly.
 */
extern const VoltageLUTEntry voltage_lut[];

/**
 * @brief Number of entries in the voltage lookup table
 */
extern const int VOLTAGE_LUT_SIZE;

/**
 * @brief Interpolate measured voltage for a given target voltage using the lookup table
 * @param v_target Target (commanded) voltage, V
 * @return Interpolated measured voltage, V (for v_target > 5V, returns v_target directly)
 */
float LookupMeasuredVoltageByTarget(float v_target);

#ifdef __cplusplus
    }
#endif

#endif
