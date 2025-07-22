/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "math.h"
#include "drv_mrc.h"
#include "stdio.h"
#include "crc_ccitt.h"
#include "string.h"

extern DMA_HandleTypeDef hdma_usart2_rx;

#define MRC_COIL_MAX_VOLTAGE 12.0f //remeber to change the same value in drv_vnh7040.c
#define H_Encoder htim1 // Encoder timer handle for capture the pwm signal
#define Encoder_CH TIM_CHANNEL_4 // timer channel for encoder
#define PWM_TIM htim2 // PWM timer handle to generate the PWM for the VNH7040 device
#define PWM_CH TIM_CHANNEL_1
#define ADC_HANDLE &hadc1 // ADC handle for the VNH7040 device
uint32_t ADC_BUFFER[1]; // Buffer for ADC test
#define ADC_OVER_SAMPLING_RATIO 16 // ADC oversampling ratio for coil current measurement
#define ADC_SAMPLE_FREQUENCY 10000 // ADC sample frequency (Hz) for coil current measurement
#define COIL_RESISTANCE 4.22f // Coil resistance for the MRC device
#define COIL_INDUCTANCE 0.0014868f // Coil inductance for the MRC device
#define COIL_CURRENT_FILTER_CUTOFF_FREQUENCY 1000.0f // Coil current filter cutoff frequency for the MRC device
// Coil current PI controller parameters (can be tuned as needed)
#define COIL_PID_KP      1.2f
#define COIL_PID_KI      200.0f
#define COIL_PID_KD      0.0f
#define COIL_PID_TS      0.001f
#define COIL_PID_MAX_OUT 12.0f
#define COIL_PID_MIN_OUT -12.0f
//collison detection
#define COLLISION_THRESHOLD 2.0f

/**
 * @brief Initialize the MRC device
 *
 * This function initializes the MRC device, including setting the device name, timer handle, PWM channel, INA and INB
 * pins, etc.
 *
 * @details
 * The function first assigns the passed parameters to the MRC device structure, then starts the PWM channel of the
 * timer, sets the PWM parameters, and enables the INA and INB pins. Next, the function starts ADC calibration and DMA
 * transfer, and finally prints a message indicating successful device initialization.
 *
 * @note
 * This function assumes that the ADC_BUFFER array has been defined and contains two ADC values.
 */
void MRC_Init(const uint8_t *dev_name, Device_MRC_t *MRC, uint8_t id)
{


    MRC->device_name = dev_name;
    
    // Initialize MRC communication module
    if (MRC_Com_Init(&MRC->com, &huart2, id) != 0) {
        printf("MRC communication initialization failed!\n");
        return;
    }
    MRC->com.RxFlag = 0;
    // Print SAFEMRC ASCII art and copyright info
    printf("\n");
    printf("===============================================================\n");
    printf("   _____         ______ ______ __  __ _____   _____ \n");
    printf("  / ____|  /\\   |  ____|  ____|  \\/  |  __ \\ / ____|\n");
    printf(" | (___   /  \\  | |__  | |__  | \\  / | |__) | |     \n");
    printf("  \\___ \\ / /\\ \\ |  __| |  __| | |\\/| |  _  /| |     \n");
    printf("  ____) / ____ \\| |    | |____| |  | | | \\ \\| |____ \n");
    printf(" |_____/_/    \\_\\_|    |______|_|  |_|_|  \\_\\\\_____|\n");
    printf("                                                    \n");
    printf("                                                    \n");
    printf("                        (SAFEMRC)\n");
    printf("===============================================================\n");
    printf("Copyright (c) 2024 Wenbo Li, University of Science and Technology of China\n");
    printf("===============================================================\n");
    printf("\n");

    drv_led_init(&MRC->LED1, "LED1", LED1_GPIO_Port, LED1_Pin, HIGH_LEVEL); // LED1 GPIO
    drv_led_init(&MRC->LED2, "LED2", LED2_GPIO_Port, LED2_Pin, HIGH_LEVEL); // LED2 GPIO
    drv_key_init(&MRC->KEY1, "KEY1", KEY1_GPIO_Port, KEY1_Pin);         // KEY1 GPIO
    drv_key_init(&MRC->KEY2, "KEY2", KEY2_GPIO_Port, KEY2_Pin);         // KEY1 GPIO
    drv_encoder_init(&MRC->Encoder, &H_Encoder, Encoder_CH); // Encoder GPIO
    drv_VNH7040_init("H-driver", &MRC->VNH7040, INA1_GPIO_Port, INA1_Pin, INB1_GPIO_Port, INB1_Pin, &PWM_TIM, PWM_CH); // VNH7040 device structure
    
    PID_Init(&MRC->coil_pid, COIL_PID_KP, COIL_PID_KI, COIL_PID_KD, COIL_PID_TS, COIL_PID_MAX_OUT, COIL_PID_MIN_OUT); // PID controller for the coil
    
    MRC->state_phase = Disengagement;

    MRC->COLLISION_REACT_FLAG = 0;  //collision react
    MRC->collision_threshold = COLLISION_THRESHOLD; //collision threshold

    HAL_TIM_Base_Start_IT(&htim6); // timer for key

    HAL_ADCEx_Calibration_Start(ADC_HANDLE, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(ADC_HANDLE,(uint32_t *)ADC_BUFFER,1);
    HAL_TIM_Base_Start_IT(&htim4);
    SimpleLowPassFilter_InitByCutoff(&MRC->coil_current_filter, ADC_SAMPLE_FREQUENCY, COIL_CURRENT_FILTER_CUTOFF_FREQUENCY);
    MRC->coil_resistance = COIL_RESISTANCE;
    MRC->coil_inductance = COIL_INDUCTANCE;
    MRC->coil_sample_time = 1.0f / ADC_SAMPLE_FREQUENCY;

    MRC->VNH7040.des_voltage = 0.0f; // Set the initial desired voltage to 0.0V
    MRC_set_voltage(MRC);
    MRC->control_mode = MRC_CURRENT_CONTROL;
    // MRC_StateMachine_SetMode(&MRC->statemachine, FIX_LIMIT, MRC->filtered_coil_current);
    MRC_SetMode(MRC, FIX_LIMIT);

    led_on(&MRC->LED2); // Turn on LED2
    printf("Device MRC(%s) initialized successfully!\n", dev_name);

    MRC_StateMachine_Init(&MRC->statemachine);
    printf("Device MRC(%s) state machine initialized successfully!\n", dev_name);
}

/**
 * @brief Set the des_voltage of the MRC device.
 *
 * This function sets the des_voltage of the MRC device. It calculates the duty cycle based on the input des_voltage and sets
 * the PWM parameters accordingly. If the des_voltage is greater than 5.0V, it logs a message indicating that the des_voltage is
 * too high.
 *
 * @param des_voltage The des_voltage to be set for the MRC device.
 * @param MRC A pointer to the MRC device structure.
 */
void MRC_set_voltage(Device_MRC_t *MRC)
{
    if(MRC->VNH7040.des_voltage >MRC_COIL_MAX_VOLTAGE) // Check if the desired voltage is greater than the maximum voltage
    {
        MRC->VNH7040.des_voltage = MRC_COIL_MAX_VOLTAGE; // Set the desired voltage to the maximum voltage
    }
    else if(MRC->VNH7040.des_voltage < -MRC_COIL_MAX_VOLTAGE) // Check if the desired voltage is less than the minimum voltage
    {
        MRC->VNH7040.des_voltage = -MRC_COIL_MAX_VOLTAGE; // Set the desired voltage to the minimum voltage
    }
    // Use lookup table to get corrected voltage for v_target <= 5V, otherwise use v_target directly
    float corrected_voltage;
    if (MRC->VNH7040.des_voltage <= 5.0f) {
        corrected_voltage = LookupMeasuredVoltageByTarget(MRC->VNH7040.des_voltage); // This function interpolates from your table
    } else {
        corrected_voltage = MRC->VNH7040.des_voltage;
    }
    // Pass corrected_voltage to VNH7040_Set_Voltage
    VNH7040_Set_Voltage(&MRC->VNH7040, corrected_voltage);
}

/**
 * @brief Detect collisions in the MRC device.
 *
 * This function is responsible for detecting collisions in the MRC device.
 * It takes three parameters: MRC, which is a pointer to the MRC device structure,
 * param, which is a float representing the parameter to be checked for collision,
 * and threshold, which is a float representing the threshold value for collision detection.
 *
 * @param MRC A pointer to the MRC device structure.
 * @param param A float representing the parameter to be checked for collision.
 * @param threshold A float representing the threshold value for collision detection.
 *
 * @details
 * The function first calculates the absolute value of the param using the fabsf function. It then checks the current state_phase of the COLLISION_REACT_FLAG member of the MRC structure. If the flag is 0, it means no collision has occurred yet. The function then checks if the absolute value of the param is less than the threshold. If it is, it means no collision has occurred and the function returns.
 *
 * If the absolute value of the param is greater than or equal to the threshold, it means a collision has occurred. The function then calls the MRC_unlock function to unlock the MRC device, sets the COLLISION_REACT_FLAG to 1 to indicate that a collision has occurred, and prints a message indicating that a collision has been detected.
 *
 * If the COLLISION_REACT_FLAG is 1, it means a collision has already occurred. The function then checks if the absolute value of the param is less than the threshold. If it is, it means the collision has ended and the function calls the MRC_lock function to lock the MRC device, sets the COLLISION_REACT_FLAG to 0 to indicate that the collision has ended, and prints a message indicating that the collision has ended.
 */
void MRC_collision_detect(Device_MRC_t *MRC)
{
    float index = fabsf(MRC->Encoder.filtered_anguvel);
    if(!(MRC->COLLISION_REACT_FLAG))          // Non-collision state
    {
        if(index < MRC->collision_threshold)                   // Below threshold, no collision
        {
            return;
        }
        else                                    // Collision occurred, unlock MRC
        {
            MRC_StateMachine_OnCollisionDetected(&MRC->statemachine, MRC->filtered_coil_current);
            MRC->COLLISION_REACT_FLAG = 1;
            led_on(&MRC->LED1);
            printf("Collision detected!\n");
            return;
        }
    }
    else if(MRC->COLLISION_REACT_FLAG)     // Collision state
    {
        if(index >= index < MRC->collision_threshold)                  // Above threshold, collision not ended
        {
            return;
        }
        else                                    // Collision ended, relock MRC
        {
            MRC_StateMachine_OnCollisionEnded(&MRC->statemachine, MRC->filtered_coil_current);
            MRC->COLLISION_REACT_FLAG = 0;
            led_off(&MRC->LED1);
            printf("Collision ended!\n");
            return;
        }
    }
}

int8_t MRC_SetMode(Device_MRC_t *mrc, MRC_Mode mode)
{
    if(mrc->COLLISION_REACT_FLAG == 0)
    {
        MRC_StateMachine_SetMode(&mrc->statemachine, mode, mrc->filtered_coil_current);
        return 0;
    }
    else
    {
        return -1;
    }
}

/**
 * @brief MRC communication process using new mrc_com module
 * @param MRC MRC device structure pointer
 * 
 * @note Handle communication with host computer using DMA idle reception mode
 * @note Extract command and send feedback automatically
 */
void MRC_Com_Process(Device_MRC_t *MRC)
{
    // Check if new command received (RxFlag set by UART IDLE interrupt)
    if (MRC->com.RxFlag == 1) {
        if(MRC->com.cmd_msg.id == MRC->com.id){

            // Unpack and validate command message from DMA buffer
            if (MRC_Com_UnpackCmd(&MRC->com) == 0) {
                // Update device parameters from received command
                if(MRC->statemachine.current_mode != DEBUG && MRC->COLLISION_REACT_FLAG == 0)
                {
                    // MRC_StateMachine_SetMode(&MRC->statemachine, MRC->com.cmd_msg.mode, MRC->filtered_coil_current);
                    MRC_SetMode(MRC, MRC->com.cmd_msg.mode);
                    if(MRC->com.cmd_msg.mode == FIX_LIMIT || MRC->com.cmd_msg.mode == ADAPTATION)
                    {
                        MRC->des_coil_current = ((float)MRC->com.cmd_msg.des_coil_current) / 1000.0f;
                    }
                }
                
                // Prepare feedback data
                int32_t encoder_value = (int32_t)(MRC->Encoder.CurrentEncoderValRad * 65535);
                int32_t encoder_velocity = (int32_t)(MRC->Encoder.filtered_anguvel*1000);
                int32_t present_current = (int32_t)(MRC->filtered_coil_current * 1000); // Use actual voltage as torque indicator
                uint8_t collision_flag = MRC->COLLISION_REACT_FLAG; // Use correct collision flag
                
                // Pack feedback message with current device status
                if (MRC_Com_PackFbk(&MRC->com, MRC->statemachine.current_mode, encoder_value, encoder_velocity, present_current, collision_flag) == 0) {
                    // Send feedback response
                    MRC_Com_SendFbk(&MRC->com);
                }
            }
        }
        
        // Reset RxFlag after processing
        MRC->com.RxFlag = 0;
        
    }
}

/**
 * @brief Handle the reaction to the KEY1 press event.
 *
 * This function checks if KEY1 is pressed and if so, decreases the desired voltage by 1.0V.
 * It ensures that the voltage does not go below the minimum allowed value.
 *
 * @param MRC A pointer to the MRC device structure.
 */
void MRC_Key1_Reaction(Device_MRC_t *MRC)
{
    if (MRC->KEY1.key_flag == 1) // KEY1 is pressed
    {
        MRC->KEY1.key_flag = 0; // clear the flag
        led_toggle(&MRC->LED1); // toggle LED1 state
        if (MRC->control_mode == MRC_VOLTAGE_CONTROL) {
            // Voltage control mode: decrease voltage
            MRC->VNH7040.des_voltage -= 1.0f;
            if (MRC->VNH7040.des_voltage < -MRC_COIL_MAX_VOLTAGE)
                MRC->VNH7040.des_voltage = -MRC_COIL_MAX_VOLTAGE;
            MRC_set_voltage(MRC); // set the new voltage
            printf("MRC voltage decreased to %.2fV\n", MRC->VNH7040.des_voltage);
        } else {
            // Current control mode: decrease desired coil current
            MRC->des_coil_current -= 0.1f;
            printf("MRC desired coil current decreased to %.2fA\n", MRC->des_coil_current);
        }
    }
}

/**
 * @brief Handle the reaction to the KEY2 press event.
 *
 * This function checks if KEY2 is pressed and if so, decreases the desired voltage by 1.0V.
 * It ensures that the voltage does not go below the minimum allowed value.
 *
 * @param MRC A pointer to the MRC device structure.
 */
void MRC_Key2_Reaction(Device_MRC_t *MRC)
{
    if (MRC->KEY2.key_flag == 1) // KEY2 is pressed
    {
        MRC->KEY2.key_flag = 0; // clear the flag
        led_toggle(&MRC->LED2); // toggle LED2 state
        if (MRC->control_mode == MRC_VOLTAGE_CONTROL) {
            // Voltage control mode: increase voltage
            MRC->VNH7040.des_voltage += 1.0f;
            if (MRC->VNH7040.des_voltage > MRC_COIL_MAX_VOLTAGE)
                MRC->VNH7040.des_voltage = MRC_COIL_MAX_VOLTAGE;
            MRC_set_voltage(MRC); // set the new voltage
            printf("MRC voltage increased to %.2fV\n", MRC->VNH7040.des_voltage);
        } else {
            // Current control mode: increase desired coil current
            MRC->des_coil_current += 0.1f;
            printf("MRC desired coil current increased to %.2fA\n", MRC->des_coil_current);
        }
    }
}

float MRC_Get_Coil_Current(Device_MRC_t *MRC)
{
    MRC->actual_coil_current = -(3.3f*(float)ADC_BUFFER[0] / (4095 * ADC_OVER_SAMPLING_RATIO)-3.3f/2.0f)/0.264f;
    return MRC->actual_coil_current;
}

float MRC_Update_Coil_Current(Device_MRC_t *MRC)
{
    MRC->actual_coil_current = MRC_Get_Coil_Current(MRC);
    MRC->filtered_coil_current = SimpleLowPassFilter_Update(&MRC->coil_current_filter, MRC->actual_coil_current);
    return MRC->filtered_coil_current;
}

/**
 * @brief Update coil current control (feedforward + PI)
 *
 * This function computes the feedforward voltage and PI output, sums them to get the target voltage,
 * and then uses a lookup table to convert the target voltage to a PWM duty cycle.
 *
 * @param MRC Pointer to MRC device structure
 * @param i_ref Desired coil current (A)
 * @param i_meas Measured coil current (A)
 * @param R_coil Coil resistance (Ohm, e.g. 4.22)
 * @param L_coil Coil inductance (H, e.g. 0.0014868)
 * @param Ts Sample time (s)
 * @return PWM duty cycle (0~1)
 */
float MRC_CoilCurrentControl_Update(Device_MRC_t *MRC)
{
    if(MRC->control_mode == MRC_CURRENT_CONTROL)
    {
        float i_ref = MRC->des_coil_current;
        float i_meas = MRC->filtered_coil_current;
        // Feedforward voltage
        static float prev_i_ref = 0.0f;
        float di_ref = (i_ref - prev_i_ref) / MRC->coil_sample_time;
        float v_ff = MRC->coil_resistance * i_ref + MRC->coil_inductance * di_ref;
        prev_i_ref = i_ref;

        // PI controller output (target voltage correction)
        float v_pi = PID_Calculate(&MRC->coil_pid, i_ref, i_meas);

        // Total target voltage
        float v_target = v_ff + v_pi;

        // Use lookup table to get corrected voltage for |v_target| <= 5V, otherwise use v_target directly
        float corrected_voltage;
        if (v_target >= -5.0f && v_target <= 5.0f) {
            // For negative voltages, lookup uses absolute value, then restores sign
            float sign = (v_target >= 0.0f) ? 1.0f : -1.0f;
            corrected_voltage = sign * LookupMeasuredVoltageByTarget(fabsf(v_target));
        } else {
            corrected_voltage = v_target;
        }
        // Pass corrected_voltage to VNH7040_Set_Voltage
        VNH7040_Set_Voltage(&MRC->VNH7040, corrected_voltage);

        // Return the actual voltage command for logging or monitoring
        return corrected_voltage;
    }
    return 0.0f;
}

/**
 * @brief Lookup table for target-to-measured voltage compensation
 *
 * This table is based on experimental data (see README) and is used to correct for nonlinearity
 * in the H-bridge output at low voltages. For v_target <= 5V, use this table for interpolation;
 * for v_target > 5V, use v_target directly as the output voltage.
 */
const VoltageLUTEntry voltage_lut[] = {
    {0.0f, 0.0017f},
    {0.1f, 0.0017f},
    {0.2f, 0.0017f},
    {0.3f, 0.00295f},
    {0.4f, 0.00735f},
    {0.5f, 0.0142f},
    {0.6f, 0.0434f},
    {0.7f, 0.1336f},
    {0.8f, 0.2831f},
    {0.9f, 0.50175f},
    {1.0f, 0.66635f},
    {1.1f, 0.828f},
    {1.2f, 1.036f},
    {1.3f, 1.188f},
    {1.4f, 1.365f},
    {1.5f, 1.524f},
    {1.6f, 1.665f},
    {1.7f, 1.797f},
    {1.8f, 1.9745f},
    {1.9f, 2.101f},
    {2.0f, 2.229f},
    {2.1f, 2.393f},
    {2.2f, 2.5135f},
    {2.3f, 2.632f},
    {2.4f, 2.771f},
    {2.5f, 2.92f},
    {2.6f, 3.032f},
    {2.7f, 3.145f},
    {2.8f, 3.29f},
    {2.9f, 3.398f},
    {3.0f, 3.504f},
    {3.1f, 3.644f},
    {3.2f, 3.755f},
    {3.3f, 3.858f},
    {3.4f, 3.995f},
    {3.5f, 4.098f},
    {3.6f, 4.2f},
    {3.7f, 4.335f},
    {3.8f, 4.438f},
    {3.9f, 4.541f},
    {4.0f, 4.674f},
    {4.1f, 4.781f},
    {4.2f, 4.883f},
    {4.3f, 5.013f},
    {4.4f, 5.043f},
    {4.5f, 5.162f},
    {4.6f, 5.305f},
    {4.7f, 5.408f},
    {4.8f, 5.52f},
    {4.9f, 5.651f},
    {5.0f, 5.752f}
};

/**
 * @brief Number of entries in the voltage lookup table
 */
const int VOLTAGE_LUT_SIZE = sizeof(voltage_lut)/sizeof(voltage_lut[0]);

/**
 * @brief Interpolate measured voltage for a given target voltage using the lookup table
 *
 * For v_target <= 5V, performs linear interpolation using the voltage_lut table.
 * For v_target > 5V, returns v_target directly (assumes linear region).
 *
 * @param v_target Target (commanded) voltage, V
 * @return Interpolated measured voltage, V
 */
float LookupMeasuredVoltageByTarget(float v_target) {
    if (v_target >= 5.0f) {
        return v_target;
    }
    if (v_target <= voltage_lut[0].target_voltage) {
        return voltage_lut[0].measured_voltage;
    }
    for (int i = 0; i < VOLTAGE_LUT_SIZE - 1; ++i) {
        float v0 = voltage_lut[i].target_voltage;
        float v1 = voltage_lut[i+1].target_voltage;
        if (v_target >= v0 && v_target <= v1) {
            float m0 = voltage_lut[i].measured_voltage;
            float m1 = voltage_lut[i+1].measured_voltage;
            // Linear interpolation
            return m0 + (m1 - m0) * (v_target - v0) / (v1 - v0);
        }
    }
    // If not found, return last value (should not happen)
    return voltage_lut[VOLTAGE_LUT_SIZE-1].measured_voltage;
}

// 实现MRC_SetCurrent，假设通过MRC->des_coil_current设置
void MRC_SetCurrent(float current_A) {
    extern Device_MRC_t MRC;
    MRC.des_coil_current = current_A;
    // 可选：立即调用电流控制更新函数
    // MRC_CoilCurrentControl_Update(&MRC);
}
