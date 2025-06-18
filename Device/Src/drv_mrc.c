//
// Created by wenbo on 24-9-7.
//
#include "math.h"
#include "drv_mrc.h"
#include "Array_user.h"
#include "stdio.h"
#include "crc_ccitt.h"
#include "string.h"

extern DMA_HandleTypeDef hdma_usart2_rx;

#define MRC_COIL_MAX_VOLTAGE 12.0f //remeber to change the same value in drv_vnh7040.c
#define H_Encoder htim1 // Encoder timer handle for capture the pwm signal
#define Encoder_CH TIM_CHANNEL_4 // timer channel for encoder
#define PWM_TIM htim2 // PWM timer handle to generate the PWM for the VNH7040 device
#define PWM_CH TIM_CHANNEL_1
#define ADC_HANDLE hadc1 // ADC handle for the VNH7040 device

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

    drv_led_init(&MRC->LED1, "LED1", LED1_GPIO_Port, LED1_Pin, HIGH_LEVEL); // LED1 GPIO
    drv_led_init(&MRC->LED2, "LED2", LED2_GPIO_Port, LED2_Pin, HIGH_LEVEL); // LED2 GPIO
    drv_key_init(&MRC->KEY1, "KEY1", KEY1_GPIO_Port, KEY1_Pin);         // KEY1 GPIO
    drv_key_init(&MRC->KEY2, "KEY2", KEY2_GPIO_Port, KEY2_Pin);         // KEY1 GPIO
    drv_encoder_init(&MRC->Encoder, &H_Encoder, Encoder_CH); // Encoder GPIO
    drv_VNH7040_init("H-driver", &MRC->VNH7040, INA1_GPIO_Port, INA1_Pin, INB1_GPIO_Port, INB1_Pin, &PWM_TIM, PWM_CH, &hadc1); // VNH7040 device structure
    
    PID_Init(&MRC->coil_pid, 0.1f, 0.0f, 0.01f, 0.001f, MRC_COIL_MAX_VOLTAGE, -MRC_COIL_MAX_VOLTAGE); // PID controller for the coil
    
    MRC->state_phase = Disengagement;

    MRC->COLLISION_REACT_FLAG = 0;  //collision react

    HAL_TIM_Base_Start_IT(&htim6); // timer for key

    MRC->VNH7040.des_voltage = 0.0f; // Set the initial desired voltage to 0.0V
    MRC_set_voltage(MRC);
    led_on(&MRC->LED2); // Turn on LED1
    printf("device MRC(%s) init success!\n", dev_name);
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
    VNH7070_Multisense_ADC_process(&MRC->VNH7040); // Process the ADC data
#if 0
    VNH7040_Set_Voltage(&MRC->VNH7040, 
        PID_Calculate(  &MRC->coil_pid, 
                        MRC->VNH7040.des_voltage, 
                        MRC->VNH7040.actual_voltage)); // Set the voltage of the VNH7040 device
#endif
    VNH7040_Set_Voltage(&MRC->VNH7040, MRC->VNH7040.des_voltage); // Set the voltage of the VNH7040 device
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
void MRC_collision_detect(Device_MRC_t *MRC1, Device_MRC_t *MRC2, float param, float threshold)
{
    float index = fabsf(param);
    if(!(MRC1->COLLISION_REACT_FLAG && MRC2->COLLISION_REACT_FLAG))          // Non-collision state
    {
        if(index < threshold)                   // Below threshold, no collision
        {
            return;
        }
        else                                    // Collision occurred, unlock MRC
        {
            // MRC_unlock(MRC1);
            // MRC_unlock(MRC2);
            MRC1->COLLISION_REACT_FLAG = 1;
            MRC2->COLLISION_REACT_FLAG = 1;
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);   // MRC power off demagnetization, indicator light off
            printf("Collision detected!\n");
            return;
        }
    }
    else if(MRC1->COLLISION_REACT_FLAG && MRC2->COLLISION_REACT_FLAG)     // Collision state
    {
        if(index >= threshold)                  // Above threshold, collision not ended
        {
            return;
        }
        else                                    // Collision ended, relock MRC
        {
            // MRC_lock(MRC1);
            // MRC_lock(MRC2);
            MRC1->COLLISION_REACT_FLAG = 0;
            MRC2->COLLISION_REACT_FLAG = 0;
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);    // MRC power on magnetization, indicator light on
            printf("Collision ended!\n");
            return;
        }
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
        // Unpack and validate command message from DMA buffer
        if (MRC_Com_UnpackCmd(&MRC->com) == 0) {
            // Update device parameters from received command
            MRC->mode = MRC->com.cmd_msg.mode;
            MRC->des_torque = ((float)MRC->com.cmd_msg.des_torque) / 100.0f;
            
            // Prepare feedback data
            uint32_t encoder_value = (uint32_t)(MRC->Encoder.filtered_angle * 1000);
            uint16_t present_torque = (uint16_t)(MRC->VNH7040.actual_voltage * 1000); // Use actual voltage as torque indicator
            uint8_t collision_flag = MRC->COLLISION_REACT_FLAG; // Use correct collision flag
            
            // Pack feedback message with current device status
            if (MRC_Com_PackFbk(&MRC->com, MRC->mode, encoder_value, present_torque, collision_flag) == 0) {
                // Send feedback response
                MRC_Com_SendFbk(&MRC->com);
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
    if(MRC->KEY1.key_flag == 1) // KEY1 is pressed
    {
        MRC->KEY1.key_flag = 0; // clear the flag
        led_toggle(&MRC->LED1); // toggle LED1 state
        MRC->VNH7040.des_voltage -= 0.1f; // drecease the voltage by 1.0V
        if(MRC->VNH7040.des_voltage >=-MRC_COIL_MAX_VOLTAGE) // voltage is greater than or equal to the minimum voltage
        {
            MRC_set_voltage(MRC); // set the new voltage
            printf("MRC voltage decreased to %.2fV\n", MRC->VNH7040.des_voltage);
        }
        else // voltage is less than the minimum voltage
        {
            MRC->VNH7040.des_voltage = -MRC_COIL_MAX_VOLTAGE; // set the voltage to the minimum voltage
            MRC_set_voltage(MRC); // set the new voltage
            printf("MRC voltage is already at minimum value of %.2fV\n", -MRC_COIL_MAX_VOLTAGE);
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
    if(MRC->KEY2.key_flag == 1) // KEY2 is pressed
    {
        MRC->KEY2.key_flag = 0; // clear the flag
        led_toggle(&MRC->LED2); // toggle LED2 state
        MRC->VNH7040.des_voltage += 0.1f; // increase the voltage by 1.0V
        if(MRC->VNH7040.des_voltage <= MRC_COIL_MAX_VOLTAGE) // the voltage is less than the maximum voltage
        {
            MRC_set_voltage(MRC); // set the new voltage
            printf("MRC voltage increased to %.2fV\n", MRC->VNH7040.des_voltage);
        }
        else // the voltage is greater than or equal to the maximum voltage
        {
            MRC->VNH7040.des_voltage = MRC_COIL_MAX_VOLTAGE; // set the voltage to the maximum value
            MRC_set_voltage(MRC); // set the new voltage
            printf("MRC voltage is already at maximum value of %.2fV\n", MRC_COIL_MAX_VOLTAGE);
        }
    }
}
