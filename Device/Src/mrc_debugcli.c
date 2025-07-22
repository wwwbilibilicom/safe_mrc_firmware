/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "mrc_debugcli.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "drv_mrc.h"
#include "mrc_statemachine.h"

char usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
volatile uint16_t usart1_rx_index = 0;

// 安全范围定义
#define DEBUGCLI_VOLTAGE_MIN   -12.0f
#define DEBUGCLI_VOLTAGE_MAX    12.0f
#define DEBUGCLI_CURRENT_MIN   -5.0f
#define DEBUGCLI_CURRENT_MAX    5.0f

void MRC_DebugCLI_Parse(const char *cmd_str, Device_MRC_t *mrc) {
    if (!cmd_str || !mrc || strlen(cmd_str) == 0) return;
    char cmd[16] = {0};
    char arg1[16] = {0};
    char arg2[16] = {0};
    float value = 0.0f;
    int n = sscanf(cmd_str, "%15s %15s %15s", cmd, arg1, arg2);
    if (n < 1) {
        printf("[USART1] Invalid command.\n");
        return;
    }
    for (int i = 0; cmd[i]; ++i) cmd[i] = toupper(cmd[i]);
    for (int i = 0; arg1[i]; ++i) arg1[i] = toupper(arg1[i]);
    for (int i = 0; arg2[i]; ++i) arg2[i] = toupper(arg2[i]);

    // DEBUG ON/OFF commands
    if (strcmp(cmd, "DEBUG") == 0) {
        if (strcmp(arg1, "ON") == 0) {
            // MRC_StateMachine_SetMode(&mrc->statemachine, DEBUG, mrc->filtered_coil_current);
            MRC_SetMode(mrc, DEBUG);
            printf("[USART1] DEBUG mode enabled.\n");
        } else if (strcmp(arg1, "OFF") == 0) {
            // MRC_StateMachine_SetMode(&mrc->statemachine, FIX_LIMIT, mrc->filtered_coil_current);
            MRC_SetMode(mrc, FIX_LIMIT);
            mrc->control_mode = MRC_CURRENT_CONTROL;
            printf("[USART1] DEBUG mode disabled. Now in FIX_LIMIT mode.\n");
        } else {
            printf("[USART1] Unknown DEBUG command: %s\n", arg1);
        }
        return;
    }

    // Only allow other commands in DEBUG mode
    if (mrc->statemachine.current_mode != DEBUG) {
        const char *mode_str = "UNKNOWN";
        switch (mrc->statemachine.current_mode) {
            case FREE: mode_str = "FREE"; break;
            case FIX_LIMIT: mode_str = "FIX_LIMIT"; break;
            case ADAPTATION: mode_str = "ADAPTATION"; break;
            case DEBUG: mode_str = "DEBUG"; break;
            default: break;
        }
        printf("[USART1] DEBUG mode is not enabled! Current mode: %s\n", mode_str);
        return;
    }

    if (n < 2) {
        printf("[USART1] Invalid command.\n");
        return;
    }

    if (strcmp(cmd, "MODE") == 0) {
        if (strcmp(arg1, "VOLTAGE") == 0) {
            if (!MRC_StateMachine_CanSetVoltage(&mrc->statemachine)) {
                printf("[USART1] Cannot switch to VOLTAGE mode in current state.\n");
                return;
            }
            mrc->control_mode = MRC_VOLTAGE_CONTROL;
            printf("[USART1] Switched to VOLTAGE control mode.\n");
        } else if (strcmp(arg1, "CURRENT") == 0) {
            if (!MRC_StateMachine_CanSetCurrent(&mrc->statemachine)) {
                printf("[USART1] Cannot switch to CURRENT mode in current state.\n");
                return;
            }
            mrc->control_mode = MRC_CURRENT_CONTROL;
            printf("[USART1] Switched to CURRENT control mode.\n");
        } else {
            printf("[USART1] Unknown mode: %s\n", arg1);
        }
    } else if (strcmp(cmd, "SET") == 0) {
        if (n < 3) {
            printf("[USART1] SET command missing value.\n");
            return;
        }
        value = atof(arg2);
        if (strcmp(arg1, "VOLTAGE") == 0) {
            if (!MRC_StateMachine_CanSetVoltage(&mrc->statemachine) || mrc->control_mode != MRC_VOLTAGE_CONTROL) {
                printf("[USART1] Please switch to VOLTAGE control mode first using: MODE VOLTAGE\n");
                return;
            }
            if (value < DEBUGCLI_VOLTAGE_MIN || value > DEBUGCLI_VOLTAGE_MAX) {
                printf("[USART1] Voltage out of range (%.1f~%.1f V)!\n", DEBUGCLI_VOLTAGE_MIN, DEBUGCLI_VOLTAGE_MAX);
                return;
            }
            mrc->VNH7040.des_voltage = value;
            MRC_set_voltage(mrc);
            printf("[USART1] Set target voltage: %.3f V\n", value);
        } else if (strcmp(arg1, "CURRENT") == 0) {
            if (!MRC_StateMachine_CanSetCurrent(&mrc->statemachine) || mrc->control_mode != MRC_CURRENT_CONTROL) {
                printf("[USART1] Please switch to CURRENT control mode first using: MODE CURRENT\n");
                return;
            }
            if (value < DEBUGCLI_CURRENT_MIN || value > DEBUGCLI_CURRENT_MAX) {
                printf("[USART1] Current out of range (%.1f~%.1f A)!\n", DEBUGCLI_CURRENT_MIN, DEBUGCLI_CURRENT_MAX);
                return;
            }
            mrc->des_coil_current = value;
            printf("[USART1] Set target current: %.3f A\n", value);
        } else {
            printf("[USART1] Unknown SET arg: %s\n", arg1);
        }
    } else {
        printf("[USART1] Unknown command: %s\n", cmd);
    }
}

void MRC_DebugCLI_Init(UART_HandleTypeDef *huart) {
    // Start DMA reception for USART1 CLI
    memset(usart1_rx_buffer, 0, USART1_RX_BUFFER_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE); //使能接收中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); // Enable idle interrupt
    HAL_UART_Receive_DMA(huart, (uint8_t*)usart1_rx_buffer, USART1_RX_BUFFER_SIZE);
}

void MRC_DebugCLI_UART_IdleHandler(UART_HandleTypeDef *huart) {
    extern Device_MRC_t MRC;
    // Clear idle flag
    __HAL_UART_CLEAR_IDLEFLAG(huart); 
    // Stop DMA
    HAL_UART_DMAStop(huart);
    // Calculate received length
    uint16_t len = USART1_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (len > 0 && len < USART1_RX_BUFFER_SIZE) {
        usart1_rx_buffer[len] = '\0';
        MRC_DebugCLI_Parse((const char*)usart1_rx_buffer, &MRC);
    }
    // Restart DMA reception
    memset(usart1_rx_buffer, 0, USART1_RX_BUFFER_SIZE);
    HAL_UART_Receive_DMA(huart, (uint8_t*)usart1_rx_buffer, USART1_RX_BUFFER_SIZE);

}

