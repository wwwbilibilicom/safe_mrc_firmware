#ifndef __MRC_DEBUGCLI_H__
#define __MRC_DEBUGCLI_H__

#include "main.h"
#include "drv_mrc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USART1_RX_BUFFER_SIZE 64
extern char usart1_rx_buffer[USART1_RX_BUFFER_SIZE];

/**
 * @brief Parse and execute a received USART1 string command
 * @param cmd_str: Command string
 * @param mrc: Pointer to MRC device structure
 */
void MRC_DebugCLI_Parse(const char *cmd_str, Device_MRC_t *mrc);

/**
 * @brief Initialize USART1 CLI with DMA+idle reception
 * @param huart: USART1 handle
 */
void MRC_DebugCLI_Init(UART_HandleTypeDef *huart);

/**
 * @brief Should be called from USART1 idle interrupt handler. Handles DMA stop, parsing, and restart.
 * @param huart: USART1 handle
 */
void MRC_DebugCLI_UART_IdleHandler(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif 
