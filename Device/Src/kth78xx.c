/**************************************************
** Copyright (c) 2016-202X 昆泰芯微电子科技有限公司
** 文件名: kth78xx.c
** 作者: liujunbo
** 日期: 2023.08.16
** 描述: KTH78xx芯片相关文件，存放用来操作KTH78xx芯片的函数
**
**************************************************/

#include "kth78xx.h"

extern SPI_HandleTypeDef hspi4;

static uint8_t KTH78_SPI_TransmitReceiveBytes(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    uint32_t timeout = HAL_GetTick();
    
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, 2);
    
    // 添加超时机制
    while(HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY) {
        if(HAL_GetTick() - timeout > 1) {
            HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);
            return HAL_ERROR;
        }
    }
    
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);
    return HAL_OK;
}

void KTH78_SPI_TransmitReceiveBytes_WithoutWait(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size)
{
    HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi, pTxData, pRxData, 2);
}

//读取角度值
float KTH78_ReadAngle(void)
{
  uint8_t databack[2];
  uint8_t cmd[2];

  cmd[0] = KTH78_READ_ANGLE;
  cmd[1] = KTH78_NON_DATA;
  
  KTH78_SPI_TransmitReceiveBytes(&hspi4, cmd, databack, 2);

  return (float)(((databack[0] << 8) + databack[1]) * 360.0f/65536.0f);
}

void KTH78_ExchangeData_WithoutWait(uint8_t *databack)
{
  uint8_t cmd[2];

  cmd[0] = KTH78_READ_ANGLE;
  cmd[1] = KTH78_NON_DATA;

  KTH78_SPI_TransmitReceiveBytes_WithoutWait(&hspi4, cmd, databack, 2);
}

//读取寄存器数据
uint8_t KTH78_ReadReg(uint8_t addr)
{
  uint8_t databack[2];
  uint8_t cmd[2];

  cmd[0] = KTH78_READ_REG + addr;
  cmd[1] = KTH78_NON_DATA;

  KTH78_SPI_TransmitReceiveBytes(&hspi4, cmd, databack, 2);

  return databack[0];
}

//写寄存器数据
uint8_t KTH78_WriteReg(uint8_t addr, uint8_t data)
{
  uint8_t databack[2];
  uint8_t cmd[2];

  cmd[0] = KTH78_WRITE_REG + addr;
  cmd[1] = data;

  KTH78_SPI_TransmitReceiveBytes(&hspi4, cmd, databack, 2);

  return databack[0];
}

//向MTP中写入配置
uint8_t KTH78_WriteMTP(uint8_t addr, uint8_t data)
{
  uint8_t databack[2];
  uint8_t cmd[2];

  cmd[0] = KTH78_WRITE_MTP + addr;
  cmd[1] = data;

  KTH78_SPI_TransmitReceiveBytes(&hspi4, cmd, databack, 2);

  return databack[0];
}
