#ifndef __KTH78XX_H
#define __KTH78XX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define KTH78_READ_ANGLE          0x00
#define KTH78_READ_REG            (0x01 << 6)
#define KTH78_WRITE_REG           (0x03 << 6)
#define KTH78_WRITE_MTP           (0x02 << 6)

#define KTH78_NON_DATA            0x00

float KTH78_ReadAngle(void);
uint8_t KTH78_ReadReg(uint8_t addr);
uint8_t KTH78_WriteReg(uint8_t addr, uint8_t data);
uint8_t KTH78_WriteMTP(uint8_t addr, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __KTH78XX_H */
