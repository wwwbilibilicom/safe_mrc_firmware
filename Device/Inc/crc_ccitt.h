/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

/*
 * This mysterious table is just the CRC of each possible byte. It can be
 * computed using the standard bit-at-a-time methods. The polynomial can
 * be seen in entry 128, 0x8408. This corresponds to x^0 + x^5 + x^12.
 * Add the implicit x^16, and you have the standard CRC-CCITT.
 * https://github.com/torvalds/linux/blob/5bfc75d92efd494db37f5c4c173d3639d4772966/lib/crc-ccitt.c
 */

/**
 * @brief Calculate CRC-CCITT checksum
 * @param crc Initial CRC value, typically use 0xFFFF
 * @param buffer Pointer to data buffer to calculate CRC
 * @param len Data length (number of bytes)
 * @return Calculated CRC-CCITT checksum
 * 
 * @note This is a CRC-CCITT variant implementation with polynomial x^16 + x^12 + x^5 + 1
 * @note Suitable for data integrity verification, such as error detection in communication protocols
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint8_t len);


#endif
