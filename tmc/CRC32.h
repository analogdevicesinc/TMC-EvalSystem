/*******************************************************************************
* Copyright © 2020 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices, Inc.),
*
* Copyright © 2023 Analog Devices, Inc.
*******************************************************************************/

/*
 * Author: Trinamic Software Team
 */

#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>

extern const uint32_t crc32Lut[256];

uint32_t crc_crc32(uint8_t *data, uint32_t bytes);
uint32_t crc_crc32step(uint8_t data, uint32_t crc);

#endif /* CRC32_H_ */
