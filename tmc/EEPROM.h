/*
 * EEPROM.h
 *
 *  Created on: 13.04.2017
 *      Author: je
 *  Modified on: 26.02.2019
 *      Author: LK
 */

#ifndef TMC_EEPROM_H_
#define TMC_EEPROM_H_

#include "tmc/helpers/API_Header.h"

#include "../hal/SPI.h"
#include "../hal/HAL.h"

#define EEPROM_ADDR_NAME   0
#define EEPROM_ADDR_ID     16
#define EEPROM_ADDR_HW     18
#define EEPROM_ADDR_MAGIC  20
#define EEPROM_ADDR_META   0

#define EEPROM_SIZE_NAME   16
#define EEPROM_SIZE_ID     2
#define EEPROM_SIZE_HW     2
#define EEPROM_SIZE_MAGIC  2
#define EEPROM_SIZE_META   (EEPROM_SIZE_NAME + EEPROM_SIZE_ID + EEPROM_SIZE_HW + EEPROM_SIZE_MAGIC)

#define MAGICNUMBER_LOW   0x12
#define MAGICNUMBER_HIGH  0x34

#define ID_CHECKERROR_MAGICNUMBER 2

typedef struct {
	bool init;
	uint8 name[EEPROM_SIZE_NAME];
	uint16 id;
	uint16 hw;
	uint16 magic;
} EEPROM_Data;

typedef struct {
	EEPROM_Data ch1;
	EEPROM_Data ch2;
} EEPROM_Channels;

EEPROM_Channels EEPROM;

void eeprom_init(SPIChannelTypeDef *SPIChannel);

uint8 eeprom_check(SPIChannelTypeDef *SPIChannel);

void eeprom_write_byte(SPIChannelTypeDef *SPIChannel, uint16 address, uint8 value);
void eeprom_write_array(SPIChannelTypeDef *SPIChannel, uint16 address, uint8 *data, uint16 size);

uint8 eeprom_read_byte(SPIChannelTypeDef *SPIChannel, uint16 address);
void eeprom_read_array(SPIChannelTypeDef *SPIChannel, uint16 address, uint8 *block, uint16 size);

#endif /* TMC_EEPROM_H_ */
