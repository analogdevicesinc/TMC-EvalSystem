/*
 * IdEeprom.h
 *
 *  Created on: 13.04.2017
 *      Author: je
 */

#ifndef TMC_IDEEPROM_H_
#define TMC_IDEEPROM_H_

#include "tmc/helpers/API_Header.h"

#include "../hal/SPI.h"
#include "../hal/HAL.h"

#define IDEEPROM_ADDR_NAME   0
#define IDEEPROM_ADDR_ID     16
#define IDEEPROM_ADDR_HW     18
#define IDEEPROM_ADDR_MAGIC  20

#define ID_MAGICNUMBER_LOW   0x12
#define ID_MAGICNUMBER_HIGH  0x34

#define ID_CHECKERROR_MAGICNUMBER 2

uint8 checkEeprom(SPIChannelTypeDef *SPIChannel);

void writeBoardIdEepromByte(SPIChannelTypeDef *SPIChannel, uint16 Address, uint8 Value);
void writeBoardIdEepromBlock(SPIChannelTypeDef *SPIChannel, uint16 Address, uint8 *Block, uint16 Size);

uint8 readBoardIdEepromByte(SPIChannelTypeDef *SPIChannel, uint16 Address);
void readBoardIdEepromBlock(SPIChannelTypeDef *SPIChannel, uint16 Address, uint8 *Block, uint16 Size);


#endif /* TMC_IDEEPROM_H_ */
