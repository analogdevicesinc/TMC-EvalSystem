/*
 * EEPROM.c
 *
 *  Created on: 13.04.2017
 *      Author: je
 *  Modified on: 26.02.2019
 *      Author: LK
 */

#include "EEPROM.h"
#include <string.h>

EEPROM_Channels EEPROM =
{
	.ch1 =
	{
		.init = false,
		.name = { 0 },
		.id = 0,
		.hw = 0,
		.magic = 0
	},
	.ch2 =
	{
		.init = false,
		.name = { 0 },
		.id = 0,
		.hw = 0,
		.magic = 0
	}
};

// Perform initial scan and store to struct
void eeprom_init(SPIChannelTypeDef *SPIChannel)
{
	uint8_t buffer[EEPROM_SIZE_META] = { 0 };
	EEPROM_Data *eep = (SPIChannel == &SPI.ch1) ? &EEPROM.ch1 : &EEPROM.ch2;
	eeprom_read_array(SPIChannel, EEPROM_ADDR_META, buffer, EEPROM_SIZE_META);
	memcpy(eep->name, &buffer[EEPROM_ADDR_NAME - EEPROM_ADDR_META], EEPROM_SIZE_NAME);
	eep->id = _8_16(buffer[EEPROM_ADDR_ID - EEPROM_ADDR_META], buffer[(EEPROM_ADDR_ID + 1) - EEPROM_ADDR_META]);
	eep->hw = _8_16(buffer[EEPROM_ADDR_HW - EEPROM_ADDR_META], buffer[(EEPROM_ADDR_HW + 1) - EEPROM_ADDR_META]);
	eep->magic = _8_16(buffer[EEPROM_ADDR_MAGIC - EEPROM_ADDR_META], buffer[(EEPROM_ADDR_MAGIC + 1) - EEPROM_ADDR_META]);
	eep->init = true;
//	eeprom_read_array(&SPI.ch2, EEPROM_ADDR_META, buffer, EEPROM_SIZE_META);
//	memcpy(EEPROM.ch2.name, &buffer[EEPROM_ADDR_NAME - EEPROM_ADDR_META], EEPROM_SIZE_NAME);
//	EEPROM.ch2.id = _8_16(buffer[EEPROM_ADDR_ID - EEPROM_ADDR_META], buffer[(EEPROM_ADDR_ID + 1) - EEPROM_ADDR_META]);
//	EEPROM.ch2.hw = _8_16(buffer[EEPROM_ADDR_HW - EEPROM_ADDR_META], buffer[(EEPROM_ADDR_HW + 1) - EEPROM_ADDR_META]);
//	EEPROM.ch2.magic = _8_16(buffer[EEPROM_ADDR_MAGIC - EEPROM_ADDR_META], buffer[(EEPROM_ADDR_MAGIC + 1) - EEPROM_ADDR_META]);
//	EEPROM.ch2.init = true;
}

/*******************************************************************
	Function: eeprom_check
	Parameters: SPI channel the Eeprom should be checked on

	Returns: false when an Eeprom has been successfully detected and is in ready status
			The status of the eeprom otherwise

	Purpose: checking whether Eeprom is connected and ready
********************************************************************/
uint8_t eeprom_check(SPIChannelTypeDef *SPIChannel)
{
	// Prüfen, ob der SPI-Bus schon funktioniert: Im Status-Register des EEPROMs
	// müssen Bit 6, 5, 4 und 0 auf jedem Fall 0 sein und nicht 1.
	// Watchdog darf an dieser Stelle ruhig zuschlagen.

	// select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	SPIChannel->readWrite(0x05, false);  // Befehl "Get Status"
	uint8_t out = SPIChannel->readWrite(0x00, true);
	// check whether bits 6, 5, 4 and 0 are cleared
	if((out & 0x71) != 0)
		goto end;

	out = 0;

	//check for magic number in eeprom
	uint8_t number[2];

	eeprom_read_array(SPIChannel, EEPROM_ADDR_MAGIC, number, 2);

	if(number[0] != MAGICNUMBER_LOW || number[1] != MAGICNUMBER_HIGH)
	{
		out = ID_CHECKERROR_MAGICNUMBER;
	}

	end:
	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;

	if(out && ((SPIChannel == &SPI.ch1 && !EEPROM.ch1.init)
			|| (SPIChannel == &SPI.ch2 && !EEPROM.ch2.init))) {
		eeprom_init(SPIChannel);
	}

	return out;
}


/*******************************************************************
	Funktion: eeprom_write_byte
	Parameter: 	Channel: EEP_CH1 oder EEP_CH2
				address: Adresse im EEPROM (0..16383)
				value: der zu schreibende Wert

	Rückgabewert: ---

	Zweck: Schreiben eines Bytes in das EEPROM auf dem Evalboard.
********************************************************************/
void eeprom_write_byte(SPIChannelTypeDef *SPIChannel, uint16_t address, uint8_t value)
{
	// select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1) {
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
		EEPROM.ch1.init = false;
	} else {
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;
		EEPROM.ch2.init = false;
	}

	IOs.toOutput(SPIChannel->CSN);

	// Schreiben erlauben
	SPIChannel->readWrite(0x06, true); // Befehl "Write Enable"
	do
	{
		SPIChannel->readWrite(0x05, false); // Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, true) & 0x02) == 0x00);  // Warte bis "Write Enable"-Bit gesetzt ist

	// Eigentliches Schreiben
	SPIChannel->readWrite(0x02, false); // Befehl "Write"
	SPIChannel->readWrite(address >> 8, false);
	SPIChannel->readWrite(address & 0xFF, false);
	SPIChannel->readWrite(value, true);

	// Warten bis Schreibvorgang beendet ist
	do
	{
		SPIChannel->readWrite(0x05, false); //Befehl "Get Status"
	} while(SPIChannel->readWrite(0x00, true) & 0x01);

	//block writing
	SPIChannel->readWrite(0x04, true); //Befehl "Write Disable"
	do
	{
		SPIChannel->readWrite(0x05, false); //Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, true) & 0x02) != 0x00); //Warte bis "Write Enable"-Bit zurückgesetzt wird

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
}


/*******************************************************************
	Funktion: eeprom_write_array
	Parameter:	Channel: EEP_CH1 oder EEP_CH2
				address: Adresse im EEPROM (0..16383)
				data: Startadresse des zu schreibenden Blocks
				size: Länge des Blocks in Bytes

	Rückgabewert: ---

	Zweck: Schreiben mehrerer Bytes in das EEPROM auf dem Evalboard.
	Dabei können beliebig viele Bytes (also auch das gesamte EEPROM
	beschrieben werden (die speziellen Eigenschaften des 25128 werden
	dabei beachtet).
********************************************************************/
void eeprom_write_array(SPIChannelTypeDef *SPIChannel, uint16_t address, uint8_t *data, uint16_t size)
{
	uint16_t i;

	//select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1) {
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
		EEPROM.ch1.init = false;
	} else {
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;
		EEPROM.ch2.init = false;
	}

	IOs.toOutput(SPIChannel->CSN);

	// Schreiben erlauben
	SPIChannel->readWrite(0x06, true); // Befehl "Write Enable"
	do
	{
		SPIChannel->readWrite( 0x05, false); //Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, true) & 0x02)==0x00); //Warte bis "Write Enable"-Bit gesetzt

	// Schreibvorgang (Startadresse)
	SPIChannel->readWrite(0x02, false); // Befehl "Write"
	SPIChannel->readWrite(address >> 8, false);
	SPIChannel->readWrite(address & 0xFF, false);

	// Eigentliches Schreiben der Daten
	for(i = 0; i < size; i++)
	{
		// Adresse mitzählen und bei Überlauf der untersten sechs Bits das EEPROM deselektieren
		// und neuen Write-Befehl senden (bzw. beim letzten Datenbyte einfach nur EEPROM
		// deselektieren).
		// Dies ist erforderlich, da beim Beschreiben im 25128 nur die untersten sechs Bits der
		// Adresse hochgezählt werden (anders als beim Lesen).
		address++;
		SPIChannel->readWrite(*(data+i), (address & 0x0000003F)==0 || i==size-1);
		if((address & 0x0000003F)==0 && i<size-1)  // Adressbits übergelaufen, aber noch Bytes zu schreiben?
		{
			// Warte bis Schreibvorgang beendet
			do
			{
				SPIChannel->readWrite(0x05, false);  // Befehl "Get Status"
			} while(SPIChannel->readWrite(0x00, true) & 0x01);

			// Neuer "Write Enable"-Befehl
			SPIChannel->readWrite(0x06, true);  // Befehl "Write Enable"
			do
			{
				SPIChannel->readWrite(0x05, false);  // Befehl "Get Status"
			} while((SPIChannel->readWrite(0x00, true) & 0x02)==0x00); //Warte bis "Write Enable"-Bit gesetzt

			// Neuer "Write"-Befehl (mit der nächsten Adresse)
			SPIChannel->readWrite(0x02, false); // Befehl "Write"
			SPIChannel->readWrite(address >> 8, false);
			SPIChannel->readWrite(address & 0xFF, false);
		}
	}

	// Warte bis Schreibvorgang beendet
	do
	{
		SPIChannel->readWrite(0x05, false); // Befehl "Get Status"
	} while(SPIChannel->readWrite(0x00, true) & 0x01);

	// block writing
	SPIChannel->readWrite(0x04, true); // Befehl "Write Disable"
	do
	{
		SPIChannel->readWrite(0x05, false); // Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, true) & 0x02) != 0x00);  // Warte bis "Write Enable"-Bit zurückgesetzt wird

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
}


/*******************************************************************
	Funktion: eeprom_read_byte
	Parameter:	Channel: EEP_CH1 oder EEP_CH2
				address: Adresse im EEPROM (0..16383)

	Rückgabewert: der gelesene Wert

	Zweck: Lesen eines Bytes aus dem EEPROM des Evalboards.
********************************************************************/
uint8_t eeprom_read_byte(SPIChannelTypeDef *SPIChannel, uint16_t address)
{
	//select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	SPIChannel->readWrite(0x03, false); //Befehl "Read"
	SPIChannel->readWrite(address >> 8, false);
	SPIChannel->readWrite(address & 0xFF, false);

	uint8_t out = SPIChannel->readWrite(0, true);

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;

	return out;
}


/*******************************************************************
	Funktion: eeprom_read_array
	Parameter:	Channel: EEP_CH1 oder EEP_CH2
				address: Adresse im EEPROM (0..16383)
				data: Startadresse des zu lesenden Blocks
				size: Länge des Blocks in Bytes

	Rückgabewert: ---

	Zweck: Lesen mehrerer Bytes aus dem Konfigurations-EEPROM.
	Dabei dürfen ab beliebiger Adresse beliebig viele Bytes gelesen
	werden.
********************************************************************/
void eeprom_read_array(SPIChannelTypeDef *SPIChannel, uint16_t address, uint8_t *data, uint16_t size)
{
	uint16_t i;

	// select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	SPIChannel->readWrite(0x03, false); // Befehl "Read"
	SPIChannel->readWrite(address >> 8, false);
	SPIChannel->readWrite(address & 0xFF, false);

	for(i = 0; i < size; i++)
		*(data+i) = SPIChannel->readWrite(0, i == size-1); // beim letzten Byte EEPROM deselektieren

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
}
