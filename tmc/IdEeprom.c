/*
 * IdEeprom.c
 *
 *  Created on: 13.04.2017
 *      Author: je
 */

#include "IdEeprom.h"

/*******************************************************************
	Function: checkEeprom
	Parameters: SPI channel the Eeprom should be checked on

	Returns: false when an Eeprom has been successfully detected and is in ready status
			The status of the eeprom otherwise

	Purpose: checking whether Eeprom is connected and ready
********************************************************************/
uint8 checkEeprom(SPIChannelTypeDef *SPIChannel)
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

	SPIChannel->readWrite(0x05, FALSE);  // Befehl "Get Status"
	uint8 out = SPIChannel->readWrite(0x00, TRUE);
	// check whether bits 6, 5, 4 and 0 are cleared
	if((out & 0x71) != 0)
		goto end;

	out = 0;

	//check for magic number in eeprom
	u8 number[2];

	readBoardIdEepromBlock(SPIChannel, IDEEPROM_ADDR_MAGIC, number, 2);

	if(number[0] != ID_MAGICNUMBER_LOW || number[1] != ID_MAGICNUMBER_HIGH)
	{
		out = ID_CHECKERROR_MAGICNUMBER;
	}

	end:
	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
	return out;
}


/*******************************************************************
	Funktion: WriteBoardIdEepromByte
	Parameter: 	Channel: EEP_CH1 oder EEP_CH2
				Address: Adresse im EEPROM (0..16383)
				Value: der zu schreibende Wert

	Rückgabewert: ---

	Zweck: Schreiben eines Bytes in das EEPROM auf dem Evalboard.
********************************************************************/
void writeBoardIdEepromByte(SPIChannelTypeDef *SPIChannel, uint16 Address, uint8 Value)
{
	// select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	// Schreiben erlauben
	SPIChannel->readWrite(0x06, TRUE); // Befehl "Write Enable"
	do
	{
		SPIChannel->readWrite(0x05, FALSE); // Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, TRUE) & 0x02) == 0x00);  // Warte bis "Write Enable"-Bit gesetzt ist

	// Eigentliches Schreiben
	SPIChannel->readWrite(0x02, FALSE); // Befehl "Write"
	SPIChannel->readWrite(Address >> 8, FALSE);
	SPIChannel->readWrite(Address & 0xFF, FALSE);
	SPIChannel->readWrite(Value, TRUE);

	// Warten bis Schreibvorgang beendet ist
	do
	{
		SPIChannel->readWrite(0x05, FALSE); //Befehl "Get Status"
	} while(SPIChannel->readWrite(0x00, TRUE) & 0x01);

	//block writing
	SPIChannel->readWrite(0x04, TRUE); //Befehl "Write Disable"
	do
	{
		SPIChannel->readWrite(0x05, FALSE); //Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, TRUE) & 0x02) == 0x01); //Warte bis "Write Enable"-Bit zurückgesetzt wird

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
}


/*******************************************************************
	Funktion: WriteBoardIdEepromBlock
	Parameter:	Channel: EEP_CH1 oder EEP_CH2
				Address: Adresse im EEPROM (0..16383)
				Block: Startadresse des zu schreibenden Blocks
				Size: Länge des Blocks in Bytes

	Rückgabewert: ---

	Zweck: Schreiben mehrerer Bytes in das EEPROM auf dem Evalboard.
	Dabei können beliebig viele Bytes (also auch das gesamte EEPROM
	beschrieben werden (die speziellen Eigenschaften des 25128 werden
	dabei beachtet).
********************************************************************/
void writeBoardIdEepromBlock(SPIChannelTypeDef *SPIChannel, uint16 Address, uint8 *Block, uint16 Size)
{
	uint16 i;

	//select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	// Schreiben erlauben
	SPIChannel->readWrite(0x06, TRUE); // Befehl "Write Enable"
	do
	{
		SPIChannel->readWrite( 0x05, FALSE); //Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, TRUE) & 0x02)==0x00); //Warte bis "Write Enable"-Bit gesetzt

	// Schreibvorgang (Startadresse)
	SPIChannel->readWrite(0x02, FALSE); // Befehl "Write"
	SPIChannel->readWrite(Address >> 8, FALSE);
	SPIChannel->readWrite(Address & 0xFF, FALSE);

	// Eigentliches Schreiben der Daten
	for(i = 0; i < Size; i++)
	{
		// Adresse mitzählen und bei Überlauf der untersten sechs Bits das EEPROM deselektieren
		// und neuen Write-Befehl senden (bzw. beim letzten Datenbyte einfach nur EEPROM
		// deselektieren).
		// Dies ist erforderlich, da beim Beschreiben im 25128 nur die untersten sechs Bits der
		// Adresse hochgezählt werden (anders als beim Lesen).
		Address++;
		SPIChannel->readWrite(*(Block+i), (Address & 0x0000003F)==0 || i==Size-1);
		if((Address & 0x0000003F)==0 && i<Size-1)  // Adressbits übergelaufen, aber noch Bytes zu schreiben?
		{
			// Warte bis Schreibvorgang beendet
			do
			{
				SPIChannel->readWrite(0x05, FALSE);  // Befehl "Get Status"
			} while(SPIChannel->readWrite(0x00, TRUE) & 0x01);

			// Neuer "Write Enable"-Befehl
			SPIChannel->readWrite(0x06, TRUE);  // Befehl "Write Enable"
			do
			{
				SPIChannel->readWrite(0x05, FALSE);  // Befehl "Get Status"
			} while((SPIChannel->readWrite(0x00, TRUE) & 0x02)==0x00); //Warte bis "Write Enable"-Bit gesetzt

			// Neuer "Write"-Befehl (mit der nächsten Adresse)
			SPIChannel->readWrite(0x02, FALSE); // Befehl "Write"
			SPIChannel->readWrite(Address >> 8, FALSE);
			SPIChannel->readWrite(Address & 0xFF, FALSE);
		}
	}

	// Warte bis Schreibvorgang beendet
	do
	{
		SPIChannel->readWrite(0x05, FALSE); // Befehl "Get Status"
	} while(SPIChannel->readWrite(0x00, TRUE) & 0x01);

	// block writing
	SPIChannel->readWrite(0x04, TRUE); // Befehl "Write Disable"
	do
	{
		SPIChannel->readWrite(0x05, FALSE); // Befehl "Get Status"
	} while((SPIChannel->readWrite(0x00, TRUE) & 0x02) == 0x01);  // Warte bis "Write Enable"-Bit zurückgesetzt wird

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
}


/*******************************************************************
	Funktion: ReadBoardIdEepromByte
	Parameter:	Channel: EEP_CH1 oder EEP_CH2
				Address: Adresse im EEPROM (0..16383)

	Rückgabewert: der gelesene Wert

	Zweck: Lesen eines Bytes aus dem EEPROM des Evalboards.
********************************************************************/
uint8 readBoardIdEepromByte(SPIChannelTypeDef *SPIChannel, uint16 Address)
{
	//select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	SPIChannel->readWrite(0x03, FALSE); //Befehl "Read"
	SPIChannel->readWrite(Address >> 8, FALSE);
	SPIChannel->readWrite(Address & 0xFF, FALSE);

	uint8 out = SPIChannel->readWrite(0, TRUE);

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;

	return out;
}


/*******************************************************************
	Funktion: ReadBoardIdEepromBlock
	Parameter:	Channel: EEP_CH1 oder EEP_CH2
				Address: Adresse im EEPROM (0..16383)
				Block: Startadresse des zu lesenden Blocks
				Size: Länge des Blocks in Bytes

	Rückgabewert: ---

	Zweck: Lesen mehrerer Bytes aus dem Konfigurations-EEPROM.
	Dabei dürfen ab beliebiger Adresse beliebig viele Bytes gelesen
	werden.
********************************************************************/
void readBoardIdEepromBlock(SPIChannelTypeDef *SPIChannel, uint16 Address, uint8 *Block, uint16 Size)
{
	uint16 i;

	// select CSN of eeprom
	IOPinTypeDef* io = SPIChannel->CSN;
	if(SPIChannel == &SPI.ch1)
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH0;
	else
		SPIChannel->CSN = &HAL.IOs->pins->ID_CH1;

	IOs.toOutput(SPIChannel->CSN);

	SPIChannel->readWrite(0x03, FALSE); // Befehl "Read"
	SPIChannel->readWrite(Address >> 8, FALSE);
	SPIChannel->readWrite(Address & 0xFF, FALSE);

	for(i = 0; i < Size; i++)
		*(Block+i) = SPIChannel->readWrite(0, i == Size-1); // beim letzten Byte EEPROM deselektieren

	HAL.IOs->config->toInput(SPIChannel->CSN);
	SPIChannel->CSN = io;
}
