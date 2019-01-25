#ifndef _IO_PIN_MAP_H_
	#define _IO_PIN_MAP_H_

	#include "IOs.h"

	typedef struct
	{
		void (*init) (void);

#if defined(Startrampe)
		IOPinTypeDef VM_MEAS;
		IOPinTypeDef AIN0;
		IOPinTypeDef AIN1;
		IOPinTypeDef AIN2;
#endif

		IOPinTypeDef ID_CLK;
		IOPinTypeDef ID_CH0;
		IOPinTypeDef ID_CH1;

		IOPinTypeDef DIO0;
		IOPinTypeDef DIO1;
		IOPinTypeDef DIO2;
		IOPinTypeDef DIO3;
		IOPinTypeDef DIO4;
		IOPinTypeDef DIO5;

		IOPinTypeDef DIO6;
		IOPinTypeDef DIO7;
		IOPinTypeDef DIO8;
		IOPinTypeDef DIO9;
		IOPinTypeDef DIO10;
		IOPinTypeDef DIO11;

		IOPinTypeDef CLK16;

		IOPinTypeDef SPI2_CSN0;
		IOPinTypeDef SPI2_CSN1;
		IOPinTypeDef SPI2_CSN2;
		IOPinTypeDef SPI2_SCK;
		IOPinTypeDef SPI2_SDO;
		IOPinTypeDef SPI2_SDI;

		IOPinTypeDef SPI1_CSN;
		IOPinTypeDef SPI1_SCK;
		IOPinTypeDef SPI1_SDI;
		IOPinTypeDef SPI1_SDO;

		IOPinTypeDef DIO12;
		IOPinTypeDef DIO13;
		IOPinTypeDef DIO14;
		IOPinTypeDef DIO15;
		IOPinTypeDef DIO16;
		IOPinTypeDef DIO17;
		IOPinTypeDef DIO18;
		IOPinTypeDef DIO19;

		IOPinTypeDef WIRELESS_TX;
		IOPinTypeDef WIRELESS_RX;
		IOPinTypeDef WIRELESS_NRST;

		IOPinTypeDef RS232_TX;
		IOPinTypeDef RS232_RX;

		IOPinTypeDef USB_V_BUS;
		IOPinTypeDef USB_V_DM;
		IOPinTypeDef USB_V_DP;

		IOPinTypeDef LED_STAT;
		IOPinTypeDef LED_ERROR;

		IOPinTypeDef EXTIO_2;
		IOPinTypeDef EXTIO_3;
		IOPinTypeDef EXTIO_4;
		IOPinTypeDef EXTIO_5;
		IOPinTypeDef EXTIO_6;
		IOPinTypeDef EXTIO_7;

		IOPinTypeDef EEPROM_SCK;
		IOPinTypeDef EEPROM_SI;
		IOPinTypeDef EEPROM_SO;
		IOPinTypeDef EEPROM_NCS;

		IOPinTypeDef MIXED0;
		IOPinTypeDef MIXED1;
		IOPinTypeDef MIXED2;
		IOPinTypeDef MIXED3;
		IOPinTypeDef MIXED4;
		IOPinTypeDef MIXED5;
		IOPinTypeDef MIXED6;

#if defined(Landungsbruecke) // HWID detection for Landungsbruecke v2.0+
		IOPinTypeDef ID_HW_0;
		IOPinTypeDef ID_HW_1;
		IOPinTypeDef ID_HW_2;
#endif

		IOPinTypeDef DUMMY;
	} IOPinMapTypeDef;

	IOPinMapTypeDef IOMap;

#endif /* _IO_PIN_MAP_H_ */
