#ifndef SPI_H
#define SPI_H

	#include "derivative.h"
	#include "IOs.h"

	typedef struct
	{
		#ifdef Startrampe
			SPI_TypeDef *periphery; // pointer to ST SPI configuration structure
		#else
			SPI_MemMapPtr periphery; // pointer to freescale SPI memory base pointer
		#endif

		IOPinTypeDef *CSN;
		unsigned char (*readWrite) (unsigned char data, unsigned char lastTransfer);
		void (*readWriteArray) (uint8 *data, size_t length);
		void (*reset) (void);
	} SPIChannelTypeDef;

	typedef struct
	{
		SPIChannelTypeDef ch1;
		SPIChannelTypeDef ch2;
		void (*init) (void);
	} SPITypeDef;

	SPITypeDef SPI;

	// read/write 32 bit value at address
	int32 spi_readInt(SPIChannelTypeDef *SPIChannel, uint8 address);
	void spi_writeInt(SPIChannelTypeDef *SPIChannel, uint8 address, int value);

	// for default channels
	uint8 spi_ch1_readWriteByte(uint8 data, uint8 lastTransfer);

	int32 spi_ch1_readInt(uint8 address);
	void spi_ch1_writeInt(uint8 address, int value);

	int32 spi_ch2_readInt(uint8 address);
	void spi_ch2_writeInt(uint8 address, int value);

#endif /* SPI_H */
