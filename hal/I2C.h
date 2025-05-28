/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef I2C_H_
#define I2C_H_

typedef struct
{
    void (*init) (void);
} I2CTypeDef;

extern I2CTypeDef I2C;

uint8_t I2CMasterWrite(uint8_t address, uint8_t *data, uint8_t size);
uint8_t I2CMasterRead(uint8_t address, uint8_t *data, uint8_t size);
uint8_t I2CMasterWriteRead(uint8_t address, uint8_t *writeData, uint8_t writeLength, uint8_t *readData, uint8_t readLength);

#endif /*I2C_H_ */
