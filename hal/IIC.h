/*******************************************************************************
* Copyright © 2025 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef IIC_H_
#define IIC_H_

typedef struct
{
    void (*init) (void);
} IICTypeDef;

extern IICTypeDef IIC;

uint8_t IICMasterWrite(uint8_t address, uint8_t *data, uint8_t size);
uint8_t IICMasterRead(uint8_t address, uint8_t *data, uint8_t size);
uint8_t IICMasterWriteRead(uint8_t address, uint8_t *writeData, uint8_t writeLength, uint8_t *readData, uint8_t readLength);

#endif /*IIC_H_ */
