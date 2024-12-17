/*******************************************************************************
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef IIC_H
#define IIC_H

typedef struct
{
    void (*init) (void);
} IICTypeDef;

extern IICTypeDef IIC;

void InitIIC(void);
uint8_t IICMasterWrite(uint8_t Address, uint8_t *Data, uint8_t Size);
uint8_t IICMasterRead(uint8_t Address, uint8_t *Data, uint8_t Size);
uint8_t IICMasterWriteRead(uint8_t Address, uint8_t *WriteData, uint8_t WriteLength, uint8_t *ReadData, uint8_t ReadLength);

#endif /*IIC_H */
