/*
 * IOs.c
 *
 *  Created on: 29.03.2019
 *      Author: LK
 */

#include "IOs.h"

static IO_States getPinState(IOPinTypeDef *pin);

static IO_States getPinState(IOPinTypeDef *pin)
{
	return pin->state;
}
