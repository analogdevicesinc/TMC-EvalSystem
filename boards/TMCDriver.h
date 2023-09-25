/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef TMCDRIVER_H_
#define TMCDRIVER_H_

	#include "Board.h"

	typedef struct
	{
		ConfigurationTypeDef config;
	} EvalBoardDriverTypeDef;

	extern EvalBoardDriverTypeDef TMCDriver;

	void tmcdriver_init();

#endif /* TMCDRIVER_H_ */
