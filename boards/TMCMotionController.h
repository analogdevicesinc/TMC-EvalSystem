/*******************************************************************************
* Copyright © 2019 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2023 Analog Devices Inc. All Rights Reserved. This software is
* proprietary & confidential to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#ifndef TMCMOTIONCONTROLLER_H_
#define TMCMOTIONCONTROLLER_H_

	#include "Board.h"

	typedef struct
	{
		ConfigurationTypeDef config;
	} EvalBoardMotionControllerTypeDef;

	extern EvalBoardMotionControllerTypeDef TMCMotionController;

	void tmcmotioncontroller_init();

#endif /* TMCMOTIONCONTROLLER_H_ */
