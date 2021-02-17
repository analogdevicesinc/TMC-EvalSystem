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
