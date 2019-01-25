#ifndef TMCDRIVER_H_
#define TMCDRIVER_H_

	#include "Board.h"

	typedef struct
	{
		ConfigurationTypeDef config;
	} EvalBoardDriverTypeDef;

	EvalBoardDriverTypeDef TMCDriver;

	void tmcdriver_init();

#endif /* TMCDRIVER_H_ */
