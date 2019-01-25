#include "../hal/derivative.h"
#include "../hal/HAL.h"

#include "TMCL.h"
#include "IdDetection.h"
#include "BoardAssignment.h"

static uint8 assignCh1(uint8 id, uint8 justCheck);
static uint8 assignCh2(uint8 id, uint8 justCheck);
static void hookDriverSPI(IdAssignmentTypeDef *ids);
static void unassign(IdAssignmentTypeDef *ids);

int32 Board_assign(IdAssignmentTypeDef *ids)
{
	int32 out = 0;

	// Test mode // todo REM 2: still needed? (LH)
	if((ids->ch1.id == 0xFF) || (ids->ch2.id == 0xFF))
	{
		if((Evalboards.ch1.id != 0) || (Evalboards.ch2.id != 0) || (ids->ch1.id != ids->ch2.id))
		{
			ids->ch1.state = ID_STATE_NOT_IN_FW;
			ids->ch2.state = ID_STATE_NOT_IN_FW;
			out |= ids->ch2.state  << 24;
			out |= ids->ch2.id     << 16;
			out |= ids->ch1.state  << 8;
			out |= ids->ch1.id     << 0;
			return out;
		}
	}

	// Assign motion controller
	if((Evalboards.ch1.id == ids->ch1.id) && (ids->ch1.id != 0))
	{	// todo CHECK 2: Evalboards.ch_.id only gets written at the end of this function - so the only way we can reach this case by calling this function multiple times.
		//               Therefor, the else case always runs before, which means any information returned by the justCheck = true call here would have already been
		//               given by the previous call of this function. This entire ID detection procedure is kinda messy, maybe we can actually completely rework it (LH)
		ids->ch1.state = assignCh1(ids->ch1.id, TRUE);
	}
	else
	{
		Evalboards.ch1.deInit(); // todo REM 2: Hot-Unplugging is not maintained currently, should probably be removed (LH) #1
		if(ids->ch1.state == ID_STATE_DONE)
			ids->ch1.state = assignCh1(ids->ch1.id, FALSE);
		Evalboards.ch1.config->reset();
	}

	// Assign driver
	if((Evalboards.ch2.id == ids->ch2.id) && (ids->ch2.id != 0))
	{
		ids->ch2.state = assignCh2(ids->ch2.id, TRUE);
	}
	else
	{
		Evalboards.ch2.deInit(); // todo REM 2: Hot-Unplugging is not maintained currently, should probably be removed (LH) #2
		if(ids->ch2.state == ID_STATE_DONE)
			ids->ch2.state = assignCh2(ids->ch2.id, FALSE);
		Evalboards.ch2.config->reset();
	}

	// Reroute SPI 2 (that the driver uses) to run through the motion controller if required
	// This allows the chaining of a motion controller and a driver.
	// Note that the motion controller has to invoke reset() or restore() of the driver
	// in order to have settings sent through the hooked SPI.
	// This is currently done on completed motion controller reset/restore
	hookDriverSPI(ids);

	Evalboards.ch1.id = ids->ch1.id;
	Evalboards.ch2.id = ids->ch2.id;

	out |= (ids->ch2.state  << 24) & 0xFF;
	out |= (ids->ch2.id     << 16) & 0xFF;
	out |= (ids->ch1.state  << 8)  & 0xFF;
	out |= (ids->ch1.id     << 0)  & 0xFF;

	return out;
}

int32 Board_supported(IdAssignmentTypeDef *ids)
{
	int32 out = 0;

	ids->ch1.state = assignCh1(ids->ch1.id, TRUE);
	ids->ch2.state = assignCh2(ids->ch2.id, TRUE);

	out |= ids->ch2.state  << 24;
	out |= ids->ch2.id     << 16;
	out |= ids->ch1.state  << 8;
	out |= ids->ch1.id     << 0;

	return out;
}

static uint8 assignCh1(uint8 id, uint8 justCheck)
{
	uint8 ok = ID_STATE_NOT_IN_FW;
	if(!justCheck)
		tmcmotioncontroller_init();

	for(size_t i = 0, sz = ARRAY_SIZE(init_ch1); i < sz; i++)
	{
		if(init_ch1[i].id == id)
		{
			if(!justCheck)
				init_ch1[i].init();
			ok = ID_STATE_DONE;
			break;
		}
	}

	return ok;
}

static uint8 assignCh2(uint8 id, uint8 justCheck)
{
	uint8 ok = ID_STATE_NOT_IN_FW;

//	if(!justCheck)
//		tmcdriver_init();

	for(size_t i = 0, sz = ARRAY_SIZE(init_ch2); i < sz; i++)
	{
		if(init_ch2[i].id == id)
		{
			if(!justCheck)
				init_ch2[i].init();
			ok = ID_STATE_DONE;
			break;
		}
	}

	return ok;
}

// Reroute the driver's SPI to run through the motion controller if required
// This also handles special case logic for the motion controller + driver chain (different pins etc.)
static void hookDriverSPI(IdAssignmentTypeDef *ids)
{
	if((ids->ch1.id == ID_TMC4361) || (ids->ch1.id == ID_TMC4361A) || (ids->ch1.id == ID_TMC4331))
	{
		// Redirect ch2 SPI to the SPI cover function of the TMC43XX Board
		HAL.SPI->ch2.readWrite = Evalboards.ch1.cover;

		if(ids->ch2.id == ID_TMC2660 ||
				ids->ch2.id == ID_TMC262_1420)
		{
			// TMC2660: Disable the continuous mode via userFunction
			int32 value = 1;
			Evalboards.ch2.userFunction(0, 0, &value);
		}
	}


	if(ids->ch1.id == ID_TMC4330)
	{
		if(ids->ch2.id == ID_TMC2100)
		{
			Evalboards.ch2.userFunction(1, 0, NULL);
		}
		else if(ids->ch2.id == ID_TMC2130)
		{
			Evalboards.ch2.userFunction(5, 0, NULL);
		}
		else if(ids->ch2.id == ID_TMC2160)
		{
			Evalboards.ch2.userFunction(5, 0, NULL);
		}
	}
}

static void unassign(IdAssignmentTypeDef *ids)
{
	UNUSED(ids);
}

void periodicJob(unsigned int tick)
{
	UNUSED(tick);
}

void deInit(void)
{
	IdAssignmentTypeDef ids;
	Evalboards.ch1.deInit();
	Evalboards.ch2.deInit();

	ids.ch1.id = Evalboards.ch1.id;
	ids.ch2.id = Evalboards.ch2.id;
	unassign(&ids);

	tmcdriver_init();
	tmcmotioncontroller_init();
}
