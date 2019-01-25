#include "../tmc/idDetection.h"
#include "SelfTest.h"
#include "board.h"

static void deInit();
static uint32 selfTest(uint8 type, uint8 motor, int32 *value);
static void periodicJob(uint32 tick);

IOPinTypeDef *groupA[SELF_TEST_PINS_PER_GROUP];
IOPinTypeDef *groupB[SELF_TEST_PINS_PER_GROUP];

void SelfTest_init()
{
	groupA[0]   = &HAL.IOs->pins->DIO6;
	groupA[1]   = &HAL.IOs->pins->ID_CH1;
	groupA[2]   = &HAL.IOs->pins->DIO1;
	groupA[3]   = &HAL.IOs->pins->DIO3;
	groupA[4]   = &HAL.IOs->pins->DIO5;
	groupA[5]   = &HAL.IOs->pins->DIO8;
	groupA[6]   = &HAL.IOs->pins->DIO10;
	groupA[7]   = &HAL.IOs->pins->CLK16;
	groupA[8]   = &HAL.IOs->pins->SPI2_CSN1;
	groupA[9]   = &HAL.IOs->pins->SPI2_SCK;
	groupA[10]  = &HAL.IOs->pins->SPI2_SDI;
	groupA[11]  = &HAL.IOs->pins->SPI1_SCK;
	groupA[12]  = &HAL.IOs->pins->SPI1_SDO;
	groupA[13]  = &HAL.IOs->pins->DIO13;
	groupA[14]  = &HAL.IOs->pins->DIO15;
	groupA[15]  = &HAL.IOs->pins->DIO17;
	groupA[16]  = &HAL.IOs->pins->DIO19;

	groupB[0]   = &HAL.IOs->pins->ID_CLK;
	groupB[1]   = &HAL.IOs->pins->ID_CH0;
	groupB[2]   = &HAL.IOs->pins->DIO0;
	groupB[3]   = &HAL.IOs->pins->DIO2;
	groupB[4]   = &HAL.IOs->pins->DIO4;
	groupB[5]   = &HAL.IOs->pins->DIO7;
	groupB[6]   = &HAL.IOs->pins->DIO9;
	groupB[7]   = &HAL.IOs->pins->DIO11;
	groupB[8]   = &HAL.IOs->pins->SPI2_CSN0;
	groupB[9]   = &HAL.IOs->pins->SPI2_CSN2;
	groupB[10]  = &HAL.IOs->pins->SPI2_SDO;
	groupB[11]  = &HAL.IOs->pins->SPI1_CSN;
	groupB[12]  = &HAL.IOs->pins->SPI1_SDI;
	groupB[13]  = &HAL.IOs->pins->DIO12;
	groupB[14]  = &HAL.IOs->pins->DIO14;
	groupB[15]  = &HAL.IOs->pins->DIO16;
	groupB[16]  = &HAL.IOs->pins->DIO18;

	VitalSignsMonitor.debugMode = 1;

	Evalboards.ch1.userFunction  = selfTest;
	Evalboards.ch1.deInit        = deInit;
	Evalboards.ch1.periodicJob   = periodicJob;

	EXTI_DeInit();
}

static void deInit()
{
	IDDetection_init();
	VitalSignsMonitor.debugMode = 0;
	IOMap.init();
	HAL.LEDs->error.off();
}

static uint32 selfTest(uint8 type, uint8 motor, int32 *value)
{
	uint32 i;
	int32 result = 0xFFFFFFFF;
	uint32 errors = TMC_ERROR_NONE;
	IOPinTypeDef **inGroup, **outGroup;


	// 1 40 1 0 0 10 C9 45 60
	// 1 40 2 0 0 0 0 0 43
	// 1 40 3 0 0 0 0 0 44
	// 1 40 5 0 0 0 0 0 45
	// 1 40 6 0 0 0 0 0 46

	// 1 40 1 0 0 10 C9 45 60 1 40 2 0 0 0 0 0 43 1 40 3 0 0 0 0 0 44

	switch(type)
	{
	case SELF_TEST_LEAVE:
		deInit();
		*value = 32168;
		break;
	case SELF_TEST_A_OUT_B_IN:
	case SELF_TEST_A_IN_B_OUT:
		inGroup   = (type == SELF_TEST_A_OUT_B_IN) ? groupB : groupA;
		outGroup  = (type == SELF_TEST_A_OUT_B_IN) ? groupA : groupB;

		for(i = 0; i < SELF_TEST_PINS_PER_GROUP; i++)
		{
			HAL.IOs->config->toInput(inGroup[i]);
			HAL.IOs->config->toOutput(outGroup[i]);

			HAL.IOs->config->setHigh(outGroup[i]);
			if(!HAL.IOs->config->isHigh(inGroup[i]))
				result &= (uint32) (~(1<<i));

			HAL.IOs->config->setLow(outGroup[i]);
			if(HAL.IOs->config->isHigh(inGroup[i]))
				result &= (uint32) (~(1<<i));
		}
		result &= ((1<<SELF_TEST_PINS_PER_GROUP) - 1);
		*value = result;
		break;
	case SELF_TEST_READ_AN:
		*value = motor;

		switch(motor)
		{
		case 0:
			*value = (uint32) (((*HAL.ADCs->AIN0)/4095.0)*5.016*10);
			break;
		case 1:
			*value = (uint32) (((*HAL.ADCs->AIN1)/4095.0)*5.016*10);
			break;
		case 2:
			*value = (uint32) (((*HAL.ADCs->AIN2)/4095.0)*5.016*10);
			break;
		case 3:
			*value = VitalSignsMonitor.VM;
			break;
		default:
			errors |= TMC_ERROR_FUNCTION;
			break;
		}
		break;
	case SELF_TEST_SET_AN:
		*value = motor;

		switch(motor)
		{
		case 0:
			HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN1);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN2);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN1);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN2);
			break;
		case 1:
			HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN2);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN2);
			break;
		case 2:
			HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN1);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN1);
			break;
		default:
			errors |= TMC_ERROR_FUNCTION;
			break;
		}
		break;
	case SELF_TEST_SET_AN_2:
		*value = motor;

		switch(motor)
		{
		case 0:
			HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN1);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN2);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN1);
			HAL.IOs->config->setLow(&HAL.IOs->pins->AIN2);
			break;
		case 1:
			HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN2);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->setLow(&HAL.IOs->pins->AIN2);
			break;
		case 2:
			HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->AIN1);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->AIN0);
			HAL.IOs->config->setLow(&HAL.IOs->pins->AIN1);
			break;
		default:
			errors |= TMC_ERROR_FUNCTION;
			break;
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static void periodicJob(uint32 tick)
{
	static uint32 lastTick = 0;

	if((tick - lastTick) >= 500)
	{
		HAL.LEDs->error.toggle();
		lastTick = tick;
	}
}
