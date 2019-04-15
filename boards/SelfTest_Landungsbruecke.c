#include "Board.h"
#include "SelfTest.h"
#include "tmc/VitalSignsMonitor.h"
#include "tmc/IdDetection.h"

static void deInit();
static uint32_t selfTest(uint8_t type, uint8_t motor, int32_t *value);
static void periodicJob(uint32_t tick);


IOPinTypeDef
	*groupA[SELF_TEST_PINS_PER_GROUP],
	*groupB[SELF_TEST_PINS_PER_GROUP];


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
}

static void deInit()
{
	IDDetection_init();
	VitalSignsMonitor.debugMode = 0;
	IOMap.init();
	HAL.LEDs->error.off();
}

static uint32_t selfTest(uint8_t type, uint8_t motor, int32_t *value)  // Aufrufen des Selftest durch Command 143 , Type 3 und Value 0xFF00FF in dezimal => 16711935
{                                                              // LEDs blinken gleichmäßig
	uint32_t i;
	uint32_t result = 0xFFFFFFFF;
	uint32_t errors = TMC_ERROR_NONE;
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
				result &= (unsigned int) (~(1<<i));

			HAL.IOs->config->setLow(outGroup[i]);
			if(HAL.IOs->config->isHigh(inGroup[i]))
				result &= (unsigned int) (~(1<<i));
		}
		result &= ((1<<SELF_TEST_PINS_PER_GROUP) - 1);
		*value	= result;
		break;
	case SELF_TEST_READ_AN:
		*value = motor;

		switch(motor)
		{
		case 0:
			*value = (unsigned int) (((*HAL.ADCs->AIN0)*50)>>16);  // fullscale @ 5V, 16Bit [V/10]
			break;
		case 1:
			*value = (unsigned int) (((*HAL.ADCs->AIN1)*50)>>16);  // fullscale @ 5V, 16Bit [V/10]
			break;
		case 2:
			*value = (unsigned int) (((*HAL.ADCs->AIN2)*50)>>16);  // fullscale @ 5V, 16Bit [V/10]
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
		case 0:  //AIN0
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO0);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO1);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO0);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO1);
			break;
		case 1:  //AIN1
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO3);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO2);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO3);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO2);
			break;
		case 2:  //AIN2
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO7);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO8);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO7);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO8);
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
		case 0:  //AIN0
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO1);
			HAL.IOs->config->toInput(&HAL.IOs->pins->DIO0);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO1);
//				HAL.IOs->config->setLow(&HAL.IOs->pins->DIO1);
			break;
		case 1:  //AIN1
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO3);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO2);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO3);
			HAL.IOs->config->setLow(&HAL.IOs->pins->DIO2);
			break;
		case 2:  //AIN2
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO7);
			HAL.IOs->config->toOutput(&HAL.IOs->pins->DIO8);

			HAL.IOs->config->setHigh(&HAL.IOs->pins->DIO7);
			HAL.IOs->config->setLow(&HAL.IOs->pins->DIO8);
			break;
		default:
			errors |= TMC_ERROR_FUNCTION;
			break;
		}
		break;
	case SELF_TEST_SET_MIXED:
		*value = motor;

		switch(motor)
		{
		case 0:
			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED0);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED0);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED1);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED1);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED2);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED2);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED3);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED3);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED4);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED4);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED5);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED5);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED6);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->MIXED6);
			break;
		case 1:
			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED0);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED0);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED1);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED1);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED2);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED2);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED3);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED3);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED4);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED4);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED5);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED5);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->MIXED6);
			HAL.IOs->config->setLow(&HAL.IOs->pins->MIXED6);
			break;
		default:
			errors |= TMC_ERROR_FUNCTION;
			break;
		}
		break;
	case SELF_TEST_SET_EXTIO:
		*value = motor;

		switch(motor)
		{
		case 0:
			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_2);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->EXTIO_2);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_3);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->EXTIO_3);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_4);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->EXTIO_4);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_5);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->EXTIO_5);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_6);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->EXTIO_6);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_7);
			HAL.IOs->config->setHigh(&HAL.IOs->pins->EXTIO_7);
			break;
		case 1:
			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_2);
			HAL.IOs->config->setLow(&HAL.IOs->pins->EXTIO_2);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_3);
			HAL.IOs->config->setLow(&HAL.IOs->pins->EXTIO_3);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_4);
			HAL.IOs->config->setLow(&HAL.IOs->pins->EXTIO_4);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_5);
			HAL.IOs->config->setLow(&HAL.IOs->pins->EXTIO_5);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_6);
			HAL.IOs->config->setLow(&HAL.IOs->pins->EXTIO_6);

			HAL.IOs->config->toOutput(&HAL.IOs->pins->EXTIO_7);
			HAL.IOs->config->setLow(&HAL.IOs->pins->EXTIO_7);
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

static void periodicJob(uint32_t tick)
{
	static uint32_t lastTick = 0;

	if((tick - lastTick) >= 500)
	{
		HAL.LEDs->error.toggle();
		lastTick = tick;
	}
}
