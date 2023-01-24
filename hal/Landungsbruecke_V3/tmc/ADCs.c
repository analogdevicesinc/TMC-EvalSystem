#include <math.h>
#include "hal/HAL.h"
#include "hal/ADCs.h"

#define ADC1_DR_ADDRESS  ((uint32_t)0x4001204C)

static void init(void);
static void deInit(void);

ADCTypeDef ADCs =
{
	.AIN0   = &ADCValue[0],
	.AIN1   = &ADCValue[1],
	.AIN2   = &ADCValue[2],
	.DIO4   = &ADCValue[3],
	.DIO5   = &ADCValue[4],
	.VM     = &ADCValue[5],
	.init   = init,
	.deInit  = deInit
};

void init(void)
{
	
}

static void deInit(void)
{

}
