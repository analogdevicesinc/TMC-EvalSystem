#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define N_O_ADC_CHANNELS 6

static volatile uint16_t ADCValue[N_O_ADC_CHANNELS];

typedef struct
{
	volatile uint16_t *AIN0;
	volatile uint16_t *AIN1;
	volatile uint16_t *AIN2;
	volatile uint16_t *DIO4;
	volatile uint16_t *DIO5;
	volatile uint16_t *VM;
	void (*init)();
	void (*deInit)();
} ADCTypeDef;

extern ADCTypeDef ADCs;

#endif /* ADC_H */
