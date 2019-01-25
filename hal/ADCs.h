#ifndef ADC_H
#define ADC_H

	#define N_O_ADC_CHANNELS 6

	static volatile unsigned short ADCValue[N_O_ADC_CHANNELS];

	typedef struct
	{
		volatile unsigned short *AIN0;
		volatile unsigned short *AIN1;
		volatile unsigned short *AIN2;
		volatile unsigned short *DIO4;
		volatile unsigned short *DIO5;
		volatile unsigned short *VM;
		void (*init)();
		void (*deInit)();
	} ADCTypeDef;

	ADCTypeDef ADCs;

#endif /* ADC_H */
