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
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	ADC_DeInit();

	/* Enable peripheral clocks *************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);
	HAL.IOs->config->reset(&HAL.IOs->pins->VM_MEAS);


	/* DMA2_Stream0 channel0 configuration **************************************/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr  = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr     = (uint32_t)&ADCValue;
	DMA_InitStructure.DMA_DIR                 = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize          = N_O_ADC_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc       = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc           = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize      = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode                = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority            = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst         = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst     = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);


	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode              = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler         = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode     = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay  = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode          = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv      = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion       = N_O_ADC_CHANNELS;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* ADC1 regular channel configuration ******************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_15Cycles);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 **************************************************************/
	ADC_Cmd(ADC1, ENABLE);

	ADC_SoftwareStartConv(ADC1);
}

static void deInit(void)
{
	 ADC_DeInit();
}
