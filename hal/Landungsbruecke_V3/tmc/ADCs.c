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
//	ADC_InitTypeDef ADC_InitStructure;
//	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	dma_multi_data_parameter_struct DMA_InitStructure;

	adc_deinit();

	/* Enable peripheral clocks *************************************************/
    rcu_periph_clock_enable(RCU_DMA1);
    rcu_periph_clock_enable(RCU_ADC1);


//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);
	HAL.IOs->config->reset(&HAL.IOs->pins->VM_MEAS);


	/* DMA2_Stream0 channel0 configuration **************************************/
	dma_deinit(DMA1, DMA_CH0);

//	DMA_DeInit(DMA2_Stream0);
//	DMA_InitStructure.DMA_Channel             = DMA_Channel_0;
	DMA_InitStructure.periph_addr  = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.periph_width  = DMA_PERIPH_WIDTH_16BIT;
	DMA_InitStructure.periph_inc       = DMA_PERIPH_INCREASE_DISABLE;

	DMA_InitStructure.memory0_addr     = (uint32_t)&ADCValue;
	DMA_InitStructure.memory_width      = DMA_MEMORY_WIDTH_16BIT;
	DMA_InitStructure.memory_inc           = DMA_MEMORY_INCREASE_ENABLE;

	DMA_InitStructure.memory_burst_width         = DMA_MEMORY_BURST_SINGLE;
	DMA_InitStructure.periph_burst_width     = DMA_PERIPH_BURST_SINGLE;

	DMA_InitStructure.circular_mode                = DMA_CIRCULAR_MODE_ENABLE;
	DMA_InitStructure.direction                 = DMA_PERIPH_TO_MEMORY;
	DMA_InitStructure.priority            = DMA_PRIORITY_HIGH;

	DMA_InitStructure.DMA_BufferSize          = N_O_ADC_CHANNELS;
	DMA_InitStructure.DMA_FIFOMode            = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold       = DMA_FIFOThreshold_HalfFull;
//	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	dma_multi_data_mode_init(DMA1, DMA_CH0, &DMA_InitStructure);

	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);


	/* ADC Common Init **********************************************************/
	adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
//	ADC_CommonInitStructure.ADC_Mode              = ADC_Mode_Independent;

	ADC_CommonInitStructure.ADC_Prescaler         = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode     = ADC_DMAAccessMode_Disabled;

	adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);
//	ADC_CommonInitStructure.ADC_TwoSamplingDelay  = ADC_TwoSamplingDelay_5Cycles;

//	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
//	ADC_InitStructure.ADC_Resolution            = ADC_Resolution_12b;

	adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);
//	ADC_InitStructure.ADC_ScanConvMode          = ENABLE;

	adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
//	ADC_InitStructure.ADC_ContinuousConvMode    = ENABLE;

	adc_external_trigger_config(EXTERNAL_TRIGGER_DISABLE);
//	ADC_InitStructure.ADC_ExternalTrigConvEdge  = ADC_ExternalTrigConvEdge_None;

	adc_external_trigger_source_config(ADC1, , ADC_EXTTRIG_ROUTINE_T1_CH1);
//	ADC_InitStructure.ADC_ExternalTrigConv      = ADC_ExternalTrigConv_T1_CC1;

	adc_data_alignment_config(ADC1,ADC_DATAALIGN_RIGHT);
//	ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;

	ADC_InitStructure.ADC_NbrOfConversion       = N_O_ADC_CHANNELS;
//	ADC_Init(ADC1, &ADC_InitStructure);

	/* Enable ADC1 DMA */
	adc_dma_mode_enable(ADC1);
//	ADC_DMACmd(ADC1, ENABLE);

	/* ADC1 regular channel configuration ******************************/
	adc_routine_channel_config(ADC1, 0, ADC_CHANNEL_7, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC1, 1, ADC_CHANNEL_8, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC1, 2, ADC_CHANNEL_9, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC1, 3, ADC_CHANNEL_4, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC1, 4, ADC_CHANNEL_5, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC1, 5, ADC_CHANNEL_12, ADC_SAMPLETIME_15);

//	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_15Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_15Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 3, ADC_SampleTime_15Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_15Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_15Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_15Cycles);

	/* Enable DMA request after last transfer (Single-ADC mode) */
	adc_dma_request_after_last_enable(ADC1);
//	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 **************************************************************/
	adc_enable(ADC1);
//	ADC_Cmd(ADC1, ENABLE);

	adc_software_trigger_enable(ADC1, ADC_ROUTINE_CHANNEL);
//	ADC_SoftwareStartConv(ADC1);
}

static void deInit(void)
{
	adc_deinit();
}
