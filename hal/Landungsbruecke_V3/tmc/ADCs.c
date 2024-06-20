/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include <math.h>
#include "hal/HAL.h"
#include "hal/ADCs.h"

#define ADC1_DR_ADDRESS  ((uint32_t)0x4001204C)

static void init(void);
static void deInit(void);

ADCTypeDef ADCs =
{
	.AIN0    = &ADCValue[0],
	.AIN1    = &ADCValue[1],
	.AIN2    = &ADCValue[2],
	.DIO4    = &ADCValue[3],
	.DIO5    = &ADCValue[4],
	.VM      = &ADCValue[5],
	.AIN_EXT = &ADCValue[6],
	.init    = init,
	.deInit  = deInit,
};

void init(void)
{
	adc_deinit();

	rcu_periph_clock_enable(RCU_DMA1);
	rcu_periph_clock_enable(RCU_ADC0);

	HAL.IOs->config->reset(&HAL.IOs->pins->AIN0);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN1);
	HAL.IOs->config->reset(&HAL.IOs->pins->AIN2);
	HAL.IOs->config->reset(&HAL.IOs->pins->ADC_VM);

	dma_deinit(DMA1, DMA_CH0);

	dma_multi_data_parameter_struct dma_init_struct;
	dma_multi_data_para_struct_init(&dma_init_struct);
	dma_init_struct.periph_addr = (uint32_t) (ADC0 + 0x4CU);
	dma_init_struct.periph_width = DMA_PERIPH_WIDTH_16BIT;
	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_init_struct.memory0_addr = (uint32_t)&ADCValue[0];
	dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_init_struct.memory_burst_width = DMA_MEMORY_BURST_SINGLE;
	dma_init_struct.periph_burst_width = DMA_PERIPH_BURST_SINGLE;
	dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
	dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
	dma_init_struct.number = N_O_ADC_CHANNELS;
	dma_init_struct.critical_value = DMA_FIFO_2_WORD;
	dma_init_struct.priority = DMA_PRIORITY_HIGH;
	dma_multi_data_mode_init(DMA1, DMA_CH0, &dma_init_struct);

	dma_channel_enable(DMA1, DMA_CH0);
	
	adc_clock_config(ADC_ADCCK_PCLK2_DIV2);
	adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
	adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);
	adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
	adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
	adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
	adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
	adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
	adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, N_O_ADC_CHANNELS);
	adc_dma_mode_enable(ADC0);

	adc_routine_channel_config(ADC0, 0, ADC_CHANNEL_14, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC0, 1, ADC_CHANNEL_15, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC0, 2, ADC_CHANNEL_8, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC0, 3, ADC_CHANNEL_0, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC0, 4, ADC_CHANNEL_1, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC0, 5, ADC_CHANNEL_3, ADC_SAMPLETIME_15);
	adc_routine_channel_config(ADC0, 6, ADC_CHANNEL_2, ADC_SAMPLETIME_15);

	adc_dma_request_after_last_enable(ADC0);

	adc_enable(ADC0);

	adc_calibration_enable(ADC0);

	adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
}

static void deInit(void)
{
	adc_deinit();
}
