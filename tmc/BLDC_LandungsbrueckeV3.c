/*******************************************************************************
* Copyright © 2023 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/


#include "BLDC.h"
#include "hal/HAL.h"
#include "hal/Timer.h"
#include "hal/ADCs.h"

#define VELOCITY_CALC_FREQ  10                     // in Hz
#define PWM_FREQ 		    20000                  // in Hz

	/* Timer0 Frequency:
	 *
	 * CK_TIMER0 = 2 x CK_APB2 = 240MHz (APB1PSC/APB2PSC in RCU_CFG0 register is 0b101)
	 *
	 *   CLK_TIMER      240MHz
	 *  -----------  =  ----- = 80MHz
	 *   Prescaler        3
	 *
	 */
#define PWM_PERIOD 		    ((80000000 / PWM_FREQ)-1)

typedef enum {
	PWM_PHASE_U = TIMER_CH_0,
	PWM_PHASE_V = TIMER_CH_1,
	PWM_PHASE_W = TIMER_CH_2,
} PWM_Phase;

typedef enum {
	ADC_PHASE_U,
	ADC_PHASE_V,
	ADC_PHASE_W,
} ADC_Channel;

uint8_t adcPhases[3] = { 0 };
uint8_t adcCount = 3;
volatile ADC_Channel adc = ADC_PHASE_U;

int16_t  targetPWM        = 0;
uint32_t openloopVelocity = 60; // mechanical RPM
uint16_t openloopStepTime = 0;  // Calculate on init
BLDCMode commutationMode  = BLDC_OPENLOOP;
uint8_t  pwmEnabled       = 0;
uint8_t  bbmTime          = 50;
uint8_t  motorPolePairs   = 1;

int32_t targetAngle         = 0;
int32_t hallAngle           = 0;

int32_t actualHallVelocity = 0; // electrical RPM

volatile int32_t adcSamples[8] = { 0 };
uint8_t adcSampleIndex = 0;

int32_t adcOffset[3]    = { 0 };
uint32_t sampleCount[3] = { 0 };

int32_t currentScalingFactor = 256; // u24q8 format

#define ADC_SAMPLES 100

typedef enum {
	HALL_INVALID_0 = 0,

	HALL_001 = 1,
	HALL_010 = 2,
	HALL_011 = 3,
	HALL_100 = 4,
	HALL_101 = 5,
	HALL_110 = 6,

	HALL_INVALID_1 = 7,
} HallStates;

static int16_t hallStateToAngle(HallStates hallState);
static HallStates inputToHallState(uint8_t in_0, uint8_t in_1, uint8_t in_2);

// Hall parameters
uint8_t hallOrder = 0;
uint8_t hallInvert = 0;

volatile enum {
	ADC_INIT,
	ADC_READY
} adcState[3] = { ADC_INIT, ADC_INIT, ADC_INIT };

typedef struct
{
	IOPinTypeDef  *HALL_U;
	IOPinTypeDef  *HALL_V;
	IOPinTypeDef  *HALL_W;
	IOPinTypeDef  *PWM_UL;
	IOPinTypeDef  *PWM_UH;
	IOPinTypeDef  *PWM_VL;
	IOPinTypeDef  *PWM_VH;
	IOPinTypeDef  *PWM_WL;
	IOPinTypeDef  *PWM_WH;
} PinsTypeDef;

static PinsTypeDef Pins;

static HallStates inputToHallState(uint8_t in_0, uint8_t in_1, uint8_t in_2)
{
	uint8_t tmp;
	HallStates retVal = HALL_INVALID_0;

	if (hallInvert)
	{
		// Swap in_1 and in_2
		tmp = in_1;
		in_1 = in_2;
		in_2 = tmp;
	}

	switch(hallOrder)
	{
	case 0: // U/V/W
		retVal = in_0 << 0
		       | in_1 << 1
		       | in_2 << 2;
		break;
	case 1: // V/W/U
		retVal = in_0 << 1
		       | in_1 << 2
		       | in_2 << 0;
		break;
	case 2: // W/U/V
		retVal = in_0 << 2
		       | in_1 << 0
		       | in_2 << 1;
		break;
	}

	return retVal;
}

static int16_t hallStateToAngle(HallStates hallState)
{
	switch (hallState)
	{
	case HALL_001:
		return 0;
		break;
	case HALL_011:
		return 60;
		break;
	case HALL_010:
		return 120;
		break;
	case HALL_110:
		return 180;
		break;
	case HALL_100:
		return 240;
		break;
	case HALL_101:
		return 300;
		break;
	default:
		break;
	}

	return 0;
}

void BLDC_init(BLDCMeasurementType type, uint32_t currentScaling, IOPinTypeDef *hallU, IOPinTypeDef *hallV, IOPinTypeDef *hallW)
{
	if (type == MEASURE_THREE_PHASES)
	{
		// Three phase measurement
		adcCount = 3;
		adcPhases[0] = ADC_CHANNEL_14; // AIN0
		adcPhases[1] = ADC_CHANNEL_15; // AIN1
		adcPhases[2] = ADC_CHANNEL_8; // AIN2
	}
	else if (type == MEASURE_ONE_PHASE)
	{
		// One phase measurement
		adcCount = 1;
		adcPhases[0] = ADC_CHANNEL_8; // AIN2
	}

	//Set MUX_1 and MUX_2 to one to connect DIO10 and DIO11 to PWM pins DIO10_A and DIO11_A respectively.
//	*HAL.IOs->pins->SW_UART_PWM.setBitRegister     = HAL.IOs->pins->SW_UART_PWM.bitWeight;
	HAL.IOs->config->toOutput(&HAL.IOs->pins->SW_UART_PWM);

	HAL.IOs->config->setHigh(&HAL.IOs->pins->SW_UART_PWM);


	Pins.PWM_UL       = &HAL.IOs->pins->DIO6;
	Pins.PWM_UH       = &HAL.IOs->pins->DIO7;
	Pins.PWM_VL       = &HAL.IOs->pins->DIO8;
	Pins.PWM_VH       = &HAL.IOs->pins->DIO9;
	Pins.PWM_WL       = &HAL.IOs->pins->DIO10_PWM_WL;
	Pins.PWM_WH       = &HAL.IOs->pins->DIO11_PWM_WH;

	currentScalingFactor = currentScaling;

	Pins.HALL_U       = hallU;
	Pins.HALL_V       = hallV;
	Pins.HALL_W       = hallW;

	HAL.IOs->config->toInput(Pins.HALL_U);
	HAL.IOs->config->toInput(Pins.HALL_V);
	HAL.IOs->config->toInput(Pins.HALL_W);

	// Calculate the openloop step time by setting the velocity
	BLDC_setTargetOpenloopVelocity(openloopVelocity);

	// ADC
	// Operation mode: Continuous mode on single selected channel

	rcu_periph_clock_enable(RCU_ADC1);

	adc_clock_config(ADC_ADCCK_PCLK2_DIV2);
	adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
	adc_sync_delay_config(ADC_SYNC_DELAY_5CYCLE);
	adc_resolution_config(ADC1, ADC_RESOLUTION_12B);
	adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
	adc_external_trigger_config(ADC1, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
	adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
	adc_channel_length_config(ADC1, ADC_ROUTINE_CHANNEL, 1);

	adc_routine_channel_config(ADC1, 0, adcPhases[adc], ADC_SAMPLETIME_15);

	adc_interrupt_enable(ADC1, ADC_INT_EOC);
	nvic_irq_enable(ADC_IRQn, 0, 1);

	adc_enable(ADC1);

	adc_calibration_enable(ADC1);

	adc_software_trigger_enable(ADC1, ADC_ROUTINE_CHANNEL);

	// Timer

	rcu_periph_clock_enable(RCU_TIMER0);

	timer_parameter_struct params;
	timer_oc_parameter_struct oc_params;
	timer_break_parameter_struct break_params;
	timer_deinit(TIMER0);

	timer_struct_para_init(&params);
	params.prescaler = 2; // Divides the timer freq by (prescaler + 1) => 3
	params.alignedmode = TIMER_COUNTER_EDGE;
	params.counterdirection = TIMER_COUNTER_UP;
	params.period = PWM_PERIOD;
	params.clockdivision = TIMER_CKDIV_DIV1;
	params.repetitioncounter = 1;
	timer_init(TIMER0, &params);



	timer_channel_output_struct_para_init(&oc_params);
	oc_params.ocpolarity = TIMER_OC_POLARITY_HIGH;
	oc_params.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
	oc_params.outputstate = TIMER_CCX_ENABLE;
	oc_params.outputnstate = TIMER_CCXN_ENABLE;
	oc_params.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
	oc_params.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;
	timer_channel_output_config(TIMER0, TIMER_CH_0, &oc_params);
	timer_channel_output_config(TIMER0, TIMER_CH_1, &oc_params);
	timer_channel_output_config(TIMER0, TIMER_CH_2, &oc_params);

	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, PWM_PERIOD >> 1);
	timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);

	timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, PWM_PERIOD >> 1);
	timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);

	timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

	timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, PWM_PERIOD >> 1);
	timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);

	timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
	
	timer_break_struct_para_init(&break_params);
	break_params.runoffstate     = TIMER_ROS_STATE_ENABLE;
	break_params.ideloffstate    = TIMER_IOS_STATE_ENABLE;
	break_params.deadtime = bbmTime;
	break_params.breakpolarity   = TIMER_BREAK_POLARITY_HIGH;
	break_params.outputautostate = TIMER_OUTAUTO_ENABLE;
	break_params.breakstate = TIMER_BREAK_DISABLE;
	timer_break_config(TIMER0, &break_params);


	timer_primary_output_config(TIMER0, ENABLE);

	timer_auto_reload_shadow_enable(TIMER0);

	timer_interrupt_enable(TIMER0, TIMER_INT_UP);
	timer_update_event_enable(TIMER0);
	nvic_irq_enable(TIMER0_UP_TIMER9_IRQn, 0, 1);
	timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);

	timer_enable(TIMER0);
}

void BLDC_calibrateADCs()
{
	// Only do the offset compensation if PWM is off
	if (targetPWM != 0)
		return;

	// Temporarily disable the FTM interrupt to prevent it from overriding
	// the adc phase selection
	nvic_irq_disable(TIMER0_UP_TIMER9_IRQn);

	for (uint8_t myadc = 0; myadc < adcCount; myadc++)
	{
		adc = myadc % 3;
		// Select the ADC phase for offset compensation
		adc_routine_channel_config(ADC1, 0, adcPhases[adc], ADC_SAMPLETIME_15);

		// Reset the ADC state
		adcOffset[adc] = 0;
		adcState[adc] = ADC_INIT;

		// Wait until the ADC is initialized again
		while (adcState[adc] == ADC_INIT);
	}

	nvic_irq_enable(TIMER0_UP_TIMER9_IRQn, 0, 1);
}

void ADC_IRQHandler()
{
	if(adc_interrupt_flag_get(ADC1, ADC_INT_FLAG_EOC) == SET)
	{
		uint16_t tmp = adc_routine_data_read(ADC1);
		static ADC_Channel lastChannel = ADC_PHASE_U;

		switch(adcState[lastChannel])
		{
		case ADC_INIT:
			if (sampleCount[lastChannel] < ADC_SAMPLES)
			{
				// Store a calibration sample
				adcOffset[lastChannel] += tmp;

				sampleCount[lastChannel]++;
			}
			else
			{
				// Finished collection of calibration samples
				// Calculate offset
				adcOffset[lastChannel] /= ADC_SAMPLES;

				adcState[lastChannel] = ADC_READY;
				sampleCount[lastChannel] = 0;
			}
			break;
		case ADC_READY:
			adcSamples[adcSampleIndex] = (tmp - adcOffset[lastChannel]) * currentScalingFactor / 65536;
			adcSampleIndex = (adcSampleIndex + 1) % ARRAY_SIZE(adcSamples);

			break;
		}

		if (lastChannel != adc)
		{
			// Update the channel
			lastChannel = adc;

			adc_routine_channel_config(ADC1, 0, adcPhases[adc], ADC_SAMPLETIME_15);
		}

		adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
	}
}

void TIMER0_UP_TIMER9_IRQHandler(void)
{
	if(timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP) == SET)
	{
		static int32_t commutationCounter = 0;
		static int32_t velocityCounter = 0;

		static int32_t lastHallAngle = 0;
		static int32_t hallAngleDiffAccu = 0;

		// Measure the hall sensor
		HallStates actualHallState = inputToHallState(HAL.IOs->config->isHigh(Pins.HALL_U), HAL.IOs->config->isHigh(Pins.HALL_V), HAL.IOs->config->isHigh(Pins.HALL_W));
		hallAngle = hallStateToAngle(actualHallState);

		// Calculate the hall angle difference
		int32_t hallAngleDiff = (hallAngle - lastHallAngle);         // [-300 ; +360)
		hallAngleDiff     = (hallAngleDiff + 360) % 360;         // [   0 ; +360)
		hallAngleDiff     = ((hallAngleDiff + 180) % 360) - 180; // [-180 ; +180)
		lastHallAngle = hallAngle;

		// Accumulate the hall angle for velocity measurement
		hallAngleDiffAccu += hallAngleDiff;

		// Calculate the velocity
		if (++velocityCounter >= PWM_FREQ / VELOCITY_CALC_FREQ)
		{
			actualHallVelocity = hallAngleDiffAccu * 60 * VELOCITY_CALC_FREQ / 360; // electrical rotations per minute

			hallAngleDiffAccu = 0;
			velocityCounter = 0;
		}

		if (commutationMode == BLDC_OPENLOOP)
		{
			// open loop mode
			if (openloopStepTime)
			{
				if (++commutationCounter >= openloopStepTime)
				{
					if (targetPWM > 0)
						targetAngle = (targetAngle + 60) % 360;
					else if (targetPWM < 0)
						targetAngle = (targetAngle - 60 + 360) % 360;

					commutationCounter = 0;
				}
			}
		}
		else if (commutationMode == BLDC_HALL)
		{
			if (targetPWM > 0)
			{
				// The +30 are to compensate hall getting rounded to the nearest 60° step
				targetAngle = ((hallAngle + 30) + 90) % 360;
			}
			else if (targetPWM < 0)
			{
				// The +30 are to compensate hall getting rounded to the nearest 60° step
				// The +360 are to prevent a negative operand for the modulo.
				targetAngle = ((hallAngle + 30) - 90 + 360) % 360;
			}
			else
			{
				targetAngle = hallAngle;
			}
		}

		// update commutation step
		int16_t duty = ((int32_t)targetPWM * ((int32_t)PWM_PERIOD))/(int32_t)s16_MAX;

		if (duty < 0)
			duty = -duty;

		switch (targetAngle % 360)
		{
		case 0:
			// U: Disabled
			// V: PWM
			// W: GND
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_DISABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_ENABLE);

			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_U, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_V, duty);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_W, 0);

			adc = ADC_PHASE_W;
			break;
		case 60:
			// U: GND
			// V: PWM
			// W: Disabled
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_DISABLE);

			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_U, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_V, duty);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_W, 0);

			adc = ADC_PHASE_U;
			break;
		case 120:
			// U: GND
			// V: Disabled
			// W: PWM
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_DISABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_ENABLE);

			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_U, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_V, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_W, duty);

			adc = ADC_PHASE_U;
			break;
		case 180:
			// U: Disabled
			// V: GND
			// W: PWM
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_DISABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_ENABLE);

			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_U, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_V, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_W, duty);

			adc = ADC_PHASE_V;
			break;
		case 240:
			// U: PWM
			// V: GND
			// W: Disabled
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_DISABLE);

			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_U, duty);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_V, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_W, 0);

			adc = ADC_PHASE_V;
			break;
		case 300:
			// U: PWM
			// V: Disabled
			// W: GND
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_ENABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_DISABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_ENABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_ENABLE);

			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_U, duty);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_V, 0);
			timer_channel_output_pulse_value_config(TIMER0, PWM_PHASE_W, 0);

			adc = ADC_PHASE_W;
			break;
		default:
			// Disable all phases
			timer_channel_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_U, TIMER_CCXN_DISABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_V, TIMER_CCXN_DISABLE);
			timer_channel_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCX_DISABLE);
			timer_channel_complementary_output_state_config(TIMER0, PWM_PHASE_W, TIMER_CCXN_DISABLE);
			break;
		}

		// For one-phase measurement always use the same phase
		if (adcCount == 1)
		{
			adc = ADC_PHASE_U;
		}

		timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
	}
}

void BLDC_enablePWM(uint8_t enable)
{
	if (enable)
	{
		Pins.PWM_UH->configuration.GPIO_Mode  = GPIO_MODE_AF;
		Pins.PWM_UH->configuration.GPIO_OType = GPIO_OTYPE_PP;
		Pins.PWM_UH->configuration.GPIO_PuPd  = GPIO_PUPD_NONE;
		gpio_af_set(Pins.PWM_UH->port, GPIO_AF_1, Pins.PWM_UH->bitWeight);
		HAL.IOs->config->set(Pins.PWM_UH);

		gpio_af_set(Pins.PWM_UL->port, GPIO_AF_1, Pins.PWM_UL->bitWeight);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_UL);
		gpio_af_set(Pins.PWM_VH->port, GPIO_AF_1, Pins.PWM_VH->bitWeight);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_VH);
		gpio_af_set(Pins.PWM_VL->port, GPIO_AF_1, Pins.PWM_VL->bitWeight);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_VL);
		gpio_af_set(Pins.PWM_WH->port, GPIO_AF_1, Pins.PWM_WH->bitWeight);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_WH);
		gpio_af_set(Pins.PWM_WL->port, GPIO_AF_1, Pins.PWM_WL->bitWeight);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_WL);

		pwmEnabled = 1;
	}
	else
	{
		Pins.PWM_UH->configuration.GPIO_Mode  = GPIO_MODE_INPUT;
		Pins.PWM_UH->configuration.GPIO_OType = GPIO_OTYPE_PP;
		Pins.PWM_UH->configuration.GPIO_PuPd  = GPIO_PUPD_PULLDOWN;

		HAL.IOs->config->set(Pins.PWM_UH);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_UL);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_VH);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_VL);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_WH);
		HAL.IOs->config->copy(&Pins.PWM_UH->configuration, Pins.PWM_WL);

		pwmEnabled = 0;
	}
}

uint8_t BLDC_isPWMenabled()
{
	return pwmEnabled;
}

void BLDC_setTargetPWM(int16_t pwm)
{
	targetPWM = pwm;
}

int16_t BLDC_getTargetPWM()
{
	return targetPWM;
}

int32_t BLDC_getMeasuredCurrent()
{
	int32_t sum = 0;
	for (uint8_t i = 0; i < ARRAY_SIZE(adcSamples); i++)
	{
		sum += adcSamples[i];
	}

	return sum / (int32_t) ARRAY_SIZE(adcSamples);
}

void BLDC_setCommutationMode(BLDCMode mode)
{
	commutationMode = mode;
}

BLDCMode BLDC_getCommutationMode()
{
	return commutationMode;
}

void BLDC_setPolePairs(uint8 polePairs)
{
	if (polePairs == 0)
		return;

	motorPolePairs = polePairs;
}

uint8_t BLDC_getPolePairs()
{
	return motorPolePairs;
}

void BLDC_setOpenloopStepTime(uint16_t stepTime)
{
	openloopStepTime = stepTime;
}

uint16_t BLDC_getOpenloopStepTime()
{
	return openloopStepTime;
}

int32_t BLDC_getTargetAngle()
{
	return targetAngle;
}

int32_t BLDC_getHallAngle()
{
	return hallAngle;
}

// Set the open loop velocity in RPM
void BLDC_setTargetOpenloopVelocity(uint32_t velocity)
{
	// 1 [RPM] = polePairs [eRPM]
	// [eRPM] = [1/60 eRPS] = 6/60 [steps/s]
	// steps/s = fpwm / openloopStepTime
	//
	// openloopStepTime = fpwm * 60 / 6 / velocity / polePairs
	openloopStepTime = PWM_FREQ * 10 / velocity / motorPolePairs;

	// Store the requested velocity for accurate reading
	// Otherwise we see rounding errors when reading back.
	openloopVelocity = velocity;
}

uint32_t BLDC_getTargetOpenloopVelocity()
{
	return openloopVelocity;
}

int32_t BLDC_getActualOpenloopVelocity()
{
	if (commutationMode != BLDC_OPENLOOP)
		return 0;

	if (targetPWM > 0)
		return openloopVelocity;
	else if (targetPWM < 0)
		return -openloopVelocity;
	else
		return 0;
}

// Velocity measured by hall in RPM
int32_t BLDC_getActualHallVelocity()
{
	return actualHallVelocity / motorPolePairs;
}

void BLDC_setHallOrder(uint8_t order)
{
	if (order < 3)
	{
		hallOrder = order;
	}
}

uint8_t BLDC_getHallOrder()
{
	return hallOrder;
}

void BLDC_setHallInvert(uint8_t invert)
{
	hallInvert = (invert)? 1:0;
}

uint8_t BLDC_getHallInvert()
{
	return hallInvert;
}

void BLDC_setBBMTime(uint8_t time)
{
	// Clip to the maximum value instead of overflowing
	bbmTime = MAX(127, time);

	TIMER_CCHP(TIMER0) = (TIMER_CCHP(TIMER0) & (~(uint32_t)TIMER_CCHP_DTCFG)) |
		(((uint32_t)TIMER_CCHP_DTCFG) & bbmTime);
}

uint8_t BLDC_getBBMTime()
{
	return bbmTime;
}
