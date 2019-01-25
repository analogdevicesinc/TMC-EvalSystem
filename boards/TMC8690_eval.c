/*
 * TMC8690_eval.c
 *
 *  Created on: 18.07.2016
 *      Author: bs, ed
 */

#include "Board.h"
#include "tmc/ic/TMC8690/TMC8690.h"

#include <math.h> // For M_PI

#define VM_MIN  50   // VM[V/10] min
#define VM_MAX  280  // VM[V/10] max +10%

#define MOTORS 1 // number of motors for this board

IOPinTypeDef *PIN_DRV_ENN;
ConfigurationTypeDef *TMC8690_config;

static u32 mainLoopCount = 0;

// => SPI wrapper
int tmc8690_readInt(uint8 address)
{
	return spi_ch1_readInt(address);
}

void tmc8690_writeInt(uint8 address, int value)
{
	spi_ch1_writeInt(address, value);
}
// <= SPI wrapper

// Helper functions: TMCL values are int, we need to transmit float values.
// To allow this, the two types are switched around via a union.
// (The values won't be the same, the bits will be)
inline static int float2int(float value)
{
	union { int i; float f; } converter = {.f = value};

	return converter.i;
}

inline static float int2float(int value)
{
	union { int i; float f; } converter = {.i = value};

	return converter.f;
}

// Efficiency Measurement
uint32 uSupply;
int iSupplyOffset;
float iSupplyGain;
int torqueOffset;
float torqueGain;
float efficiencyInPercent;

void TMC8690_calcEfficiency()
{
	//			 Pout		 Pmech		AngularSpeed * Torque
	// Eff	=	______	=	_______	=	_____________________
	//			 Pin		 Pelec		  Vsupply * Isupply
	//
	// AngularSpeed = 2 * PI * MeasuredSpeed[rpm] / 60
	//
	// Torque = (TorqueADCValue - TorqueOffset) * TorqueGain
	// ISupply = (ISupplyADCValue - ISupplyOffset) * ISupplyGain
	//
	float torque, iSupply, omega;

	iSupply = ((s32) tmc8690_readInt(ADC_I_SUPPLY_FILT) - iSupplyOffset) * iSupplyGain;

	if(iSupply == 0 || uSupply == 0) { // avoid division by 0
		efficiencyInPercent = 0;
		return;
	}

	torque = abs(TMC8690_FIELD_READ(ADC_TORQUE, TMC8690_ADC_TORQUE_MASK, TMC8690_ADC_TORQUE_SHIFT) - torqueOffset) * torqueGain;
	omega	= 2.0 * M_PI * (float) abs(tmc8690_readInt(VELOCITY_IN_RPM_ACTUAL_PT1)) / 65536 / 60; // 65536 = 2^12 // todo CHECK 3: Tippfehler oder Wertefehler? (2^12 = 4096, nicht 65536) (LH)
	efficiencyInPercent = torque * omega / iSupply / (uSupply / 10); // USupply in [0,100 V] must be divided by 10
}


static uint32 handleParameter(u8 readWrite, u8 motor, u8 type, int32 *value)
{
	u32 errors = TMC_ERROR_NONE;

	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	// Efficiency Measurement
	case 20:
		// U_Supply
		if(readWrite == READ)
			*value = uSupply;
		else if(readWrite == WRITE)
			uSupply = *value;
		break;
	case 21:
		// I_Supply_Offset
		if(readWrite == READ)
			*value = iSupplyOffset;
		else if(readWrite == WRITE)
			iSupplyOffset = *value;
		break;
	case 22:
		// I_Supply_Gain
		if(readWrite == READ)
			*value = float2int(iSupplyGain);
		else if(readWrite == WRITE)
			iSupplyGain = int2float(*value);
		break;
	case 24:
		// Torque_Offset
		if(readWrite == READ)
			*value = torqueOffset;
		else if(readWrite == WRITE)
			torqueOffset = *value;
		break;
	case 25:
		// Torque_Gain
		if(readWrite == READ)
			*value = float2int(torqueGain);
		else if(readWrite == WRITE)
			torqueGain = int2float(*value);
		break;
	case 26:
		// Efficiency_in_percent_F32
		if(readWrite == READ)
		{
			TMC8690_calcEfficiency();
			*value = float2int(efficiencyInPercent);
		}
		else if(readWrite == WRITE)
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;

	case 27:
		// POSITION_ACTUAL_16
		if(readWrite == READ)
			*value = (s32) TMC8690_FIELD_READ(POSITION_ACTUAL, 0xFFFF0000, 16);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
#ifdef ENABLE_IC_SCOPE
	case 39:  // OSCI_status
		if(readWrite == READ)
		{
			*value = (s16) TMC8690_FIELD_READ(OSCI_STATUS, TMC8690_OSCI_STATUS_MASK, TMC8690_OSCI_STATUS_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 40:  // trigger_channel
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_TRG_CHN_MASK, TMC8690_TRG_CHN_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_TRG_CHN_MASK, TMC8690_TRG_CHN_SHIFT, *value);
		}
		break;
	case 41:  // trigger_condition
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_TRG_CND_MASK, TMC8690_TRG_CND_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_TRG_CND_MASK, TMC8690_TRG_CND_SHIFT, *value);
		}
		break;
	case 42:  // trigger_value
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_TRG_VAL_MASK, TMC8690_TRG_VAL_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_TRG_VAL_MASK, TMC8690_TRG_VAL_SHIFT, *value);
		}
		break;
	case 43:  // OSR
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_OSR_MASK, TMC8690_OSR_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_OSR_MASK, TMC8690_OSR_SHIFT, *value);
		}
		break;
	case 44:  // DSR
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_DSR_MASK, TMC8690_DSR_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_DSR_MASK, TMC8690_DSR_SHIFT, *value);
		}
		break;
	case 45:  // enable/disable measurement
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_MEAS_EN_MASK, TMC8690_MEAS_EN_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_MEAS_EN_MASK, TMC8690_MEAS_EN_SHIFT, *value);
		}
		break;
	case 46:  // clear measurement
		if(readWrite == READ)
		{
			*value = TMC8690_FIELD_READ(OSCI_MAIN_CONFIG_REG, TMC8690_MEAS_CLR_MASK, TMC8690_MEAS_CLR_SHIFT);
		}
		else if(readWrite == WRITE)
		{
			TMC8690_FIELD_UPDATE(OSCI_MAIN_CONFIG_REG, TMC8690_MEAS_CLR_MASK, TMC8690_MEAS_CLR_SHIFT, *value);
		}
		break;
	case 47:  // Pretrigger_conf
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(OSCI_PRETRIGGER_CONFIG_REG, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_MASK, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(OSCI_PRETRIGGER_CONFIG_REG, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_MASK, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_SHIFT, *value);
		break;
	case 48:	// data
		if(readWrite == READ)
		{
			*value = tmc8690_readInt(OSCI_OUTPUT_DATA);
		}
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 49:  // data address // todo AP 2: shifte die restlichen Adressen nach oben #1
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(OSCI_DATA_ADDRESS, TMC8690_OSCI_DATA_ADDRESS_MASK, TMC8690_OSCI_DATA_ADDRESS_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(OSCI_DATA_ADDRESS, TMC8690_OSCI_DATA_ADDRESS_MASK, TMC8690_OSCI_DATA_ADDRESS_SHIFT, *value);
		break;
#endif /* ifdef ENABLE_IC_SCOPE */
	case 51:
		// CHIPINFO_DATA
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(CHIPINFO_DATA, TMC8690_CHIPINFO_DATA_MASK, TMC8690_CHIPINFO_DATA_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 52:
		// CHIPINFO_ADDR
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(CHIPINFO_ADDR, TMC8690_CHIPINFO_ADDR_MASK, TMC8690_CHIPINFO_ADDR_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(CHIPINFO_ADDR, TMC8690_CHIPINFO_ADDR_MASK, TMC8690_CHIPINFO_ADDR_SHIFT, *value);
		break;
	case 57:
		// STATUS_WORD
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATUS_WORD);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 58:
		// COMMAND_WORD
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(COMMAND_WORD);
		else if(readWrite == WRITE)
			tmc8690_writeInt(COMMAND_WORD, *value);
		break;
	case 59:
		// P_FAK_BRAKE
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(P_FAK_BRAKE, TMC8690_P_FAK_BRAKE_MASK, TMC8690_P_FAK_BRAKE_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(P_FAK_BRAKE, TMC8690_P_FAK_BRAKE_MASK, TMC8690_P_FAK_BRAKE_SHIFT, *value);
		break;
	case 60:
		// I_FAK_BRAKE
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(I_FAK_BRAKE, TMC8690_I_FAK_BRAKE_MASK, TMC8690_I_FAK_BRAKE_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(I_FAK_BRAKE, TMC8690_I_FAK_BRAKE_MASK, TMC8690_I_FAK_BRAKE_SHIFT, *value);
		break;
	case 61:
		// P_FAK_HEAT
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(P_FAK_HEAT, TMC8690_P_FAK_HEAT_MASK, TMC8690_P_FAK_HEAT_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(P_FAK_HEAT, TMC8690_P_FAK_HEAT_MASK, TMC8690_P_FAK_HEAT_SHIFT, *value);
		break;
	case 62:
		// I_FAK_HEAT
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(I_FAK_HEAT, TMC8690_I_FAK_HEAT_MASK, TMC8690_I_FAK_HEAT_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(I_FAK_HEAT, TMC8690_I_FAK_HEAT_MASK, TMC8690_I_FAK_HEAT_SHIFT, *value);
		break;
	case 63:
		// P_FAK_VEL
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(P_FAK_VEL, TMC8690_P_FAK_VEL_MASK, TMC8690_P_FAK_VEL_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(P_FAK_VEL, TMC8690_P_FAK_VEL_MASK, TMC8690_P_FAK_VEL_SHIFT, *value);
		break;
	case 64:
		// I_FAK_VEL
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(I_FAK_VEL, TMC8690_I_FAK_VEL_MASK, TMC8690_I_FAK_VEL_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(I_FAK_VEL, TMC8690_I_FAK_VEL_MASK, TMC8690_I_FAK_VEL_SHIFT, *value);
		break;
	case 65:
		// P_FAK_POS
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(P_FAK_POS, TMC8690_P_FAK_POS_MASK, TMC8690_P_FAK_POS_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(P_FAK_POS, TMC8690_P_FAK_POS_MASK, TMC8690_P_FAK_POS_SHIFT, *value);
		break;
	case 66:
		// ENC_Z_COMP_VAL
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(ENC_Z_COMP_VAL, TMC8690_ENC_Z_COMP_VAL_MASK, TMC8690_ENC_Z_COMP_VAL_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(ENC_Z_COMP_VAL, TMC8690_ENC_Z_COMP_VAL_MASK, TMC8690_ENC_Z_COMP_VAL_SHIFT, *value);
		break;
	case 67:
		// TORQUE_TARGET
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(TORQUE_TARGET, TMC8690_TORQUE_TARGET_MASK, TMC8690_TORQUE_TARGET_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(TORQUE_TARGET, TMC8690_TORQUE_TARGET_MASK, TMC8690_TORQUE_TARGET_SHIFT, *value);
		break;
	case 68:
		// VELOCITY_TARGET
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(VELOCITY_TARGET);
		else if(readWrite == WRITE)
			tmc8690_writeInt(VELOCITY_TARGET, *value);
		break;
	case 69:
		// POSITION_TARGET
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(POSITION_TARGET);
		else if(readWrite == WRITE)
			tmc8690_writeInt(POSITION_TARGET, *value);
		break;
	case 70:
		// TEMPERATURE_TARGET
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(TEMPERATURE_TARGET, TMC8690_TEMPERATURE_TARGET_MASK, TMC8690_TEMPERATURE_TARGET_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(TEMPERATURE_TARGET, TMC8690_TEMPERATURE_TARGET_MASK, TMC8690_TEMPERATURE_TARGET_SHIFT, *value);
		break;
	case 71:
		// OSCI_MAIN_CONFIG_REG
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(OSCI_MAIN_CONFIG_REG);
		else if(readWrite == WRITE)
			tmc8690_writeInt(OSCI_MAIN_CONFIG_REG, *value);
		break;
	case 72:
		// OSCI_PRETRIGGER_CONFIG_REG
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(OSCI_PRETRIGGER_CONFIG_REG, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_MASK, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(OSCI_PRETRIGGER_CONFIG_REG, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_MASK, TMC8690_OSCI_PRETRIGGER_CONFIG_REG_SHIFT, *value);
		break;
	case 73:
		// OSCI_DATA_ADDRESS
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(OSCI_DATA_ADDRESS, TMC8690_OSCI_DATA_ADDRESS_MASK, TMC8690_OSCI_DATA_ADDRESS_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(OSCI_DATA_ADDRESS, TMC8690_OSCI_DATA_ADDRESS_MASK, TMC8690_OSCI_DATA_ADDRESS_SHIFT, *value);
		break;
	case 74:
		// OSCI_STATUS
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(OSCI_STATUS, TMC8690_OSCI_STATUS_MASK, TMC8690_OSCI_STATUS_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 75:
		// OSCI_OUTPUT_DATA
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(OSCI_OUTPUT_DATA, TMC8690_OSCI_OUTPUT_DATA_MASK, TMC8690_OSCI_OUTPUT_DATA_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 76:
		// ADC_ENC_TRACK_A
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(ADC_ENC_TRACK_A, TMC8690_ADC_ENC_TRACK_A_MASK, TMC8690_ADC_ENC_TRACK_A_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 77:
		// ADC_ENC_TRACK_B
		if(readWrite == READ)
			*value = (s16) TMC8690_FIELD_READ(ADC_ENC_TRACK_B, TMC8690_ADC_ENC_TRACK_B_MASK, TMC8690_ADC_ENC_TRACK_B_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 78:
		// ADC_ENC_TRACK_Z
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_ENC_TRACK_Z, TMC8690_ADC_ENC_TRACK_Z_MASK, TMC8690_ADC_ENC_TRACK_Z_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 79:
		// ADC_I_U
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_I_U, TMC8690_ADC_I_U_MASK, TMC8690_ADC_I_U_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 80:
		// ADC_I_V
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_I_V, TMC8690_ADC_I_V_MASK, TMC8690_ADC_I_V_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 81:
		// ADC_I_W
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_I_W, TMC8690_ADC_I_W_MASK, TMC8690_ADC_I_W_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 82:
		// ADC_I_SUPPLY
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(ADC_I_SUPPLY);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 83:
		// ADC_I_SUPPLY_FILT
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(ADC_I_SUPPLY_FILT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 84:
		// ADC_TORQUE
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_TORQUE, TMC8690_ADC_TORQUE_MASK, TMC8690_ADC_TORQUE_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 85:
		// ADC_TEMP
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_TEMP, TMC8690_ADC_TEMP_MASK, TMC8690_ADC_TEMP_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 86:
		// ADC_MICROPHONE
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_MICROPHONE, TMC8690_ADC_MICROPHONE_MASK, TMC8690_ADC_MICROPHONE_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 87:
		// ADC_I_BRAKE
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(ADC_I_BRAKE, TMC8690_ADC_I_BRAKE_MASK, TMC8690_ADC_I_BRAKE_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 88:
		// TRACK_A_OFFSET
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(TRACK_A_OFFSET, TMC8690_TRACK_A_OFFSET_MASK, TMC8690_TRACK_A_OFFSET_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(TRACK_A_OFFSET, TMC8690_TRACK_A_OFFSET_MASK, TMC8690_TRACK_A_OFFSET_SHIFT, *value);
		break;
	case 89:
		// TRACK_B_OFFSET
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(TRACK_B_OFFSET, TMC8690_TRACK_B_OFFSET_MASK, TMC8690_TRACK_B_OFFSET_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(TRACK_B_OFFSET, TMC8690_TRACK_B_OFFSET_MASK, TMC8690_TRACK_B_OFFSET_SHIFT, *value);
		break;
	case 90:
		// DAC_V_REF
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(DAC_V_REF, TMC8690_DAC_V_REF_MASK, TMC8690_DAC_V_REF_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(DAC_V_REF, TMC8690_DAC_V_REF_MASK, TMC8690_DAC_V_REF_SHIFT, *value);
		break;
	case 91:
		// T_PT1_FILTER_VEL
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(T_PT1_FILTER_VEL, TMC8690_T_PT1_FILTER_VEL_MASK, TMC8690_T_PT1_FILTER_VEL_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(T_PT1_FILTER_VEL, TMC8690_T_PT1_FILTER_VEL_MASK, TMC8690_T_PT1_FILTER_VEL_SHIFT, *value);
		break;
	case 92:
		// T_PT1_FILTER_I_SUPPLY
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(T_PT1_FILTER_I_SUPPLY, TMC8690_T_PT1_FILTER_I_SUPPLY_MASK, TMC8690_T_PT1_FILTER_I_SUPPLY_SHIFT);
		else if(readWrite == WRITE)
			TMC8690_FIELD_UPDATE(T_PT1_FILTER_I_SUPPLY, TMC8690_T_PT1_FILTER_I_SUPPLY_MASK, TMC8690_T_PT1_FILTER_I_SUPPLY_SHIFT, *value);
		break;
	case 93:
		// DAC_BRAKE
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(DAC_BRAKE);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 94:
		// DAC_BRAKE_TARGET
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(DAC_BRAKE_TARGET);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DAC_BRAKE_TARGET, *value);
		break;
	case 95:
		// VELOCITY_ACTUAL
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(VELOCITY_ACTUAL);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 96:
		// VELOCITY_IN_RPM_ACTUAL
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(VELOCITY_IN_RPM_ACTUAL);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 97:
		// VELOCITY_IN_RPM_ACTUAL_PT1
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(VELOCITY_IN_RPM_ACTUAL_PT1);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 98:
		// VELOCITY_IN_RPM_ACTUAL_MAV
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(VELOCITY_IN_RPM_ACTUAL_MAV);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 99:
		// POSITION_ACTUAL
		if(readWrite == READ)
			*value = (u32) tmc8690_readInt(POSITION_ACTUAL);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 100:
		// VEL_CTRL_TORQUE_TARGET
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(VEL_CTRL_TORQUE_TARGET, TMC8690_VEL_CTRL_TORQUE_TARGET_MASK, TMC8690_VEL_CTRL_TORQUE_TARGET_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 101:
		// POS_CTRL_VEL_TARGET
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(POS_CTRL_VEL_TARGET, TMC8690_POS_CTRL_VEL_TARGET_MASK, TMC8690_POS_CTRL_VEL_TARGET_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 102:
		// PWM_HEATER
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(PWM_HEATER, TMC8690_PWM_HEATER_MASK, TMC8690_PWM_HEATER_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 103:
		// U_BRAKE
		if(readWrite == READ)
			*value = (u16) TMC8690_FIELD_READ(U_BRAKE, TMC8690_U_BRAKE_MASK, TMC8690_U_BRAKE_SHIFT);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 104:
		// DEBUG_AXIS_PARAM_0
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_0);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_0, *value);
		break;
	case 105:
		// DEBUG_AXIS_PARAM_1
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_1);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_1, *value);
		break;
	case 106:
		// DEBUG_AXIS_PARAM_2
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_2);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_2, *value);
		break;
	case 107:
		// DEBUG_AXIS_PARAM_3
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_3);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_3, *value);
		break;
	case 108:
		// DEBUG_AXIS_PARAM_4
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_4);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_4, *value);
		break;
	case 109:
		// DEBUG_AXIS_PARAM_5
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_5);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_5, *value);
		break;
	case 110:
		// DEBUG_AXIS_PARAM_6
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_6);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_6, *value);
		break;
	case 111:
		// DEBUG_AXIS_PARAM_7
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(DEBUG_AXIS_PARAM_7);
		else if(readWrite == WRITE)
			tmc8690_writeInt(DEBUG_AXIS_PARAM_7, *value);
		break;
	case 112:
		// STATE_VAR_PARAM_0
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_0);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 113:
		// STATE_VAR_PARAM_1
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_1);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 114:
		// STATE_VAR_PARAM_2
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_2);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 115:
		// STATE_VAR_PARAM_3
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_3);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 116:
		// STATE_VAR_PARAM_4
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_4);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 117:
		// STATE_VAR_PARAM_5
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_5);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 118:
		// STATE_VAR_PARAM_6
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_6);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 119:
		// STATE_VAR_PARAM_7
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATE_VAR_PARAM_7);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 120:
		// SAMP_INPUTS_RAW
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(SAMP_INPUTS_RAW);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 121:
		// SAMP_OUTPUTS_RAW
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(SAMP_OUTPUTS_RAW);
		else if(readWrite == WRITE)
			errors |= TMC_ERROR_TYPE;
		break;
	case 122:
		// STATUS_FLAGS
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(STATUS_FLAGS);
		else if(readWrite == WRITE)
			tmc8690_writeInt(STATUS_FLAGS, *value);
		break;
	case 123:
		// WARNING_MASK
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(WARNING_MASK);
		else if(readWrite == WRITE)
			tmc8690_writeInt(WARNING_MASK, *value);
		break;
	case 124:
		// ERROR_MASK
		if(readWrite == READ)
			*value = (s32) tmc8690_readInt(ERROR_MASK);
		else if(readWrite == WRITE)
			tmc8690_writeInt(ERROR_MASK, *value);
		break;
	case 255:
		// Main loop counter
		if(readWrite == READ) {
			*value = mainLoopCount;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	default:
		errors |= TMC_ERROR_TYPE;
		break;
	}
	return errors;
}

static void periodicJob(uint32 actualSystick)
{
	static u32 loopCounterCheckTime;
	static u32 loopCounter = 0;

	if((actualSystick - loopCounterCheckTime) >= 1000)
	{
		mainLoopCount = loopCounter;
		loopCounter = 0;

		loopCounterCheckTime = actualSystick;
	}
	loopCounter++;
}

static void writeRegister(u8 motor, uint8 address, int32 value)
{
	UNUSED(motor);
	tmc8690_writeInt(address, value);
}

static void readRegister(u8 motor, uint8 address, int32 *value)
{
	UNUSED(motor);
	*value = tmc8690_readInt(address);
}

static uint32 SAP(uint8 type, uint8 motor, int32 value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32 GAP(uint8 type, uint8 motor, int32 *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32 userFunction(uint8 type, uint8 motor, int32 *value)
{
	UNUSED(type);
	UNUSED(motor);
	UNUSED(value);
	return 0;
}

static void enableDriver(DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = Evalboards.driverEnable;

	if(state == DRIVER_DISABLE)
		HAL.IOs->config->setLow(PIN_DRV_ENN);
	else if((state == DRIVER_ENABLE) && (Evalboards.driverEnable == DRIVER_ENABLE))
		HAL.IOs->config->setHigh(PIN_DRV_ENN);
}

static void deInit(void)
{
	enableDriver(DRIVER_DISABLE);
	HAL.IOs->config->reset(PIN_DRV_ENN);
};

static uint8 reset()
{
	return 1;
}

static uint8 restore()
{
	return 1;
}

static void checkErrors(uint32 tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

void TMC8690_init(void)
{
	// configure ENABLE-PIN for TMC8690
	PIN_DRV_ENN = &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	HAL.IOs->config->setHigh(PIN_DRV_ENN);

	TMC8690_config = Evalboards.ch1.config;

	// connect evalboard functions
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_RESET;
	Evalboards.ch1.config->configIndex  = 0;
//// todo ADD 3: erweitern! (ED) => #2
//	Evalboards.ch1.rotate               = rotate;
//	Evalboards.ch1.right                = right;
//	Evalboards.ch1.left                 = left;
//	Evalboards.ch1.stop                 = stop;
//	Evalboards.ch1.getMeasuredSpeed     = getMeasuredSpeed;
//// <=
	Evalboards.ch1.GAP                  = GAP;
	Evalboards.ch1.SAP                  = SAP;
//	Evalboards.ch1.moveTo               = moveTo;
//	Evalboards.ch1.moveBy               = moveBy;
	Evalboards.ch1.writeRegister        = writeRegister;
	Evalboards.ch1.readRegister         = readRegister;
	Evalboards.ch1.periodicJob          = periodicJob;
	Evalboards.ch1.userFunction         = userFunction;
	Evalboards.ch1.enableDriver         = enableDriver;
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = MOTORS;
	Evalboards.ch1.deInit               = deInit;
	Evalboards.ch1.VMMin                = VM_MIN;
	Evalboards.ch1.VMMax                = VM_MAX;

	// init used API functions
//	tmc8690_init(); // no init function yet

	// enable the driver
	enableDriver(DRIVER_ENABLE);
};

