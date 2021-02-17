#include "Board.h"
#include "tmc/ic/TMC4670/TMC4670.h"
#include "hal/SPI.h"

#define DEFAULT_MOTOR 0

static IOPinTypeDef *PIN_DRV_ENN;
static ConfigurationTypeDef *TMC4670_config;
static SPIChannelTypeDef *TMC4670_SPIChannel;

typedef struct
{
	uint32_t  startVoltage;
	uint16_t  initWaitTime;
	uint16_t  actualInitWaitTime;
	uint8_t   initState;
	uint8_t   initMode;
	uint16_t  torqueMeasurementFactor; // u8.u8
	uint8_t   motionMode;
} TMinimalMotorConfig;

static TMinimalMotorConfig motorConfig[TMC4670_MOTORS];

// => SPI wrapper
uint8_t tmc4670_readwriteByte(uint8_t motor, uint8_t data, uint8_t lastTransfer)
{
	if (motor == DEFAULT_MOTOR)
		return TMC4670_SPIChannel->readWrite(data, lastTransfer);
	else
		return 0;
}
// <= SPI wrapper

static uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= TMC4670_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc4670_setTargetVelocity(motor, velocity);

	return TMC_ERROR_NONE;
}

static uint32_t right(uint8_t motor, int32_t velocity)
{
	return rotate(motor, velocity);
}

static uint32_t left(uint8_t motor, int32_t velocity)
{
	return rotate(motor, -velocity);
}

static uint32_t stop(uint8_t motor)
{
	return rotate(motor, 0);
}

static uint32_t moveTo(uint8_t motor, int32_t position)
{
	if(motor >= TMC4670_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc4670_setAbsolutTargetPosition(motor, position);

	return TMC_ERROR_NONE;
}

static uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= TMC4670_MOTORS)
		return TMC_ERROR_MOTOR;

	tmc4670_setRelativeTargetPosition(motor, *ticks);

	return TMC_ERROR_NONE;
}

static uint32_t handleParameter(uint8_t readWrite, uint8_t motor, uint8_t type, int32_t *value)
{
	uint32_t errors = TMC_ERROR_NONE;

	if(motor >= TMC4670_MOTORS)
		return TMC_ERROR_MOTOR;

	switch(type)
	{
	case 4:
		// Maximum speed
		if(readWrite == READ)
			*value = (uint32_t)tmc4670_readInt(motor, TMC4670_PID_VELOCITY_LIMIT);
		else if(readWrite == WRITE)
			tmc4670_writeInt(motor, TMC4670_PID_VELOCITY_LIMIT, *value);
		break;
	// ADC offsets
	case 50:
		// ADC_I1_OFFSET
		if(readWrite == READ)
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_I1_SCALE_OFFSET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4670_writeRegister16BitValue(motor, TMC4670_ADC_I1_SCALE_OFFSET, BIT_0_TO_15, *value);
		break;
	case 51:
		// ADC_I0_OFFSET
		if(readWrite == READ)
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_I0_SCALE_OFFSET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4670_writeRegister16BitValue(motor, TMC4670_ADC_I0_SCALE_OFFSET, BIT_0_TO_15, *value);
		break;
		// ADC scaler values
	case 53:
		// ADC_I1_SCALE
		if(readWrite == READ)
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_I1_SCALE_OFFSET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4670_writeRegister16BitValue(motor, TMC4670_ADC_I1_SCALE_OFFSET, BIT_16_TO_31, *value);
		break;
	case 54:
		// ADC_I0_SCALE
		if(readWrite == READ)
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_I0_SCALE_OFFSET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4670_writeRegister16BitValue(motor, TMC4670_ADC_I0_SCALE_OFFSET, BIT_16_TO_31, *value);
		break;
	case 100:
		// ADC_I_U_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 1);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 101:
		// ADC_I_V_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 1);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 102:
		// ADC_IB_RAW
		// ADC_I_B_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 2);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 103:
		// ADC_I_UX_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 0);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 104:
		// ADC_I_WY_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 0);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 105:
		// ADC_VM_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 3);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 106:
		// ADC_T_MOSFETS_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 4);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 107:
		// ADC_T_MOTOR_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 4);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 108:
		// ADC_U_UX_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 5);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 109:
		// ADC_U_WY_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 5);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 110:
		// ADC_U_V_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 6);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 111:
		// AENC_UX_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 7);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 112:
		// AENC_WY_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 7);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 113:
		// AENC_V_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 8);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 114:
		// AENC_N_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 8);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 115:
		// ANALOG_GPI_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 9);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 116:
		// ADC_I0_scaled
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_IWY_IUX, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 117:
		// ADC_I2_scaled
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_IWY_IUX, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 118:
		// ADC_I1_scaled
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_IV, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 120:
		// AENC_UX
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_WY_UX, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 121:
		// AENC_WY
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_WY_UX, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 122:
		// AENC_VN
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_N_VN, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 123:
		// AENC_N
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_N_VN, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 124:
		// ADCSD_I0_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 10);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 125:
		// ADCSD_I1_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 10);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 126:
		// ADCSD_I_B_RAW
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_ADC_RAW_ADDR, 11);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_RAW_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 127:
		// ADC_I0_EXT
		if(readWrite == READ) {
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_I1_I0_EXT, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 128:
		// ADC_I1_EXT
		if(readWrite == READ) {
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ADC_I1_I0_EXT, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 132:
		// AENC_DECODER_COUNT
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_AENC_DECODER_COUNT);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 133:
		// AENC_DECODER_COUNT_N
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_AENC_DECODER_COUNT_N);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 134:
		// AENC_DECODER_PHI_E
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_DECODER_PHI_E_PHI_M, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 135:
		// AENC_DECODER_PHI_M
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_DECODER_PHI_E_PHI_M, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 136:
		// AENC_DECODER_POSITION
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_AENC_DECODER_POSITION);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 140:
		// OPENLOOP_VELOCITY_TARGET
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_OPENLOOP_VELOCITY_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 141:
		// OPENLOOP_VELOCITY_ACTUAL
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_OPENLOOP_VELOCITY_ACTUAL);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 142:
		// OPENLOOP_PHI
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_OPENLOOP_PHI, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 150:
		// ABN_DECODER_COUNT
		if(readWrite == READ) {
			*value = tmc4670_readInt(motor, TMC4670_ABN_DECODER_COUNT) & 0x00FFFFFF;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 151:
		// ABN_DECODER_COUNT_N
		if(readWrite == READ) {
			*value = tmc4670_readInt(motor, TMC4670_ABN_DECODER_COUNT_N) & 0x00FFFFFF;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 152:
		// ABN_DECODER_PHI_E
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ABN_DECODER_PHI_E_PHI_M, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 153:
		// ABN_DECODER_PHI_M
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_ABN_DECODER_PHI_E_PHI_M, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 160:
		// HALL_PHI_E
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_HALL_PHI_E_INTERPOLATED_PHI_E, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 161:
		// HALL_PHI_E_INTERPOLATED
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_HALL_PHI_E_INTERPOLATED_PHI_E, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 162:
		// AENC_DECODER_PHI_A_RAW
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_DECODER_PHI_A_RAW, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 163:
		// AENC_DECODER_PHI_A
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_AENC_DECODER_PHI_A, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 164:
		// HALL_PHI_M
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_HALL_PHI_M, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 170:
		// PHI_E
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_PHI_E, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 171:
		// PID_TORQUE_TARGET
		if(readWrite == READ)
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31);
		else if(readWrite == WRITE)
			tmc4670_writeRegister16BitValue(motor, TMC4670_PID_TORQUE_FLUX_TARGET, BIT_16_TO_31, *value);
		break;
	case 172:
		// PID_FLUX_TARGET
		if(readWrite == READ)
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_PID_TORQUE_FLUX_TARGET, BIT_0_TO_15);
		else if(readWrite == WRITE)
			tmc4670_writeRegister16BitValue(motor, TMC4670_PID_TORQUE_FLUX_TARGET, BIT_0_TO_15, *value);
		break;
	case 173:
		// PID_VELOCITY_TARGET
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_PID_VELOCITY_TARGET);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 174:
		// PID_POSITION_TARGET
		if(readWrite == READ) {
			*value = (int32_t) tmc4670_readInt(motor, TMC4670_PID_POSITION_TARGET);
		} else if(readWrite == WRITE) {
			tmc4670_setAbsolutTargetPosition(motor, *value);
		}
		break;
	case 175:
		// PID_TORQUE_ACTUAL
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_PID_TORQUE_FLUX_ACTUAL, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 176:
		// PID_TORQUE_ACTUAL_mA
		if(readWrite == READ) {
			*value = tmc4670_getActualTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 177:
		// PID_FLUX_ACTUAL
		if(readWrite == READ) {
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_PID_TORQUE_FLUX_ACTUAL, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 178:
		// PID_VELOCITY_ACTUAL
		if(readWrite == READ) {
			*value = tmc4670_getActualVelocity(motor);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 179:
		// PID_POSITION_ACTUAL
		if(readWrite == READ)
			*value = tmc4670_getActualPosition(motor);
		else if(readWrite == WRITE)
			tmc4670_setActualPosition(motor, *value);
		break;
	case 180:
		// PID_TORQUE_ERROR
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 0);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 181:
		// PID_FLUX_ERROR
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 1);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 182:
		// PID_VELOCITY_ERROR
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 2);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 183:
		// PID_POSITION_ERROR
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 3);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 184:
		// PID_TORQUE_ERROR_SUM
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 4);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 185:
		// PID_FLUX_ERROR_SUM
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 5);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 186:
		// PID_VELOCITY_ERROR_SUM
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 6);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 187:
		// PID_POSITION_ERROR_SUM
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_PID_ERROR_ADDR, 7);
			*value = tmc4670_readInt(motor, TMC4670_PID_ERROR_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 189:
		// PIDIN_TARGET_TORQUE
		if(readWrite == READ) {
			*value = tmc4670_getTargetTorque_raw(motor);
		} else if(readWrite == WRITE) {
			tmc4670_setTargetTorque_raw(motor, *value);
		}
		break;
	case 190:
		// PIDIN_TARGET_TORQUE_mA
		if(readWrite == READ) {
			*value = tmc4670_getTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor);
		} else if(readWrite == WRITE) {
			tmc4670_setTargetTorque_mA(motor, motorConfig[motor].torqueMeasurementFactor, *value);
		}
		break;
	case 191:
		// PIDIN_TARGET_FLUX
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 1);
			*value = tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 192:
		// PIDIN_TARGET_VELOCITY
		if(readWrite == READ) {
			*value = tmc4670_getTargetVelocity(motor);
		} else if(readWrite == WRITE) {
			tmc4670_setTargetVelocity(motor, *value);
		}
		break;
	case 193:
		// PIDIN_TARGET_POSITION
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 3);
			*value = tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 194:
		// PIDOUT_TARGET_TORQUE
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 4);
			*value = tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 195:
		// PIDOUT_TARGET_FLUX
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 5);
			*value = tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 196:
		// PIDOUT_TARGET_VELOCITY
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 6);
			*value = tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 197:
		// PIDOUT_TARGET_POSITION
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 7);
			*value = tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 198:
		// FOC_IUX
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 8);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 199:
		// FOC_IWY
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 8);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 200:
		// FOC_IV
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 9);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 201:
		// FOC_IA
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 10);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 202:
		// FOC_IB
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 10);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 203:
		// FOC_ID
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 11);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 204:
		// FOC_IQ
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 11);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 205:
		// FOC_UD
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 12);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 206:
		// FOC_UQ
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 12);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 207:
		// FOC_UD_LIMITED
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 13);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 208:
		// FOC_UQ_LIMITED
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 13);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 209:
		// FOC_UA
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 14);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 210:
		// FOC_UB
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 14);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 211:
		// FOC_UUX
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 15);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 212:
		// FOC_UWY
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 15);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 213:
		// FOC_UV
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 16);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 214:
		// PWM_UX
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 17);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 215:
		// PWM_WY
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 17);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 216:
		// PWM_UV
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 18);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 217:
		// ADC_I_0
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 19);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 218:
		// ADC_I_1
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 19);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 230:
		// DEBUG_VALUE_0
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 192);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 231:
		// DEBUG_VALUE_1
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 192);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 232:
		// DEBUG_VALUE_2
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 193);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 233:
		// DEBUG_VALUE_3
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 193);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 234:
		// DEBUG_VALUE_4
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 194);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 235:
		// DEBUG_VALUE_5
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 194);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 236:
		// DEBUG_VALUE_6
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 195);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 237:
		// DEBUG_VALUE_7
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 195);
			*value = (int16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 238:
		// DEBUG_VALUE_8
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 196);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 239:
		// DEBUG_VALUE_9
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 196);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 240:
		// DEBUG_VALUE_10
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 197);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 241:
		// DEBUG_VALUE_11
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 197);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 242:
		// DEBUG_VALUE_12
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 198);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 243:
		// DEBUG_VALUE_13
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 198);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 244:
		// DEBUG_VALUE_14
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 199);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_0_TO_15);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 245:
		// DEBUG_VALUE_15
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 199);
			*value = (uint16_t) tmc4670_readRegister16BitValue(motor, TMC4670_INTERIM_DATA, BIT_16_TO_31);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 246:
		// DEBUG_VALUE_16
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 200);
			*value =  (int32_t)tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 247:
		// DEBUG_VALUE_17
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 201);
			*value =  (int32_t)tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 248:
		// DEBUG_VALUE_18
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 202);
			*value =  (int32_t) tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 249:
		// DEBUG_VALUE_19
		if(readWrite == READ) {
			tmc4670_writeInt(motor, TMC4670_INTERIM_ADDR, 203);
			*value =  (int32_t) tmc4670_readInt(motor, TMC4670_INTERIM_DATA);
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 251:
		// Torque measurement factor
		if(readWrite == READ) {
			*value = motorConfig[motor].torqueMeasurementFactor;
		} else if(readWrite == WRITE) {
			motorConfig[motor].torqueMeasurementFactor = *value;
		}
		break;
	case 252:
		// Start encoder initialization
		if(readWrite == READ) {
			*value = motorConfig[motor].initMode;
		} else if(readWrite == WRITE) {
			tmc4670_startEncoderInitialization(*value, &motorConfig[motor].initMode, &motorConfig[motor].initState);
		}
		break;
	case 253:
		// Encoder init state
		if(readWrite == READ) {
			*value = motorConfig[motor].initState;
		} else if(readWrite == WRITE) {
			errors |= TMC_ERROR_TYPE;
		}
		break;
	case 254:
		// Actual encoder wait time
		if(readWrite == READ) {
			*value = motorConfig[motor].actualInitWaitTime;
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

static uint32_t getMeasuredSpeed(uint8_t motor, int32_t *value)
{
	if(motor >= TMC4670_MOTORS)
		return TMC_ERROR_MOTOR;

	*value = tmc4670_getActualVelocity(motor);

	return TMC_ERROR_NONE;
}

static void periodicJob(uint32_t actualSystick)
{
	// do encoder initialization if necessary
	int motor;
	for(motor = 0; motor < TMC4670_MOTORS; motor++)
	{
		tmc4670_periodicJob(motor, actualSystick, motorConfig[motor].initMode,
				&(motorConfig[motor].initState), motorConfig[motor].initWaitTime,
				&(motorConfig[motor].actualInitWaitTime), motorConfig[motor].startVoltage);
	}
}

static void writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	UNUSED(motor);
	tmc4670_writeInt(DEFAULT_MOTOR, address, value);
}

static void readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	UNUSED(motor);
	*value = tmc4670_readInt(DEFAULT_MOTOR, address);
}

static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
	return handleParameter(READ, motor, type, value);
}

static uint32_t userFunction(uint8_t type, uint8_t motor, int32_t *value)
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
	HAL.IOs->config->setLow(PIN_DRV_ENN);
};

static uint8_t reset()
{
	return 1;
}

static uint8_t restore()
{
	return 1;
}

static void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	Evalboards.ch1.errors = 0;
}

void TMC4670_init(void)
{
	// configure ENABLE-PIN for TMC4670
	PIN_DRV_ENN = &HAL.IOs->pins->DIO0;
	HAL.IOs->config->toOutput(PIN_DRV_ENN);
	enableDriver(DRIVER_ENABLE);

	TMC4670_SPIChannel = &HAL.SPI->ch1;
	TMC4670_SPIChannel->CSN = &HAL.IOs->pins->SPI1_CSN;

	TMC4670_config = Evalboards.ch1.config;

	// connect evalboard functions
	Evalboards.ch1.config->reset        = reset;
	Evalboards.ch1.config->restore      = restore;
	Evalboards.ch1.config->state        = CONFIG_READY;
	Evalboards.ch1.config->configIndex  = 0;

	Evalboards.ch1.rotate               = rotate;
	Evalboards.ch1.right                = right;
	Evalboards.ch1.left                 = left;
	Evalboards.ch1.stop                 = stop;
	Evalboards.ch1.getMeasuredSpeed     = getMeasuredSpeed;
	Evalboards.ch1.GAP                  = GAP;
	Evalboards.ch1.SAP                  = SAP;
	Evalboards.ch1.moveTo               = moveTo;
	Evalboards.ch1.moveBy               = moveBy;
	Evalboards.ch1.writeRegister        = writeRegister;
	Evalboards.ch1.readRegister         = readRegister;
	Evalboards.ch1.periodicJob          = periodicJob;
	Evalboards.ch1.userFunction         = userFunction;
	Evalboards.ch1.enableDriver         = enableDriver;
	Evalboards.ch1.checkErrors          = checkErrors;
	Evalboards.ch1.numberOfMotors       = TMC4670_MOTORS;
	Evalboards.ch1.deInit               = deInit;
	Evalboards.ch1.VMMin                = 50;
	Evalboards.ch1.VMMax                = 650;

	// init motor config
	for(int i = 0; i < TMC4670_MOTORS; i++)
	{
		motorConfig[i].initWaitTime             = 1000;
		motorConfig[i].startVoltage             = 6000;
		motorConfig[i].initMode                 = 0;
		motorConfig[i].torqueMeasurementFactor  = 256;
	}
};
