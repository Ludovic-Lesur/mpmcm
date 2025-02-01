/*
 * dinfox.c
 *
 *  Created on: 30 may. 2023
 *      Author: Ludo
 */

#include "dinfox.h"

#include "error.h"
#include "parser.h"
#include "types.h"
#ifdef MPMCM
#include "flash.h"
#include "math_custom.h"
#include "string_custom.h"
#else
#include "math.h"
#include "nvm.h"
#include "string.h"
#endif

/*** DINFOX local macros ***/

#define DINFOX_SIGN_SIZE_BITS						1

#define DINFOX_TIME_UNIT_SIZE_BITS					2
#define DINFOX_TIME_VALUE_SIZE_BITS					6

#define DINFOX_TEMPERATURE_VALUE_SIZE_BITS			7

#define DINFOX_VOLTAGE_UNIT_SIZE_BITS				1
#define DINFOX_VOLTAGE_VALUE_SIZE_BITS				15

#define DINFOX_CURRENT_UNIT_SIZE_BITS				2
#define DINFOX_CURRENT_VALUE_SIZE_BITS				14

#define DINFOX_ELECTRICAL_POWER_UNIT_SIZE_BITS		2
#define DINFOX_ELECTRICAL_POWER_VALUE_SIZE_BITS		13

#define DINFOX_ELECTRICAL_ENERGY_UNIT_SIZE_BITS		2
#define DINFOX_ELECTRICAL_ENERGY_VALUE_SIZE_BITS	13

#define DINFOX_POWER_FACTOR_VALUE_SIZE_BITS			7

#define DINFOX_SECONDS_PER_MINUTE					60
#define DINFOX_MINUTES_PER_HOUR						60
#define DINFOX_HOURS_PER_DAY						24

#define DINFOX_MV_PER_DV							100

#define DINFOX_UA_PER_DMA							100
#define DINFOX_DMA_PER_MA							10
#define DINFOX_MA_PER_DA							100

#define DINFOX_MW_MVA_PER_DW_DVA					100
#define DINFOX_DW_DVA_PER_W_VA						10
#define DINFOX_W_VA_PER_DAW_DAVA					10

#define DINFOX_MWH_MVAH_PER_DWH_DVAH				100
#define DINFOX_DWH_DVAH_PER_WH_VAH					10
#define DINFOX_WH_VAH_PER_DAWH_DAVAH				10

#define DINFOX_RF_POWER_OFFSET						174

#define DINFOX_YEAR_OFFSET							2000

/*** DINFOX local structures ***/

/*******************************************************************/
typedef enum {
	DINFOX_SIGN_POSITIVE = 0,
	DINFOX_SIGN_NEGATIVE
} DINFOX_sign_t;

/*******************************************************************/
typedef enum {
	DINFOX_TIME_UNIT_SECOND = 0,
	DINFOX_TIME_UNIT_MINUTE,
	DINFOX_TIME_UNIT_HOUR,
	DINFOX_TIME_UNIT_DAY
} DINFOX_time_unit_t;

/*******************************************************************/
typedef enum {
	DINFOX_VOLTAGE_UNIT_MV = 0,
	DINFOX_VOLTAGE_UNIT_DV
} DINFOX_voltage_unit_t;

/*******************************************************************/
typedef enum {
	DINFOX_CURRENT_UNIT_UA = 0,
	DINFOX_CURRENT_UNIT_DMA,
	DINFOX_CURRENT_UNIT_MA,
	DINFOX_CURRENT_UNIT_DA
} DINFOX_current_unit_t;

/*******************************************************************/
typedef enum {
	DINFOX_ELECTRICAL_POWER_UNIT_MW_MVA = 0,
	DINFOX_ELECTRICAL_POWER_UNIT_DW_DVA,
	DINFOX_ELECTRICAL_POWER_UNIT_W_VA,
	DINFOX_ELECTRICAL_POWER_UNIT_DAW_DAVA
} DINFOX_electrical_power_unit_t;

/*******************************************************************/
typedef enum {
	DINFOX_ELECTRICAL_ENERGY_UNIT_MWH_MVAH = 0,
	DINFOX_ELECTRICAL_ENERGY_UNIT_DWH_DVAH,
	DINFOX_ELECTRICAL_ENERGY_UNIT_WH_VAH,
	DINFOX_ELECTRICAL_ENERGY_UNIT_DAWH_DAVAH
} DINFOX_electrical_energy_unit_t;

/*******************************************************************/
typedef union {
	DINFOX_time_representation_t representation;
	struct {
		unsigned value : DINFOX_TIME_VALUE_SIZE_BITS;
		DINFOX_time_unit_t unit : DINFOX_TIME_UNIT_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_time_t;

/*******************************************************************/
typedef union {
	DINFOX_temperature_representation_t representation;
	struct {
		unsigned value : DINFOX_TEMPERATURE_VALUE_SIZE_BITS;
		DINFOX_sign_t sign : DINFOX_SIGN_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_temperature_t;

/*******************************************************************/
typedef union {
	DINFOX_voltage_representation_t representation;
	struct {
		unsigned value : DINFOX_VOLTAGE_VALUE_SIZE_BITS;
		DINFOX_voltage_unit_t unit : DINFOX_VOLTAGE_UNIT_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_voltage_t;

/*******************************************************************/
typedef union {
	DINFOX_current_representation_t representation;
	struct {
		unsigned value : DINFOX_CURRENT_VALUE_SIZE_BITS;
		DINFOX_current_unit_t unit : DINFOX_CURRENT_UNIT_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_current_t;

/*******************************************************************/
typedef union {
	DINFOX_electrical_power_representation_t representation;
	struct {
		unsigned value : DINFOX_ELECTRICAL_POWER_VALUE_SIZE_BITS;
		DINFOX_electrical_power_unit_t unit : DINFOX_ELECTRICAL_POWER_UNIT_SIZE_BITS;
		DINFOX_sign_t sign : DINFOX_SIGN_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_electrical_power_t;

/*******************************************************************/
typedef union {
	DINFOX_electrical_energy_representation_t representation;
	struct {
		unsigned value : DINFOX_ELECTRICAL_ENERGY_VALUE_SIZE_BITS;
		DINFOX_electrical_energy_unit_t unit : DINFOX_ELECTRICAL_ENERGY_UNIT_SIZE_BITS;
		DINFOX_sign_t sign : DINFOX_SIGN_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_electrical_energy_t;

/*******************************************************************/
typedef union {
	DINFOX_power_factor_representation_t representation;
	struct {
		unsigned value : DINFOX_POWER_FACTOR_VALUE_SIZE_BITS;
		DINFOX_sign_t sign : DINFOX_SIGN_SIZE_BITS;
	} __attribute__((scalar_storage_order("little-endian"))) __attribute__((packed));
} DINFOX_power_factor_t;

/*** DINFOX local functions ***/

/*******************************************************************/
static uint8_t _DINFOX_get_shift(uint32_t field_mask) {
	// Local variables.
	uint8_t offset = 0;
	// Compute shift according to mask.
	for (offset=0 ; offset<(8 * DINFOX_REG_SIZE_BYTES) ; offset++) {
		if (((field_mask >> offset) & 0x01) != 0) {
			break;
		}
	}
	return offset;
}

/*** DINFOX functions ***/

/*******************************************************************/
uint32_t DINFOX_read_nvm_register(uint8_t reg_addr) {
	// Local variables.
	uint32_t reg_value = 0;
#ifdef MPMCM
	FLASH_status_t flash_status = FLASH_SUCCESS;
	// Read NVM.
	flash_status = FLASH_read_word((FLASH_ADDRESS_REGISTERS + reg_addr), &reg_value);
	FLASH_stack_error();
#else
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t nvm_byte = 0;
	uint8_t idx = 0;
	// Byte loop.
	for (idx=0 ; idx<4 ; idx++) {
		// Read NVM.
		nvm_status = NVM_read_byte((NVM_ADDRESS_REGISTERS + (reg_addr << 2) + idx), &nvm_byte);
		NVM_stack_error();
		if (nvm_status != NVM_SUCCESS) {
			reg_value = 0;
			break;
		}
		reg_value |= ((uint32_t) nvm_byte) << (idx << 3);
	}
#endif
	return reg_value;
}

/*******************************************************************/
void DINFOX_write_nvm_register(uint8_t reg_addr, uint32_t reg_value) {
#ifdef MPMCM
	// Local variables.
	FLASH_status_t flash_status = FLASH_SUCCESS;
	// Write NVM.
	flash_status = FLASH_write_word((FLASH_ADDRESS_REGISTERS + reg_addr), reg_value);
	FLASH_stack_error();
#else
	// Local variables.
	NVM_status_t nvm_status = NVM_SUCCESS;
	uint8_t nvm_byte = 0;
	uint8_t idx = 0;
	// Byte loop.
	for (idx=0 ; idx<4 ; idx++) {
		// Compute byte.
		nvm_byte = (uint8_t) (((reg_value) >> (idx << 3)) & 0x000000FF);
		// Write NVM.
		nvm_status = NVM_write_byte((NVM_ADDRESS_REGISTERS + (reg_addr << 2) + idx), nvm_byte);
		NVM_stack_error();
		if (nvm_status != NVM_SUCCESS) break;
	}
#endif
}

/*******************************************************************/
void DINFOX_write_field(uint32_t* reg_value, uint32_t* reg_mask, uint32_t field_value, uint32_t field_mask) {
	// Reset bits.
	(*reg_value) &= ~(field_mask);
	// Set field.
	(*reg_value) |= ((field_value << _DINFOX_get_shift(field_mask)) & field_mask);
	// Update mask.
	(*reg_mask) |= field_mask;
}

/*******************************************************************/
uint32_t DINFOX_read_field(uint32_t reg_value, uint32_t field_mask) {
	// Isolate field.
	return ((reg_value & field_mask) >> _DINFOX_get_shift(field_mask));
}

/*******************************************************************/
PARSER_status_t DINFOX_parse_register(PARSER_context_t* parser_ctx, char_t separator, uint32_t* reg_value) {
	// Local variables.
	PARSER_status_t status = PARSER_SUCCESS;
	uint8_t reg_bytes[DINFOX_REG_SIZE_BYTES];
	uint8_t reg_size_bytes = 0;
	uint8_t idx = 0;
	// Parse register as byte array.
	status = PARSER_get_byte_array(parser_ctx, separator, DINFOX_REG_SIZE_BYTES, 0, (uint8_t*) reg_bytes, &reg_size_bytes);
	if (status != PARSER_SUCCESS) goto errors;
	// Convert byte array to 32 bits value.
	(*reg_value) = 0;
	for (idx=0 ; idx<reg_size_bytes ; idx++) {
		(*reg_value) |= (reg_bytes[idx] << (8 * (reg_size_bytes - 1 - idx)));
	}
errors:
	return status;
}

/*******************************************************************/
STRING_status_t DINFOX_register_to_string(uint32_t reg_value, char_t* str) {
	// Local variables.
	STRING_status_t status = STRING_SUCCESS;
	uint8_t byte = 0;
	uint8_t idx = 0;
	uint8_t str_idx = 0;
	uint8_t first_non_zero_found = 0;
	// Convert 32-bits value to byte array.
	for (idx=0 ; idx<DINFOX_REG_SIZE_BYTES ; idx++) {
		// Compute byte.
		byte = (uint8_t) ((reg_value >> (8 * (DINFOX_REG_SIZE_BYTES - 1 - idx))) & 0xFF);
		// Update flag.
		if (byte != 0) {
			first_non_zero_found = 1;
		}
		// Check flag and index.
		if ((first_non_zero_found != 0) || (idx == (DINFOX_REG_SIZE_BYTES - 1))) {
			// Convert byte.
			status = STRING_value_to_string((int32_t) byte, STRING_FORMAT_HEXADECIMAL, 0, &(str[2 * str_idx]));
			str_idx++;
			// Check status.
			if (status != STRING_SUCCESS) break;
		}
	}
	str[2 * str_idx] = STRING_CHAR_NULL;
	return status;
}

/*******************************************************************/
DINFOX_time_representation_t DINFOX_convert_seconds(uint32_t time_seconds) {
	// Local variables.
	DINFOX_time_t dinfox_time;
	uint32_t value = time_seconds;
	// Select unit.
	if (value < (0b1 << DINFOX_TIME_VALUE_SIZE_BITS)) {
		dinfox_time.unit = DINFOX_TIME_UNIT_SECOND;
	}
	else {
		value /= DINFOX_SECONDS_PER_MINUTE;
		if (value < (0b1 << DINFOX_TIME_VALUE_SIZE_BITS)) {
			dinfox_time.unit = DINFOX_TIME_UNIT_MINUTE;
		}
		else {
			value /= DINFOX_MINUTES_PER_HOUR;
			if (value < (0b1 << DINFOX_TIME_VALUE_SIZE_BITS)) {
				dinfox_time.unit = DINFOX_TIME_UNIT_HOUR;
			}
			else {
				value /= DINFOX_HOURS_PER_DAY;
				dinfox_time.unit = DINFOX_TIME_UNIT_DAY;
			}
		}
	}
	dinfox_time.value = value;
	return (dinfox_time.representation);
}

/*******************************************************************/
uint32_t DINFOX_get_seconds(DINFOX_time_representation_t dinfox_time) {
	// Local variables.
	uint32_t time_seconds = 0;
	uint32_t value = 0;
	DINFOX_time_representation_t local_dinfox_time = dinfox_time;
	DINFOX_time_unit_t unit = DINFOX_TIME_UNIT_SECOND;
	// Parse fields.
	unit = ((DINFOX_time_t*) &local_dinfox_time) -> unit;
	value = (uint32_t) ((DINFOX_time_t*) &local_dinfox_time) -> value;
	// Compute seconds.
	switch (unit) {
	case DINFOX_TIME_UNIT_SECOND:
		time_seconds = value;
		break;
	case DINFOX_TIME_UNIT_MINUTE:
		time_seconds = (DINFOX_SECONDS_PER_MINUTE * value);
		break;
	case DINFOX_TIME_UNIT_HOUR:
		time_seconds = (DINFOX_MINUTES_PER_HOUR * DINFOX_SECONDS_PER_MINUTE * value);
		break;
	default:
		time_seconds = (DINFOX_HOURS_PER_DAY * DINFOX_MINUTES_PER_HOUR * DINFOX_SECONDS_PER_MINUTE * value);
		break;
	}
	return time_seconds;
}

/*******************************************************************/
DINFOX_temperature_representation_t DINFOX_convert_degrees(int8_t temperature_degrees) {
	// Local variables.
	uint32_t dinfox_temperature = 0;
	int32_t temp_degrees = (int32_t) temperature_degrees;
	// DINFox representation is equivalent to signed magnitude
	MATH_int32_to_signed_magnitude(temp_degrees, DINFOX_TEMPERATURE_VALUE_SIZE_BITS, &dinfox_temperature);
	return ((DINFOX_temperature_representation_t) dinfox_temperature);
}

/*******************************************************************/
int8_t DINFOX_get_degrees(DINFOX_temperature_representation_t dinfox_temperature) {
	// Local variables.
	int8_t temperature_degrees = 0;
	uint32_t value = 0;
	DINFOX_temperature_representation_t local_dinfox_temperature = dinfox_temperature;
	DINFOX_sign_t sign = DINFOX_SIGN_POSITIVE;
	// Parse fields.
	sign = (uint32_t) (((DINFOX_temperature_t*) &local_dinfox_temperature) -> sign);
	value = (uint32_t) (((DINFOX_temperature_t*) &local_dinfox_temperature) -> value);
	// Check sign.
	temperature_degrees = (sign == DINFOX_SIGN_POSITIVE) ? (value) : ((-1) * value);
	return temperature_degrees;
}

/*******************************************************************/
DINFOX_voltage_representation_t DINFOX_convert_mv(uint32_t voltage_mv) {
	// Local variables.
	DINFOX_voltage_t dinfox_voltage;
	// Select format.
	if (voltage_mv < (0b1 << DINFOX_VOLTAGE_VALUE_SIZE_BITS)) {
		dinfox_voltage.unit = DINFOX_VOLTAGE_UNIT_MV;
		dinfox_voltage.value = voltage_mv;
	}
	else {
		dinfox_voltage.unit = DINFOX_VOLTAGE_UNIT_DV;
		dinfox_voltage.value = (voltage_mv / DINFOX_MV_PER_DV);
	}
	return (dinfox_voltage.representation);
}

/*******************************************************************/
uint32_t DINFOX_get_mv(DINFOX_voltage_representation_t dinfox_voltage) {
	// Local variables.
	uint32_t voltage_mv = 0;
	uint32_t value = 0;
	DINFOX_voltage_representation_t local_dinfox_voltage = dinfox_voltage;
	DINFOX_voltage_unit_t unit = DINFOX_VOLTAGE_UNIT_MV;
	// Parse fields.
	unit = ((DINFOX_voltage_t*) &local_dinfox_voltage) -> unit;
	value = (uint32_t) ((DINFOX_voltage_t*) &local_dinfox_voltage) -> value;
	// Compute mV.
	voltage_mv = (unit == DINFOX_VOLTAGE_UNIT_MV) ? (value) : (value * DINFOX_MV_PER_DV);
	return voltage_mv;
}

/*******************************************************************/
DINFOX_current_representation_t DINFOX_convert_ua(uint32_t current_ua) {
	// Local variables.
	DINFOX_current_t dinfox_current;
	uint32_t value = current_ua;
	// Select unit.
	if (value < (0b1 << DINFOX_CURRENT_VALUE_SIZE_BITS)) {
		dinfox_current.unit = DINFOX_CURRENT_UNIT_UA;
	}
	else {
		value /= DINFOX_UA_PER_DMA;
		if (value < (0b1 << DINFOX_CURRENT_VALUE_SIZE_BITS)) {
			dinfox_current.unit = DINFOX_CURRENT_UNIT_DMA;
		}
		else {
			value /= DINFOX_DMA_PER_MA;
			if (value < (0b1 << DINFOX_CURRENT_VALUE_SIZE_BITS)) {
				dinfox_current.unit = DINFOX_CURRENT_UNIT_MA;
			}
			else {
				value /= DINFOX_MA_PER_DA;
				dinfox_current.unit = DINFOX_CURRENT_UNIT_DA;
			}
		}
	}
	dinfox_current.value = value;
	return (dinfox_current.representation);
}

/*******************************************************************/
uint32_t DINFOX_get_ua(DINFOX_current_representation_t dinfox_current) {
	// Local variables.
	uint32_t current_ua = 0;
	uint32_t value = 0;
	DINFOX_current_representation_t local_dinfox_current = dinfox_current;
	DINFOX_current_unit_t unit = DINFOX_CURRENT_UNIT_UA;
	// Parse fields.
	unit = ((DINFOX_current_t*) &local_dinfox_current) -> unit;
	value = (uint32_t) ((DINFOX_current_t*) &local_dinfox_current) -> value;
	// Compute seconds.
	switch (unit) {
	case DINFOX_CURRENT_UNIT_UA:
		current_ua = value;
		break;
	case DINFOX_CURRENT_UNIT_DMA:
		current_ua = (DINFOX_UA_PER_DMA * value);
		break;
	case DINFOX_CURRENT_UNIT_MA:
		current_ua = (DINFOX_UA_PER_DMA * DINFOX_DMA_PER_MA * value);
		break;
	default:
		current_ua = (DINFOX_UA_PER_DMA * DINFOX_DMA_PER_MA * DINFOX_MA_PER_DA * value);
		break;
	}
	return current_ua;
}

/*******************************************************************/
DINFOX_electrical_power_representation_t DINFOX_convert_mw_mva(int32_t electrical_power_mw_mva) {
	// Local variables.
	DINFOX_electrical_power_t dinfox_electrical_power;
	uint32_t absolute_value = 0;
	// Read absolute value.
	MATH_abs(electrical_power_mw_mva, absolute_value);
	// Select sign.
	dinfox_electrical_power.sign = (electrical_power_mw_mva < 0) ? DINFOX_SIGN_NEGATIVE : DINFOX_SIGN_POSITIVE;
	// Select unit.
	if (absolute_value < (0b1 << DINFOX_ELECTRICAL_POWER_VALUE_SIZE_BITS)) {
		dinfox_electrical_power.unit = DINFOX_ELECTRICAL_POWER_UNIT_MW_MVA;
	}
	else {
		absolute_value /= DINFOX_MW_MVA_PER_DW_DVA;
		if (absolute_value < (0b1 << DINFOX_ELECTRICAL_POWER_VALUE_SIZE_BITS)) {
			dinfox_electrical_power.unit = DINFOX_ELECTRICAL_POWER_UNIT_DW_DVA;
		}
		else {
			absolute_value /= DINFOX_DW_DVA_PER_W_VA;
			if (absolute_value < (0b1 << DINFOX_ELECTRICAL_POWER_VALUE_SIZE_BITS)) {
				dinfox_electrical_power.unit = DINFOX_ELECTRICAL_POWER_UNIT_W_VA;
			}
			else {
				absolute_value /= DINFOX_W_VA_PER_DAW_DAVA;
				dinfox_electrical_power.unit = DINFOX_ELECTRICAL_POWER_UNIT_DAW_DAVA;
			}
		}
	}
	dinfox_electrical_power.value = absolute_value;
	return (dinfox_electrical_power.representation);
}

/*******************************************************************/
int32_t DINFOX_get_mw_mva(DINFOX_electrical_power_representation_t dinfox_electrical_power) {
	// Local variables.
	int32_t electrical_power_mw_mva = 0;
	int32_t absolute_value = 0;
	int32_t sign_multiplicator = 0;
	DINFOX_electrical_power_representation_t local_dinfox_electrical_power = dinfox_electrical_power;
	DINFOX_electrical_power_unit_t unit = DINFOX_ELECTRICAL_POWER_UNIT_MW_MVA;
	DINFOX_sign_t sign = DINFOX_SIGN_POSITIVE;
	// Parse fields.
	sign = ((DINFOX_electrical_power_t*) &local_dinfox_electrical_power) -> sign;
	unit = ((DINFOX_electrical_power_t*) &local_dinfox_electrical_power) -> unit;
	absolute_value = (int32_t) ((DINFOX_electrical_power_t*) &local_dinfox_electrical_power) -> value;
	// Compute multiplicator.
	sign_multiplicator = (sign == DINFOX_SIGN_NEGATIVE) ? (-1) : (1);
	// Compute seconds.
	switch (unit) {
	case DINFOX_ELECTRICAL_POWER_UNIT_MW_MVA:
		electrical_power_mw_mva = (absolute_value * sign_multiplicator);
		break;
	case DINFOX_ELECTRICAL_POWER_UNIT_DW_DVA:
		electrical_power_mw_mva = (DINFOX_MW_MVA_PER_DW_DVA * absolute_value * sign_multiplicator);
		break;
	case DINFOX_ELECTRICAL_POWER_UNIT_W_VA:
		electrical_power_mw_mva = (DINFOX_MW_MVA_PER_DW_DVA * DINFOX_DW_DVA_PER_W_VA * absolute_value * sign_multiplicator);
		break;
	default:
		electrical_power_mw_mva = (DINFOX_MW_MVA_PER_DW_DVA * DINFOX_DW_DVA_PER_W_VA * DINFOX_W_VA_PER_DAW_DAVA * absolute_value * sign_multiplicator);
		break;
	}
	return electrical_power_mw_mva;
}

/*******************************************************************/
DINFOX_electrical_energy_representation_t DINFOX_convert_mwh_mvah(int32_t electrical_energy_mwh_mvah) {
	// Local variables.
	DINFOX_electrical_energy_t dinfox_electrical_energy;
	uint32_t absolute_value = 0;
	// Read absolute value.
	MATH_abs(electrical_energy_mwh_mvah, absolute_value);
	// Select sign.
	dinfox_electrical_energy.sign = (electrical_energy_mwh_mvah < 0) ? DINFOX_SIGN_NEGATIVE : DINFOX_SIGN_POSITIVE;
	// Select unit.
	if (absolute_value < (0b1 << DINFOX_ELECTRICAL_ENERGY_VALUE_SIZE_BITS)) {
		dinfox_electrical_energy.unit = DINFOX_ELECTRICAL_ENERGY_UNIT_MWH_MVAH;
	}
	else {
		absolute_value /= DINFOX_MWH_MVAH_PER_DWH_DVAH;
		if (absolute_value < (0b1 << DINFOX_ELECTRICAL_ENERGY_VALUE_SIZE_BITS)) {
			dinfox_electrical_energy.unit = DINFOX_ELECTRICAL_ENERGY_UNIT_DWH_DVAH;
		}
		else {
			absolute_value /= DINFOX_DWH_DVAH_PER_WH_VAH;
			if (absolute_value < (0b1 << DINFOX_ELECTRICAL_ENERGY_VALUE_SIZE_BITS)) {
				dinfox_electrical_energy.unit = DINFOX_ELECTRICAL_ENERGY_UNIT_WH_VAH;
			}
			else {
				absolute_value /= DINFOX_WH_VAH_PER_DAWH_DAVAH;
				dinfox_electrical_energy.unit = DINFOX_ELECTRICAL_ENERGY_UNIT_DAWH_DAVAH;
			}
		}
	}
	dinfox_electrical_energy.value = absolute_value;
	return (dinfox_electrical_energy.representation);
}

/*******************************************************************/
int32_t DINFOX_get_mwh_mvah(DINFOX_electrical_energy_representation_t dinfox_electrical_energy) {
	// Local variables.
	int32_t electrical_energy_mwh_mvah = 0;
	int32_t absolute_value = 0;
	int32_t sign_multiplicator = 0;
	DINFOX_electrical_energy_representation_t local_dinfox_electrical_energy = dinfox_electrical_energy;
	DINFOX_electrical_energy_unit_t unit = DINFOX_ELECTRICAL_ENERGY_UNIT_MWH_MVAH;
	DINFOX_sign_t sign = DINFOX_SIGN_POSITIVE;
	// Parse fields.
	sign = ((DINFOX_electrical_energy_t*) &local_dinfox_electrical_energy) -> sign;
	unit = ((DINFOX_electrical_energy_t*) &local_dinfox_electrical_energy) -> unit;
	absolute_value = (int32_t) ((DINFOX_electrical_energy_t*) &local_dinfox_electrical_energy) -> value;
	// Compute multiplicator.
	sign_multiplicator = (sign == DINFOX_SIGN_NEGATIVE) ? (-1) : (1);
	// Compute seconds.
	switch (unit) {
	case DINFOX_ELECTRICAL_ENERGY_UNIT_MWH_MVAH:
		electrical_energy_mwh_mvah = (absolute_value * sign_multiplicator);
		break;
	case DINFOX_ELECTRICAL_ENERGY_UNIT_DWH_DVAH:
		electrical_energy_mwh_mvah = (DINFOX_MWH_MVAH_PER_DWH_DVAH * absolute_value * sign_multiplicator);
		break;
	case DINFOX_ELECTRICAL_ENERGY_UNIT_WH_VAH:
		electrical_energy_mwh_mvah = (DINFOX_MWH_MVAH_PER_DWH_DVAH * DINFOX_DWH_DVAH_PER_WH_VAH * absolute_value * sign_multiplicator);
		break;
	default:
		electrical_energy_mwh_mvah = (DINFOX_MWH_MVAH_PER_DWH_DVAH * DINFOX_DWH_DVAH_PER_WH_VAH * DINFOX_WH_VAH_PER_DAWH_DAVAH * absolute_value * sign_multiplicator);
		break;
	}
	return electrical_energy_mwh_mvah;
}

/*******************************************************************/
DINFOX_power_factor_representation_t DINFOX_convert_power_factor(int32_t power_factor) {
	// Local variables.
	uint32_t dinfox_power_factor = 0;
	// DINFox representation is equivalent to signed magnitude
	MATH_int32_to_signed_magnitude(power_factor, DINFOX_POWER_FACTOR_VALUE_SIZE_BITS, &dinfox_power_factor);
	return ((DINFOX_power_factor_representation_t) dinfox_power_factor);
}

/*******************************************************************/
int32_t DINFOX_get_power_factor(DINFOX_power_factor_representation_t dinfox_power_factor) {
	// Local variables.
	int32_t power_factor = 0;
	uint32_t value = 0;
	DINFOX_power_factor_representation_t local_dinfox_power_factor = dinfox_power_factor;
	DINFOX_sign_t sign = DINFOX_SIGN_POSITIVE;
	// Parse fields.
	sign = (((DINFOX_power_factor_t*) &local_dinfox_power_factor) -> sign);
	value = (uint32_t) (((DINFOX_power_factor_t*) &local_dinfox_power_factor) -> value);
	// Check sign.
	power_factor = (sign == DINFOX_SIGN_POSITIVE) ? (value) : ((-1) * value);
	return power_factor;
}

/*******************************************************************/
DINFOX_rf_power_representation_t DINFOX_convert_dbm(int16_t rf_power_dbm) {
	return ((uint8_t) (rf_power_dbm + DINFOX_RF_POWER_OFFSET));
}

/*******************************************************************/
int16_t DINFOX_get_dbm(DINFOX_rf_power_representation_t dinfox_rf_power) {
	return ((uint16_t) (dinfox_rf_power - DINFOX_RF_POWER_OFFSET));
}

/*******************************************************************/
DINFOX_year_representation_t DINFOX_convert_year(uint16_t year) {
	return ((uint8_t) (year - DINFOX_YEAR_OFFSET));
}

/*******************************************************************/
uint16_t DINFOX_get_year(DINFOX_year_representation_t dinfox_year) {
	return ((uint16_t) (dinfox_year + DINFOX_YEAR_OFFSET));
}
