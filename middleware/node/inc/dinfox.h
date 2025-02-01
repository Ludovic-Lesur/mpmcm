/*
 * dinfox.h
 *
 *  Created on: 30 may. 2023
 *      Author: Ludo
 */

#ifndef __DINFOX_H__
#define __DINFOX_H__

#include "parser.h"
#include "string.h"
#include "types.h"

/*** DINFOX macros ***/

#define DINFOX_REG_SIZE_BYTES					4
#define DINFOX_REG_MASK_ALL						0xFFFFFFFF
#define DINFOX_REG_MASK_NONE					0x00000000

#define DINFOX_TIME_ERROR_VALUE					0xFF
#define DINFOX_TEMPERATURE_ERROR_VALUE			0x7F
#define DINFOX_HUMIDITY_ERROR_VALUE				0xFF
#define DINFOX_VOLTAGE_ERROR_VALUE				0xFFFF
#define DINFOX_CURRENT_ERROR_VALUE				0xFFFF
#define DINFOX_ELECTRICAL_POWER_ERROR_VALUE		0x7FFF
#define DINFOX_ELECTRICAL_ENERGY_ERROR_VALUE	0x7FFF
#define DINFOX_POWER_FACTOR_ERROR_VALUE			0x7F
#define DINFOX_RF_POWER_ERROR_VALUE				0xFF
#define DINFOX_MAINS_FREQUENCY_ERROR_VALUE		0xFFFF

#define DINFOX_NODE_ADDRESS_RANGE_R4S8CR		15

/*** DINFOX structures ***/

/*!******************************************************************
 * \enum DINFOX_board_id_t
 * \brief DINFox board identifiers list.
 *******************************************************************/
typedef enum {
	DINFOX_BOARD_ID_LVRM = 0,
	DINFOX_BOARD_ID_BPSM,
	DINFOX_BOARD_ID_DDRM,
	DINFOX_BOARD_ID_UHFM,
	DINFOX_BOARD_ID_GPSM,
	DINFOX_BOARD_ID_SM,
	DINFOX_BOARD_ID_DIM,
	DINFOX_BOARD_ID_RRM,
	DINFOX_BOARD_ID_DMM,
	DINFOX_BOARD_ID_MPMCM,
	DINFOX_BOARD_ID_R4S8CR,
	DINFOX_BOARD_ID_LAST,
	DINFOX_BOARD_ID_ERROR
} DINFOX_board_id_t;

/*!******************************************************************
 * \enum DINFOX_address_t
 * \brief DINFox boards address range.
 *******************************************************************/
typedef enum {
	DINFOX_NODE_ADDRESS_DMM = 0x00,
	DINFOX_NODE_ADDRESS_DIM = 0x01,
	DINFOX_NODE_ADDRESS_BPSM_START = 0x08,
	DINFOX_NODE_ADDRESS_UHFM_START = (DINFOX_NODE_ADDRESS_BPSM_START + 4),
	DINFOX_NODE_ADDRESS_GPSM_START = (DINFOX_NODE_ADDRESS_UHFM_START + 4),
	DINFOX_NODE_ADDRESS_SM_START = (DINFOX_NODE_ADDRESS_GPSM_START + 4),
	DINFOX_NODE_ADDRESS_RRM_START = (DINFOX_NODE_ADDRESS_SM_START + 4),
	DINFOX_NODE_ADDRESS_MPMCM_START = (DINFOX_NODE_ADDRESS_RRM_START + 4),
	DINFOX_NODE_ADDRESS_LVRM_START = (DINFOX_NODE_ADDRESS_MPMCM_START + 4),
	DINFOX_NODE_ADDRESS_DDRM_START = (DINFOX_NODE_ADDRESS_LVRM_START + 16),
	DINFOX_NODE_ADDRESS_LBUS_LAST = (DINFOX_NODE_ADDRESS_DDRM_START + 16),
	DINFOX_NODE_ADDRESS_R4S8CR_START = 0x70,
	DINFOX_NODE_ADDRESS_BROADCAST = (DINFOX_NODE_ADDRESS_R4S8CR_START + DINFOX_NODE_ADDRESS_RANGE_R4S8CR)
} DINFOX_address_t;

/*!******************************************************************
 * \enum DINFOX_register_access_t
 * \brief DINFox register accesses.
 *******************************************************************/
typedef enum {
	DINFOX_REG_ACCESS_READ_ONLY = 0,
	DINFOX_REG_ACCESS_READ_WRITE,
	DINFOX_REG_ACCESS_LAST
} DINFOX_register_access_t;

/*!******************************************************************
 * \type DINFOX_bit_representation_t
 * \brief DINFox bit representation type.
 *******************************************************************/
typedef enum {
	DINFOX_BIT_0 = 0b00,
	DINFOX_BIT_1 = 0b01,
	DINFOX_BIT_FORCED_HARDWARE = 0b10,
	DINFOX_BIT_ERROR = 0b11,
} DINFOX_bit_representation_t;

/*!******************************************************************
 * \type DINFOX_time_representation_t
 * \brief DINFox time representation type.
 *******************************************************************/
typedef uint8_t DINFOX_time_representation_t;

/*!******************************************************************
 * \type DINFOX_temperature_representation_t
 * \brief DINFox temperature representation type.
 *******************************************************************/
typedef uint8_t DINFOX_temperature_representation_t;

/*!******************************************************************
 * \type DINFOX_voltage_representation_t
 * \brief DINFox voltage representation type.
 *******************************************************************/
typedef uint16_t DINFOX_voltage_representation_t;

/*!******************************************************************
 * \type DINFOX_current_representation_t
 * \brief DINFox current representation type.
 *******************************************************************/
typedef uint16_t DINFOX_current_representation_t;

/*!******************************************************************
 * \type DINFOX_electrical_power_representation_t
 * \brief DINFox electrical power representation type.
 *******************************************************************/
typedef uint16_t DINFOX_electrical_power_representation_t;

/*!******************************************************************
 * \type DINFOX_electrical_energy_representation_t
 * \brief DINFox electrical energy representation type.
 *******************************************************************/
typedef uint16_t DINFOX_electrical_energy_representation_t;

/*!******************************************************************
 * \type DINFOX_power_factor_representation_t
 * \brief DINFox power factor representation type.
 *******************************************************************/
typedef uint8_t DINFOX_power_factor_representation_t;

/*!******************************************************************
 * \type DINFOX_rf_power_representation_t
 * \brief DINFox RF power representation type.
 *******************************************************************/
typedef uint8_t DINFOX_rf_power_representation_t;

/*!******************************************************************
 * \type DINFOX_year_representation_t
 * \brief DINFox year representation type.
 *******************************************************************/
typedef uint8_t DINFOX_year_representation_t;

/*** DINFOX functions ***/

/*!******************************************************************
 * \fn uint32_t DINFOX_read_nvm_register(uint8_t reg_addr)
 * \brief Read register value stored in NVM.
 * \param[in]  	reg_addr: Address of the register to read.
 * \param[out] 	none
 * \retval		Register value.
 *******************************************************************/
uint32_t DINFOX_read_nvm_register(uint8_t reg_addr);

/*!******************************************************************
 * \fn void DINFOX_write_nvm_register(uint8_t reg_addr, uint32_t reg_value)
 * \brief Store register value in NVM.
 * \param[in]  	reg_addr: Address of the register to write.
 * \param[in]	reg_value: Register value to write.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void DINFOX_write_nvm_register(uint8_t reg_addr, uint32_t reg_value);

/*!******************************************************************
 * \fn void DINFOX_write_field(uint32_t* reg_value, uint32_t* reg_mask, uint32_t field_value, uint32_t field_mask)
 * \brief Write a field in register.
 * \param[in]  	reg_value: Pointer to the register value.
 * \param[in]	field_value: Field value to write.
 * \param[in]	field_mask: Field mask.
 * \param[out] 	reg_mask: Pointer to the new register mask.
 * \retval		none
 *******************************************************************/
void DINFOX_write_field(uint32_t* reg_value, uint32_t* reg_mask, uint32_t field_value, uint32_t field_mask);

/*!******************************************************************
 * \fn uint32_t DINFOX_read_field(uint32_t reg_value, uint32_t field_mask)
 * \brief Read a field in register.
 * \param[in]  	reg_value: Register value.
 * \param[in]	field_mask: Field mask.
 * \param[out] 	none
 * \retval		Field value.
 *******************************************************************/
uint32_t DINFOX_read_field(uint32_t reg_value, uint32_t field_mask);

/*!******************************************************************
 * \fn PARSER_status_t DINFOX_parse_register(PARSER_context_t* parser_ctx, char_t separator, uint32_t* reg_value)
 * \brief Parse a node register within a character buffer.
 * \param[in]  	parser_ctx: Parser context.
 * \param[in]	separator: Field separator.
 * \param[out] 	reg_value: Pointer to integer that will contain the extracted register value.
 * \retval		Function execution status.
 *******************************************************************/
PARSER_status_t DINFOX_parse_register(PARSER_context_t* parser_ctx, char_t separator, uint32_t* reg_value);

/*!******************************************************************
 * \fn STRING_status_t DINFOX_register_to_string(uint32_t reg_value, char_t* str)
 * \brief Convert a register value to the corresponding string.
 * \param[in]  	reg_value: Register value to convert.
 * \param[out] 	str: Pointer to the destination string.
 * \retval		Function execution status.
 *******************************************************************/
STRING_status_t DINFOX_register_to_string(uint32_t reg_value, char_t* str);

/*!******************************************************************
 * \fn DINFOX_time_representation_t DINFOX_convert_seconds(uint32_t time_seconds)
 * \brief Convert a time to DINFox representation.
 * \param[in]  	time_seconds: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_time_representation_t DINFOX_convert_seconds(uint32_t time_seconds);

/*!******************************************************************
 * \fn uint32_t DINFOX_get_seconds(DINFOX_time_representation_t dinfox_time)
 * \brief Convert a DINFox representation to time.
 * \param[in]  	dinfox_time: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted time value in seconds.
 *******************************************************************/
uint32_t DINFOX_get_seconds(DINFOX_time_representation_t dinfox_time);

/*!******************************************************************
 * \fn DINFOX_temperature_representation_t DINFOX_convert_degrees(int8_t temperature_degrees)
 * \brief Convert a temperature to DINFox representation.
 * \param[in]  	temperature_degrees: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_temperature_representation_t DINFOX_convert_degrees(int8_t temperature_degrees);

/*!******************************************************************
 * \fn int8_t DINFOX_get_degrees(DINFOX_temperature_representation_t dinfox_temperature)
 * \brief Convert a DINFox representation to temperature.
 * \param[in]  	dinfox_temperature: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted temperature value in degrees.
 *******************************************************************/
int8_t DINFOX_get_degrees(DINFOX_temperature_representation_t dinfox_temperature);

/*!******************************************************************
 * \fn DINFOX_voltage_representation_t DINFOX_convert_mv(uint32_t voltage_mv)
 * \brief Convert a voltage to DINFox representation.
 * \param[in]  	voltage_mv: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_voltage_representation_t DINFOX_convert_mv(uint32_t voltage_mv);

/*!******************************************************************
 * \fn uint32_t DINFOX_get_mv(DINFOX_voltage_representation_t dinfox_voltage)
 * \brief Convert a DINFox representation to voltage.
 * \param[in]  	dinfox_voltage: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted voltage value in mV.
 *******************************************************************/
uint32_t DINFOX_get_mv(DINFOX_voltage_representation_t dinfox_voltage);

/*!******************************************************************
 * \fn DINFOX_current_representation_t DINFOX_convert_ua(uint32_t current_ua)
 * \brief Convert a current to DINFox representation.
 * \param[in]  	voltage_mv: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_current_representation_t DINFOX_convert_ua(uint32_t current_ua);

/*!******************************************************************
 * \fn uint32_t DINFOX_get_ua(DINFOX_current_representation_t dinfox_current)
 * \brief Convert a DINFox representation to current.
 * \param[in]  	dinfox_current: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted current value in uA.
 *******************************************************************/
uint32_t DINFOX_get_ua(DINFOX_current_representation_t dinfox_current);

/*!******************************************************************
 * \fn DINFOX_electrical_power_representation_t DINFOX_convert_mw_mva(int32_t electrical_power_mw_mva)
 * \brief Convert an electrical power to DINFox representation.
 * \param[in]  	electrical_power_mw_mva: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_electrical_power_representation_t DINFOX_convert_mw_mva(int32_t electrical_power_mw_mva);

/*!******************************************************************
 * \fn int32_t DINFOX_get_mw_mva(DINFOX_electrical_power_representation_t dinfox_electrical_power)
 * \brief Convert a DINFox representation to electrical power.
 * \param[in]  	dinfox_electrical_power: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted electrical power value in mW or mVA.
 *******************************************************************/
int32_t DINFOX_get_mw_mva(DINFOX_electrical_power_representation_t dinfox_electrical_power);

/*!******************************************************************
 * \fn DINFOX_electrical_energy_representation_t DINFOX_convert_mwh_mvah(int32_t electrical_energy_mwh_mvah)
 * \brief Convert an electrical energy to DINFox representation.
 * \param[in]  	electrical_energy_mwh_mvah: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_electrical_energy_representation_t DINFOX_convert_mwh_mvah(int32_t electrical_energy_mwh_mvah);

/*!******************************************************************
 * \fn int32_t DINFOX_get_mwh_mvah(DINFOX_electrical_energy_representation_t dinfox_electrical_energy)
 * \brief Convert a DINFox representation to electrical energy.
 * \param[in]  	dinfox_electrical_energy: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted electrical energy value in mWh or mVAh.
 *******************************************************************/
int32_t DINFOX_get_mwh_mvah(DINFOX_electrical_energy_representation_t dinfox_electrical_energy);

/*!******************************************************************
 * \fn DINFOX_power_factor_representation_t DINFOX_convert_power_factor(int32_t power_factor)
 * \brief Convert an power factor to DINFox representation.
 * \param[in]  	power_factor: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_power_factor_representation_t DINFOX_convert_power_factor(int32_t power_factor);

/*!******************************************************************
 * \fn int32_t DINFOX_get_power_factor(DINFOX_power_factor_representation_t dinfox_power_factor)
 * \brief Convert a DINFox representation to electrical power.
 * \param[in]  	dinfox_electrical_power: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted power factor.
 *******************************************************************/
int32_t DINFOX_get_power_factor(DINFOX_power_factor_representation_t dinfox_power_factor);

/*!******************************************************************
 * \fn DINFOX_rf_power_representation_t DINFOX_convert_dbm(int16_t rf_power_dbm)
 * \brief Convert an RF power to DINFox representation.
 * \param[in]  	rf_power_dbm: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_rf_power_representation_t DINFOX_convert_dbm(int16_t rf_power_dbm);

/*!******************************************************************
 * \fn int16_t DINFOX_get_dbm(DINFOX_rf_power_representation_t dinfox_rf_power)
 * \brief Convert a DINFox representation to current.
 * \param[in]  	dinfox_rf_power: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted RF power value in dBm.
 *******************************************************************/
int16_t DINFOX_get_dbm(DINFOX_rf_power_representation_t dinfox_rf_power);

/*!******************************************************************
 * \fn DINFOX_year_representation_t DINFOX_convert_year(uint16_t year)
 * \brief Convert a year to DINFox representation.
 * \param[in]  	year: Value to convert
 * \param[out] 	none
 * \retval		DINFox representation.
 *******************************************************************/
DINFOX_year_representation_t DINFOX_convert_year(uint16_t year);

/*!******************************************************************
 * \fn uint16_t DINFOX_get_year(DINFOX_year_representation_t dinfox_year)
 * \brief Convert a DINFox representation to year.
 * \param[in]  	dinfox_year: DINFox representation to convert.
 * \param[out] 	none
 * \retval		Converted year value.
 *******************************************************************/
uint16_t DINFOX_get_year(DINFOX_year_representation_t dinfox_year);

#endif /* __DINFOX_H__ */
