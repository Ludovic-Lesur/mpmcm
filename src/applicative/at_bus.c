/*
 * at.c
 *
 *  Created on: 18 apr. 2020
 *      Author: Ludo
 */

#include "at_bus.h"

#include "common_reg.h"
#include "dinfox.h"
#include "error.h"
#include "lbus.h"
#include "measure.h"
#include "mode.h"
#include "node_common.h"
#include "parser.h"
#include "pwr.h"
#include "string_custom.h"
#include "types.h"

/*** AT local macros ***/

// Commands.
#define AT_BUS_COMMAND_BUFFER_SIZE			128
// Parameters separator.
#define AT_BUS_CHAR_SEPARATOR				','
// Replies.
#define AT_BUS_REPLY_BUFFER_SIZE			128
#define AT_BUS_STRING_VALUE_BUFFER_SIZE		16
#define AT_BUS_FRAME_END					STRING_CHAR_CR
#define AT_BUS_REPLY_TAB					"     "
#ifdef UHFM
// Duration of RSSI command.
#define AT_BUS_RSSI_REPORT_PERIOD_MS		500
#endif

/*** AT callbacks declaration ***/

static void _AT_BUS_print_ok(void);
#ifdef ATM
static void _AT_BUS_print_command_list(void);
static void _AT_BUS_print_sw_version(void);
static void _AT_BUS_print_error_stack(void);
#endif

/*** AT local structures ***/

typedef struct {
	PARSER_mode_t mode;
	char_t* syntax;
	char_t* parameters;
	char_t* description;
	void (*callback)(void);
} AT_BUS_command_t;

typedef struct {
	// Command.
	volatile char_t command[AT_BUS_COMMAND_BUFFER_SIZE];
	volatile uint32_t command_size;
	volatile uint8_t line_end_flag;
	PARSER_context_t parser;
	// Replies.
	char_t reply[AT_BUS_REPLY_BUFFER_SIZE];
	uint32_t reply_size;
} AT_BUS_context_t;

/*** AT local global variables ***/

static const AT_BUS_command_t AT_BUS_COMMAND_LIST[] = {
#ifdef ATM
	{PARSER_MODE_COMMAND, "AT", STRING_NULL, "Ping command", _AT_BUS_print_ok},
	{PARSER_MODE_COMMAND, "AT?", STRING_NULL, "List all available commands", _AT_BUS_print_command_list},
	{PARSER_MODE_COMMAND, "AT$V?", STRING_NULL, "Get SW version", _AT_BUS_print_sw_version},
	{PARSER_MODE_COMMAND, "AT$ERROR?", STRING_NULL, "Read error stack", _AT_BUS_print_error_stack},
	{PARSER_MODE_COMMAND, "AT$RST", STRING_NULL, "Reset MCU", PWR_software_reset},
	{PARSER_MODE_COMMAND, "AT$ADC?", STRING_NULL, "Get ADC measurements", _AT_BUS_adc_callback},
#endif
};

static AT_BUS_context_t at_bus_ctx;

/*** AT local functions ***/

/* GENERIC MACRO TO ADD A CHARACTER TO THE REPLY BUFFER.
 * @param character:	Character to add.
 * @return:				None.
 */
#define _AT_BUS_reply_add_char(character) { \
	at_bus_ctx.reply[at_bus_ctx.reply_size] = character; \
	at_bus_ctx.reply_size = (at_bus_ctx.reply_size + 1) % AT_BUS_REPLY_BUFFER_SIZE; \
}

/* APPEND A STRING TO THE REPONSE BUFFER.
 * @param tx_string:	String to add.
 * @return:				None.
 */
static void _AT_BUS_reply_add_string(char_t* tx_string) {
	// Fill reply buffer with new bytes.
	while (*tx_string) {
		_AT_BUS_reply_add_char(*(tx_string++));
	}
}

/* APPEND A VALUE TO THE REPLY BUFFER.
 * @param tx_value:		Value to add.
 * @param format:       Printing format.
 * @param print_prefix: Print base prefix is non zero.
 * @return:				None.
 */
static void _AT_BUS_reply_add_value(int32_t tx_value, STRING_format_t format, uint8_t print_prefix) {
	// Local variables.
	STRING_status_t string_status = STRING_SUCCESS;
	char_t str_value[AT_BUS_STRING_VALUE_BUFFER_SIZE];
	uint8_t idx = 0;
	// Reset string.
	for (idx=0 ; idx<AT_BUS_STRING_VALUE_BUFFER_SIZE ; idx++) str_value[idx] = STRING_CHAR_NULL;
	// Convert value to string.
	string_status = STRING_value_to_string(tx_value, format, print_prefix, str_value);
	STRING_error_check();
	// Add string.
	_AT_BUS_reply_add_string(str_value);
}

/* APPEND A REGISTER VALUE TO THE REPLY BUFFER.
 * @param reg_value:	Register value to add.
 * @return:				None.
 */
static void _AT_BUS_reply_add_register(uint32_t reg_value) {
	// Local variables.
	STRING_status_t string_status = STRING_SUCCESS;
	char_t str_value[AT_BUS_STRING_VALUE_BUFFER_SIZE] = {STRING_CHAR_NULL};
	// Convert register to string.
	string_status = DINFOX_register_to_string(reg_value, str_value);
	STRING_error_check();
	// Add string.
	_AT_BUS_reply_add_string(str_value);
}

/* SEND AT REPONSE OVER AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_reply_send(void) {
	// Local variables.
	LBUS_status_t lbus_status = LBUS_SUCCESS;
	// Add ending character.
	_AT_BUS_reply_add_char(AT_BUS_FRAME_END);
#ifdef HIGH_SPEED_LOG
	_AT_BUS_reply_add_char(STRING_CHAR_LF);
#endif
	// Send reply.
	lbus_status = LBUS_send((uint8_t*) at_bus_ctx.reply, at_bus_ctx.reply_size);
	LBUS_error_check();
	// Flush response buffer.
	at_bus_ctx.reply_size = 0;
}

/* PRINT OK THROUGH AT INTERFACE.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_print_ok(void) {
	_AT_BUS_reply_add_string("OK");
	_AT_BUS_reply_send();
}

/* PRINT ERROR THROUGH AT INTERFACE.
 * @param error:	Error to print.
 * @return:			None.
 */
static void _AT_BUS_print_error(ERROR_t error) {
	// Add error to stack.
	ERROR_stack_add(error);
	// Print error.
	_AT_BUS_reply_add_string("ERROR_");
	if (error < 0x0100) {
		_AT_BUS_reply_add_value(0, STRING_FORMAT_HEXADECIMAL, 1);
		_AT_BUS_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 0);
	}
	else {
		_AT_BUS_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
	}
	_AT_BUS_reply_send();
}

#ifdef ATM
/* PRINT ALL SUPPORTED AT COMMANDS.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_print_command_list(void) {
	// Local variables.
	uint32_t idx = 0;
	// Commands loop.
	for (idx=0 ; idx<(sizeof(AT_BUS_COMMAND_LIST) / sizeof(AT_BUS_command_t)) ; idx++) {
		// Print syntax.
		_AT_BUS_reply_add_string(AT_BUS_COMMAND_LIST[idx].syntax);
		// Print parameters.
		_AT_BUS_reply_add_string(AT_BUS_COMMAND_LIST[idx].parameters);
		_AT_BUS_reply_send();
		// Print description.
		_AT_BUS_reply_add_string(AT_BUS_REPLY_TAB);
		_AT_BUS_reply_add_string(AT_BUS_COMMAND_LIST[idx].description);
		_AT_BUS_reply_send();
	}
	_AT_BUS_print_ok();
}
#endif

#ifdef ATM
/* PRINT SW VERSION.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_print_sw_version(void) {
	// Local variables.
	NODE_status_t node_status = NODE_SUCCESS;
	uint32_t generic_u32 = 0;
	_AT_BUS_reply_add_string("SW");
	// Major version.
	node_status = NODE_read_field(NODE_REQUEST_SOURCE_EXTERNAL, COMMON_REG_ADDR_SW_VERSION_0, COMMON_REG_SW_VERSION_0_MASK_MAJOR, &generic_u32);
	NODE_error_check_print();
	_AT_BUS_reply_add_value((int32_t) generic_u32, STRING_FORMAT_DECIMAL, 0);
	// Minor version.
	node_status = NODE_read_field(NODE_REQUEST_SOURCE_EXTERNAL, COMMON_REG_ADDR_SW_VERSION_0, COMMON_REG_SW_VERSION_0_MASK_MINOR, &generic_u32);
	NODE_error_check_print();
	_AT_BUS_reply_add_string(".");
	_AT_BUS_reply_add_value((int32_t) generic_u32, STRING_FORMAT_DECIMAL, 0);
	// Commit index.
	node_status = NODE_read_field(NODE_REQUEST_SOURCE_EXTERNAL, COMMON_REG_ADDR_SW_VERSION_0, COMMON_REG_SW_VERSION_0_MASK_COMMIT_INDEX, &generic_u32);
	NODE_error_check_print();
	_AT_BUS_reply_add_string(".");
	_AT_BUS_reply_add_value((int32_t) generic_u32, STRING_FORMAT_DECIMAL, 0);
	// Dirty flag.
	node_status = NODE_read_field(NODE_REQUEST_SOURCE_EXTERNAL, COMMON_REG_ADDR_SW_VERSION_0, COMMON_REG_SW_VERSION_0_MASK_DTYF, &generic_u32);
	NODE_error_check_print();
	if (generic_u32 != 0) {
		_AT_BUS_reply_add_string(".d");
	}
	// Commit ID.
	node_status = NODE_read_field(NODE_REQUEST_SOURCE_EXTERNAL, COMMON_REG_ADDR_SW_VERSION_1, COMMON_REG_SW_VERSION_1_MASK_COMMIT_ID, &generic_u32);
	NODE_error_check_print();
	_AT_BUS_reply_add_string(" (");
	_AT_BUS_reply_add_value((int32_t) generic_u32, STRING_FORMAT_HEXADECIMAL, 1);
	_AT_BUS_reply_add_string(")");
	_AT_BUS_reply_send();
	_AT_BUS_print_ok();
errors:
	return;
}
#endif

#ifdef ATM
/* PRINT ERROR STACK.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_print_error_stack(void) {
	// Local variables.
	NODE_status_t node_status = NODE_SUCCESS;
	uint32_t generic_u32 = 0;
	ERROR_t error = SUCCESS;
	// Read stack.
	if (ERROR_stack_is_empty() != 0) {
		_AT_BUS_reply_add_string("Error stack empty");
	}
	else {
		// Unstack all errors.
		_AT_BUS_reply_add_string("[ ");
		do {
			// Read error stack.
			node_status = NODE_read_field(NODE_REQUEST_SOURCE_EXTERNAL, COMMON_REG_ADDR_ERROR_STACK, COMMON_REG_ERROR_STACK_MASK_ERROR, &generic_u32);
			NODE_error_check_print();
			error = (ERROR_t) generic_u32;
			// Check value.
			if (error != SUCCESS) {
				_AT_BUS_reply_add_value((int32_t) error, STRING_FORMAT_HEXADECIMAL, 1);
				_AT_BUS_reply_add_string(" ");
			}
		}
		while (error != SUCCESS);
		_AT_BUS_reply_add_string("]");
	}
	_AT_BUS_reply_send();
	_AT_BUS_print_ok();
errors:
	return;
}
#endif

/* RESET AT PARSER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_reset_parser(void) {
	// Flush buffers.
	at_bus_ctx.command_size = 0;
	at_bus_ctx.reply_size = 0;
	// Reset flag.
	at_bus_ctx.line_end_flag = 0;
	// Reset parser.
	at_bus_ctx.parser.buffer = (char_t*) at_bus_ctx.command;
	at_bus_ctx.parser.buffer_size = 0;
	at_bus_ctx.parser.separator_idx = 0;
	at_bus_ctx.parser.start_idx = 0;
}

/* PARSE THE CURRENT AT COMMAND BUFFER.
 * @param:	None.
 * @return:	None.
 */
static void _AT_BUS_decode(void) {
	// Local variables.
	uint8_t idx = 0;
	uint8_t decode_success = 0;
	// Update parser length.
	at_bus_ctx.parser.buffer_size = at_bus_ctx.command_size;
	// Loop on available commands.
	for (idx=0 ; idx<(sizeof(AT_BUS_COMMAND_LIST) / sizeof(AT_BUS_command_t)) ; idx++) {
		// Check type.
		if (PARSER_compare(&at_bus_ctx.parser, AT_BUS_COMMAND_LIST[idx].mode, AT_BUS_COMMAND_LIST[idx].syntax) == PARSER_SUCCESS) {
			// Execute callback and exit.
			AT_BUS_COMMAND_LIST[idx].callback();
			decode_success = 1;
			break;
		}
	}
	if (decode_success == 0) {
		_AT_BUS_print_error(ERROR_BASE_PARSER + PARSER_ERROR_UNKNOWN_COMMAND); // Unknown command.
		goto errors;
	}
errors:
	_AT_BUS_reset_parser();
	return;
}

/*** AT functions ***/

/* INIT AT MANAGER.
 * @param self_address:	Self bus address.
 * @return:				None.
 */
void AT_BUS_init(NODE_address_t self_address) {
	// Local variables.
	LBUS_status_t lbus_status = LBUS_SUCCESS;
	// Init context.
	_AT_BUS_reset_parser();
	// Init LBUS layer.
	lbus_status = LBUS_init(self_address);
	LBUS_error_check();
	// Enable LPUART.
	LPUART1_enable_rx();
}

/* MAIN TASK OF AT COMMAND MANAGER.
 * @param:	None.
 * @return:	None.
 */
void AT_BUS_task(void) {
	// Trigger decoding function if line end found.
	if (at_bus_ctx.line_end_flag != 0) {
		// Decode and execute command.
		LPUART1_disable_rx();
		_AT_BUS_decode();
		LPUART1_enable_rx();
	}
}

/* FILL AT COMMAND BUFFER WITH A NEW BYTE (CALLED BY LPUART INTERRUPT).
 * @param rx_byte:	Incoming byte.
 * @return:			None.
 */
void AT_BUS_fill_rx_buffer(uint8_t rx_byte) {
	// Append byte if line end flag is not allready set.
	if (at_bus_ctx.line_end_flag == 0) {
		// Check ending characters.
		if (rx_byte == AT_BUS_FRAME_END) {
			at_bus_ctx.command[at_bus_ctx.command_size] = STRING_CHAR_NULL;
			at_bus_ctx.line_end_flag = 1;
		}
		else {
			// Store new byte.
			at_bus_ctx.command[at_bus_ctx.command_size] = rx_byte;
			// Manage index.
			at_bus_ctx.command_size = (at_bus_ctx.command_size + 1) % AT_BUS_COMMAND_BUFFER_SIZE;
		}
	}
}

#ifdef HIGH_SPEED_LOG
/* PRINT MEASUREMENTS RESULT ON RS485 INTERFACE.
 * @param:	None.
 * @return:	None.
 */
void AT_BUS_high_speed_log(void) {
	// Local variables.
	MEASURE_channel_result_t channel_result;
	// Get measurements.
	MEASURE_get_ac_channel_data(0, &channel_result);
	// Print data.
	_AT_BUS_reply_add_string("Pact = ");
	_AT_BUS_reply_add_value(channel_result.active_power_mw.rolling_mean, STRING_FORMAT_DECIMAL, 0);
	_AT_BUS_reply_add_string(" mW");
	_AT_BUS_reply_send();
	_AT_BUS_reply_add_string("Urms = ");
	_AT_BUS_reply_add_value(channel_result.rms_voltage_mv.rolling_mean, STRING_FORMAT_DECIMAL, 0);
	_AT_BUS_reply_add_string(" mV");
	_AT_BUS_reply_send();
	_AT_BUS_reply_add_string("Irms = ");
	_AT_BUS_reply_add_value(channel_result.rms_current_ma.rolling_mean, STRING_FORMAT_DECIMAL, 0);
	_AT_BUS_reply_add_string(" mA");
	_AT_BUS_reply_send();
	_AT_BUS_reply_add_string("Papp = ");
	_AT_BUS_reply_add_value(channel_result.apparent_power_mva.rolling_mean, STRING_FORMAT_DECIMAL, 0);
	_AT_BUS_reply_add_string(" mVA");
	_AT_BUS_reply_send();
	_AT_BUS_reply_add_string("PF = ");
	_AT_BUS_reply_add_value(channel_result.power_factor.rolling_mean, STRING_FORMAT_DECIMAL, 0);
	_AT_BUS_reply_send();
	_AT_BUS_reply_add_string(" ");
	_AT_BUS_reply_send();
}
#endif
