/*
 * at.c
 *
 *  Created on: 9 nov. 2018
 *      Author: Ludo
 */

#include "cli.h"

#include "at.h"
#include "error.h"
#include "error_base.h"
#include "node.h"
#include "una.h"
#include "una_at.h"
#include "types.h"

/*** CLI local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint8_t una_at_process_flag;
} CLI_context_t;

/*** CLI local global variables ***/

static CLI_context_t cli_ctx;

/*** CLI local functions ***/

/*******************************************************************/
#define _CLI_check_driver_status(driver_status, driver_success, driver_error_base) { \
    /* Check value */ \
    if (driver_status != driver_success) { \
        /* Stack error */ \
        ERROR_stack_add((ERROR_code_t) (driver_error_base + driver_status)); \
        /* Exit with execution error */ \
        status = AT_ERROR_COMMAND_EXECUTION; \
        goto errors; \
    } \
}

/*******************************************************************/
static void _CLI_una_at_process_callback(void) {
    // Set local flag.
    cli_ctx.una_at_process_flag = 1;
}

/*******************************************************************/
static AT_status_t _CLI_write_register_callback(uint8_t reg_addr, uint32_t reg_value, uint32_t reg_mask) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    // Write register.
    node_status = NODE_write_register(NODE_REQUEST_SOURCE_EXTERNAL, reg_addr, reg_value, reg_mask);
    _CLI_check_driver_status(node_status, NODE_SUCCESS, ERROR_BASE_NODE);
errors:
    return status;
}

/*******************************************************************/
static AT_status_t _CLI_read_register_callback(uint8_t reg_addr, uint32_t* reg_value) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    NODE_status_t node_status = NODE_SUCCESS;
    // Write register.
    node_status = NODE_read_register(NODE_REQUEST_SOURCE_EXTERNAL, reg_addr, reg_value);
    _CLI_check_driver_status(node_status, NODE_SUCCESS, ERROR_BASE_NODE);
errors:
    return status;
}

/*** CLI functions ***/

/*******************************************************************/
CLI_status_t CLI_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    UNA_AT_configuration_t una_at_config;
    // Init context.
    cli_ctx.una_at_process_flag = 0;
    // Init AT driver.
    una_at_config.process_callback = &_CLI_una_at_process_callback;
    una_at_config.write_register_callback = &_CLI_write_register_callback;
    una_at_config.read_register_callback = &_CLI_read_register_callback;
    una_at_status = UNA_AT_init(&una_at_config);
    UNA_AT_exit_error(CLI_ERROR_BASE_UNA_AT);
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_de_init(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    // Release AT driver.
    una_at_status = UNA_AT_de_init();
    UNA_AT_exit_error(CLI_ERROR_BASE_UNA_AT);
errors:
    return status;
}

/*******************************************************************/
CLI_status_t CLI_process(void) {
    // Local variables.
    CLI_status_t status = CLI_SUCCESS;
    UNA_AT_status_t una_at_status = UNA_AT_SUCCESS;
    // Check process flag.
    if (cli_ctx.una_at_process_flag != 0) {
        // Clear flag.
        cli_ctx.una_at_process_flag = 0;
        // Process AT driver.
        una_at_status = UNA_AT_process();
        UNA_AT_exit_error(CLI_ERROR_BASE_UNA_AT);
    }
errors:
    return status;
}
