/*
 * at.h
 *
 *  Created on: 02 jan. 2025
 *      Author: Ludo
 */

#ifndef __CLI_H__
#define __CLI_H__

#include "types.h"
#include "una_at.h"

/*** CLI structures ***/

/*!******************************************************************
 * \enum CLI_status_t
 * \brief CLI driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    CLI_SUCCESS = 0,
    // Low level drivers errors.
    CLI_ERROR_BASE_UNA_AT = 0x0100,
    // Last base value.
    CLI_ERROR_BASE_LAST = (CLI_ERROR_BASE_UNA_AT + UNA_AT_ERROR_BASE_LAST)
} CLI_status_t;

/*** CLI functions ***/

/*!******************************************************************
 * \fn CLI_status_t CLI_init(void)
 * \brief Init CLI driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_init(void);

/*!******************************************************************
 * \fn CLI_status_t CLI_de_init(void)
 * \brief Release CLI driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_de_init(void);

/*!******************************************************************
 * \fn CLI_status_t CLI_process(void)
 * \brief Process CLI driver
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
CLI_status_t CLI_process(void);

/*******************************************************************/
#define CLI_exit_error(base) { ERROR_check_exit(cli_status, CLI_SUCCESS, base) }

/*******************************************************************/
#define CLI_stack_error(base) { ERROR_check_stack(cli_status, CLI_SUCCESS, base) }

/*******************************************************************/
#define CLI_stack_exit_error(base, code) { ERROR_check_stack_exit(cli_status, CLI_SUCCESS, base, code) }

#endif /* __CLI_H__ */
