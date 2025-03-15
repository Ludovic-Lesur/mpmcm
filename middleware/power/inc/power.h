/*
 * power.h
 *
 *  Created on: 22 jul. 2023
 *      Author: Ludo
 */

#ifndef __POWER_H__
#define __POWER_H__

#include "error.h"
#include "lptim.h"
#include "types.h"

/*** POWER structures ***/

/*!******************************************************************
 * \enum POWER_status_t
 * \brief POWER driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    POWER_SUCCESS,
    POWER_ERROR_REQUESTER_ID,
    POWER_ERROR_DOMAIN,
    // Low level drivers errors.
    POWER_ERROR_DRIVER_ANALOG,
    POWER_ERROR_DRIVER_LPTIM,
    // Last base value.
    POWER_ERROR_BASE_LAST = ERROR_BASE_STEP
} POWER_status_t;

/*!******************************************************************
 * \enum POWER_requester_id_t
 * \brief Calling driver identifier.
 *******************************************************************/
typedef enum {
    POWER_REQUESTER_ID_NODE = 0,
    POWER_REQUESTER_ID_COMMON,
    POWER_REQUESTER_ID_MPMCM,
    POWER_REQUESTER_ID_MEASURE,
    POWER_REQUESTER_ID_TIC,
    POWER_REQUESTER_ID_LAST
} POWER_requester_id_t;

/*!******************************************************************
 * \enum POWER_domain_t
 * \brief Board external power domains list.
 *******************************************************************/
typedef enum {
    POWER_DOMAIN_MCU_TCXO = 0,
    POWER_DOMAIN_ANALOG,
    POWER_DOMAIN_TIC,
    POWER_DOMAIN_LAST
} POWER_domain_t;

/*** POWER functions ***/

/*!******************************************************************
 * \fn void POWER_init(void)
 * \brief Init power control module.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void POWER_init(void);

/*!******************************************************************
 * \fn void POWER_enable(POWER_requester_id_t requester_id, POWER_domain_t domain, LPTIM_delay_mode_t delay_mode)
 * \brief Turn power domain on.
 * \param[in]   requester_id: Identifier of the calling driver.
 * \param[in]   domain: Power domain to enable.
 * \param[in]   delay_mode: Power on delay waiting mode.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void POWER_enable(POWER_requester_id_t requester_id, POWER_domain_t domain, LPTIM_delay_mode_t delay_mode);

/*!******************************************************************
 * \fn void POWER_disable(POWER_requester_id_t requester_id, POWER_domain_t domain)
 * \brief Turn power domain off.
 * \param[in]   requester_id: Identifier of the calling driver.
 * \param[in]   domain: Power domain to disable.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void POWER_disable(POWER_requester_id_t requester_id, POWER_domain_t domain);

/*!******************************************************************
 * \fn uint8_t POWER_get_state(POWER_domain_t domain)
 * \brief Return the current state of a power domain.
 * \param[in]   domain: Power domain to check.
 * \param[out]  none
 * \retval      Power domain state.
 *******************************************************************/
uint8_t POWER_get_state(POWER_domain_t domain);

#endif /* __POWER_H__ */
