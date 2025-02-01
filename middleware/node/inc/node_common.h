/*
 * node_common.h
 *
 *  Created on: 10 jun. 2023
 *      Author: Ludo
 */

#ifndef __NODE_COMMON_H__
#define __NODE_COMMON_H__

#include "types.h"

/*** NODES common structures ***/

/*!******************************************************************
 * \type NODE_address_t
 * \brief Node address type.
 *******************************************************************/
typedef uint8_t	NODE_address_t;

/*!******************************************************************
 * \enum NODE_protocol_t
 * \brief Node protocols list.
 *******************************************************************/
typedef enum {
	NODE_PROTOCOL_NONE = 0,
	NODE_PROTOCOL_AT_BUS,
	NODE_PROTOCOL_R4S8CR,
	NODE_PROTOCOL_LAST
} NODE_protocol_t;

/*!******************************************************************
 * \enum NODE_reply_type_t
 * \brief Node reply type.
 *******************************************************************/
typedef enum {
	NODE_REPLY_TYPE_NONE = 0,
	NODE_REPLY_TYPE_OK,
	NODE_REPLY_TYPE_VALUE,
	NODE_REPLY_TYPE_LAST
} NODE_reply_type_t;

/*!******************************************************************
 * \enum NODE_reply_type_t
 * \brief Node reply type.
 *******************************************************************/
typedef enum {
	NODE_ACCESS_TYPE_READ = 0b0,
	NODE_ACCESS_TYPE_WRITE = 0b1
} NODE_access_type_t;

/*!******************************************************************
 * \struct NODE_reply_parameters_t
 * \brief Node reply parameters.
 *******************************************************************/
typedef struct {
	NODE_reply_type_t type;
	uint32_t timeout_ms;
} NODE_reply_parameters_t;

/*!******************************************************************
 * \struct NODE_command_parameters_t
 * \brief Node command parameters.
 *******************************************************************/
typedef struct {
	NODE_address_t node_addr;
	char_t* command;
} NODE_command_parameters_t;

/*!******************************************************************
 * \struct NODE_access_parameters_t
 * \brief Node access parameters.
 *******************************************************************/
typedef struct {
	NODE_address_t node_addr;
	uint8_t reg_addr;
	NODE_reply_parameters_t reply_params;
} NODE_access_parameters_t;

/*!******************************************************************
 * \union NODE_access_status_t
 * \brief Node access status.
 *******************************************************************/
typedef union {
	struct {
		NODE_access_type_t type : 1;
		unsigned flags : 7;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	struct {
		NODE_access_type_t type_bis : 1;
		unsigned unused : 3;
		unsigned sequence_timeout : 1;
		unsigned reply_timeout : 1;
		unsigned parser_error : 1;
		unsigned error_received : 1;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
	uint8_t all;
} NODE_access_status_t;

#endif /* __NODE_COMMON_H__ */
