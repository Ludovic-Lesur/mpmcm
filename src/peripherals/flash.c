/*
 * flash.c
 *
 *  Created on: 03 sep. 2023
 *      Author: Ludo
 */

#include "flash.h"

#include "flash_reg.h"
#include "types.h"

/*** FLASH local macros ***/

#define FLASH_WAIT_STATES_MAX			15
#define FLASH_TIMEOUT_COUNT				100000

#define FLASH_NUMBER_OF_PAGES			64 // For 128kB device.
#define FLASH_NUMBER_OF_ROWS_PER_PAGE	8
#define FLASH_ROW_SIZE_BYTES			256
#define FLASH_PAGE_SIZE_BYTES			(FLASH_NUMBER_OF_ROWS_PER_PAGE * FLASH_ROW_SIZE_BYTES)
#define FLASH_PAGE_SIZE_WORDS			(FLASH_PAGE_SIZE_BYTES >> 4) // Each address contains a 64-bits word.

#define FLASH_NVM_PAGE_INDEX			63
#define FLASH_NVM_PAGE_ADDRESS			0x0801F800

/*** FLASH local global variables ***/

static uint32_t flash_nvm_page[FLASH_PAGE_SIZE_WORDS];

/*** FLASH local functions ***/

/*******************************************************************/
static FLASH_status_t _FLASH_check_busy(FLASH_status_t timeout_error_code) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	uint32_t loop_count = 0;
	// Check no write/erase operation is running.
	while (((FLASH -> SR) & (0b1 << 16)) != 0) {
		// Wait till BSY='1' or timeout.
		loop_count++;
		if (loop_count > FLASH_TIMEOUT_COUNT) {
			status = timeout_error_code;
			goto errors;
		}
	}
errors:
	// Clear all flags.
	FLASH -> SR |= 0x0000C3FB;
	return status;
}

/*******************************************************************/
static FLASH_status_t __attribute__((optimize("-O0"))) _FLASH_unlock(void) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	// Check memory is ready.
	status = _FLASH_check_busy(FLASH_ERROR_UNLOCK);
	if (status != FLASH_SUCCESS) goto errors;
	// Check the memory is not already unlocked.
	if (((FLASH -> CR) & (0b1 << 31)) != 0) {
		// Perform unlock sequence.
		FLASH -> KEYR = 0x45670123;
		FLASH -> KEYR = 0xCDEF89AB;
	}
errors:
	return status;
}

/*******************************************************************/
static FLASH_status_t __attribute__((optimize("-O0"))) _FLASH_lock(void) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	// Check memory is ready.
	status = _FLASH_check_busy(FLASH_ERROR_LOCK);
	if (status != FLASH_SUCCESS) goto errors;
	// Lock sequence.
	FLASH -> CR |= (0b1 << 31);
errors:
	return status;
}

/*******************************************************************/
static FLASH_status_t __attribute__((optimize("-O0"))) _FLASH_read_nvm_page(void) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	uint32_t word_idx = 0;
	// Word loop.
	for (word_idx=0 ; word_idx<FLASH_PAGE_SIZE_WORDS ; word_idx++) {
		status = FLASH_read_word(word_idx, &(flash_nvm_page[word_idx]));
		if (status != FLASH_SUCCESS) goto errors;
	}
errors:
	return status;
}

/*******************************************************************/
static FLASH_status_t __attribute__((optimize("-O0"))) _FLASH_erase_nvm_page(void) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	// Select page.
	FLASH -> CR &= ~(0b1111111 << 3);
	FLASH -> CR |= ((FLASH_NVM_PAGE_INDEX & 0x7F) << 3);
	// Set page erase bit.
	FLASH -> CR |= (0b1 << 1);
	// Start operation.
	FLASH -> CR |= (0b1 << 16);
	// Wait the end of operation.
	status = _FLASH_check_busy(FLASH_ERROR_ERASE);
	if (status != FLASH_SUCCESS) goto errors;
errors:
	// Reset operation selection.
	FLASH -> CR &= ~(0b111 << 0);
	return status;
}

/*******************************************************************/
FLASH_status_t __attribute__((optimize("-O0"))) FLASH_write_nvm_word(FLASH_address_t address, uint32_t data) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	uint32_t absolute_address = (FLASH_NVM_PAGE_ADDRESS + (address << 4));
	// Write first word.
	*((uint32_t*) (absolute_address + 0)) = data;
	__asm volatile ("isb");
	*((uint32_t*) (absolute_address + 4)) = 0;
	// Wait the end of operation.
	status = _FLASH_check_busy(FLASH_ERROR_WRITE);
	return status;
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _FLASH_flush_caches(void) {
	// Flush instruction cache.
	FLASH -> ACR &= ~(0b1 << 9);
	FLASH -> ACR |= (0b1 << 11);
	FLASH -> ACR &= ~(0b1 << 11);
	FLASH -> ACR |= (0b1 << 9);
	// Flush data cache.
	FLASH -> ACR &= ~(0b1 << 10);
	FLASH -> ACR |= (0b1 << 12);
	FLASH -> ACR &= ~(0b1 << 12);
	FLASH -> ACR |= (0b1 << 10);
}

/*** FLASH functions ***/

/*******************************************************************/
FLASH_status_t FLASH_set_latency(uint8_t wait_states) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameter.
	if (wait_states > FLASH_WAIT_STATES_MAX) {
		status = FLASH_ERROR_LATENCY;
		goto errors;
	}
	// Configure number of wait states.
	FLASH -> ACR &= ~(0b1111 << 0); // Reset bits.
	FLASH -> ACR |= ((wait_states & 0b1111) << 0); // Set latency.
	// Wait until configuration is done.
	while (((FLASH -> ACR) & (0b1111 << 0)) != ((wait_states & 0b1111) << 0)) {
		loop_count++;
		if (loop_count > FLASH_TIMEOUT_COUNT) {
			status = FLASH_ERROR_TIMEOUT;
			goto errors;
		}
	}
errors:
	return status;
}

/*******************************************************************/
FLASH_status_t __attribute__((optimize("-O0"))) FLASH_read_word(FLASH_address_t address, uint32_t* data) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	uint32_t absolute_address = (FLASH_NVM_PAGE_ADDRESS + (address << 4));
	// Check parameters.
	if (address >= FLASH_PAGE_SIZE_WORDS) {
		status = FLASH_ERROR_ADDRESS;
		goto errors;
	}
	if (data == NULL) {
		status = FLASH_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check there is no pending operation.
	status = _FLASH_check_busy(FLASH_ERROR_READ);
	if (status != FLASH_SUCCESS) goto errors;
	// Read data.
	(*data) = *((uint32_t*) (absolute_address));
errors:
	return status;
}

/*******************************************************************/
FLASH_status_t __attribute__((optimize("-O0"))) FLASH_write_word(FLASH_address_t address, uint32_t data) {
	// Local variables.
	FLASH_status_t status = FLASH_SUCCESS;
	uint32_t word_idx = 0;
	// Check parameters.
	if (address >= FLASH_PAGE_SIZE_WORDS) {
		status = FLASH_ERROR_ADDRESS;
		goto errors;
	}
	// Load NVM page content in RAM.
	status = _FLASH_read_nvm_page();
	if (status != FLASH_SUCCESS) goto errors;
	// Write data in RAM.
	flash_nvm_page[address] = data;
	// Unlock memory.
	status = _FLASH_unlock();
	if (status != FLASH_SUCCESS) goto errors;
	// Disable data cache.
	FLASH -> ACR &= ~(0b1 << 10);
	// Erase page.
	status = _FLASH_erase_nvm_page();
	if (status != FLASH_SUCCESS) goto errors;
	// Set programming bit.
	FLASH -> CR |= (0b1 << 0);
	// Reprogram page.
	for (word_idx=0 ; word_idx<FLASH_PAGE_SIZE_WORDS ; word_idx++) {
		status = FLASH_write_nvm_word(word_idx, flash_nvm_page[word_idx]);
		if (status != FLASH_SUCCESS) goto errors;
	}
	// Reset programming bit.
	FLASH -> CR &= ~(0b1 << 0);
	// Flush instruction and data caches.
	_FLASH_flush_caches();
	// Lock memory.
	status = _FLASH_lock();
	if (status != FLASH_SUCCESS) goto errors;
	return status;
errors:
	// Reset programming bit.
	FLASH -> CR &= ~(0b1 << 0);
	// Flush instruction and data caches.
	_FLASH_flush_caches();
	// Lock memory.
	_FLASH_lock();
	return status;
}
