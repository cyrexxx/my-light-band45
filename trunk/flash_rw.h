#ifndef FLASH_RW_H__
#define FLASH_RW_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
static void flash_page_erase(uint32_t *page_address);

/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
static void flash_word_write(uint32_t *address, uint32_t value);



#endif // FLASH_RW_H__

/** @} */
