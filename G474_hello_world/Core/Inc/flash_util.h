/**
 * @file flash_util.h
 * @brief Flash memory utility functions for STM32G474
 * 
 * Provides high-level functions to:
 * - Erase flash pages or entire bank
 * - Write data to flash (word/double-word)
 * - Read data from flash
 * 
 * Note: Flash operations require unlock/lock. This module handles that internally.
 */

#ifndef INC_FLASH_UTIL_H_
#define INC_FLASH_UTIL_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* Flash memory map (STM32G474) */
#define FLASH_BANK1_START   0x08000000u
#define FLASH_BANK2_START   0x08040000u
/* Use HAL's FLASH_PAGE_SIZE if provided to avoid redefinition warnings */
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE     2048u          /* 2 KB per page */
#endif
#define FLASH_TOTAL_SIZE    (512 * 1024u)  /* 256 KB total */
#define FLASH_NUM_PAGES     256u

/* Return codes */
typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR_ADDRESS,
    FLASH_ERROR_ERASE,
    FLASH_ERROR_WRITE,
    FLASH_ERROR_LOCKED,
    FLASH_ERROR_INVALID_SIZE
} FlashStatus_t;

/**
 * @brief Erase a range of flash pages
 * 
 * @param start_addr    Starting address (must be page-aligned)
 * @param num_pages     Number of consecutive pages to erase
 * 
 * @return FLASH_OK on success, error code otherwise
 * 
 * Note: Erasing sets all bits to 1 (0xFF)
 */
FlashStatus_t Flash_ErasePage(uint32_t start_addr, uint32_t num_pages);

/**
 * @brief Erase entire flash bank (Bank 1 or Bank 2)
 * 
 * @param bank_num      1 or 2 (FLASH_BANK_1 or FLASH_BANK_2)
 * 
 * @return FLASH_OK on success, error code otherwise
 */
FlashStatus_t Flash_EraseBank(uint32_t bank_num);

/**
 * @brief Write 64-bit double-word to flash
 * 
 * @param address   Flash address (must be 8-byte aligned)
 * @param data      64-bit data to write
 * 
 * @return FLASH_OK on success, error code otherwise
 * 
 * Note: Address must be even offset within flash (8-byte boundary)
 */
FlashStatus_t Flash_WriteWord64(uint32_t address, uint64_t data);

/**
 * @brief Write buffer to flash (multiple 64-bit words)
 * 
 * @param address   Starting flash address (must be 8-byte aligned)
 * @param data      Pointer to data buffer
 * @param size      Number of bytes to write (must be multiple of 8)
 * 
 * @return FLASH_OK on success, error code otherwise
 */
FlashStatus_t Flash_WriteBuffer(uint32_t address, const uint8_t *data, uint32_t size);

/**
 * @brief Read buffer from flash
 * 
 * @param address   Starting flash address
 * @param data      Pointer to buffer to store read data
 * @param size      Number of bytes to read
 * 
 * @return FLASH_OK on success, error code otherwise
 * 
 * Note: Reading does not require unlock/lock
 */
FlashStatus_t Flash_ReadBuffer(uint32_t address, uint8_t *data, uint32_t size);

/**
 * @brief Get the page address given a page index and bank
 * 
 * @param bank_num  Bank number (FLASH_BANK_1 or FLASH_BANK_2)
 * @param page_idx  Page index within bank (0 to 63 per bank)
 * 
 * @return Flash address of page, or 0 if invalid
 * 
 * Note: Bank 1 contains your code. Use Bank 2 (pages 0-63) for data storage.
 */
uint32_t Flash_GetPageAddress(uint32_t bank_num, uint32_t page_idx);

/**
 * @brief Convenience: Get page address from Bank 2 (safe for data storage)
 * 
 * @param page_idx  Page index within Bank 2 (0 to 63)
 * 
 * @return Flash address of page in Bank 2, or 0 if invalid
 */
#define Flash_GetPageAddressBank2(page_idx) Flash_GetPageAddress(FLASH_BANK_2, (page_idx))

/**
 * @brief Verify flash write by comparing written data
 * 
 * @param address   Flash address to verify
 * @param data      Expected data
 * @param size      Size in bytes
 * 
 * @return FLASH_OK if data matches, error code otherwise
 */
FlashStatus_t Flash_Verify(uint32_t address, const uint8_t *data, uint32_t size);

#endif /* INC_FLASH_UTIL_H_ */
