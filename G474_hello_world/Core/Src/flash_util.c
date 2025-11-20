/**
 * @file flash_util.c
 * @brief Flash memory utility implementation for STM32G474
 */

#include "flash_util.h"
#include <string.h>

/**
 * @brief Validate flash address
 */
static int _is_valid_address(uint32_t addr) {
    return (addr >= FLASH_BANK1_START && addr < (FLASH_BANK2_START + FLASH_TOTAL_SIZE));
}

/**
 * @brief Validate page-aligned address
 */
static int _is_page_aligned(uint32_t addr) {
    return (addr % FLASH_PAGE_SIZE) == 0;
}

/**
 * Erase a range of flash pages
 */
FlashStatus_t Flash_ErasePage(uint32_t start_addr, uint32_t num_pages) {
    if (!_is_valid_address(start_addr)) {
        return FLASH_ERROR_ADDRESS;
    }
    if (!_is_page_aligned(start_addr)) {
        return FLASH_ERROR_ADDRESS;
    }
    
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError = 0;
    
    /* Determine which bank and starting page */
    uint32_t page_start;
    if (start_addr >= FLASH_BANK2_START) {
        EraseInit.Banks = FLASH_BANK_2;
        page_start = (start_addr - FLASH_BANK2_START) / FLASH_PAGE_SIZE;
    } else {
        EraseInit.Banks = FLASH_BANK_1;
        page_start = (start_addr - FLASH_BANK1_START) / FLASH_PAGE_SIZE;
    }
    
    EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInit.Page = page_start;
    EraseInit.NbPages = num_pages;
    
    HAL_FLASH_Unlock();
    
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
    
    HAL_FLASH_Lock();
    
    if (status != HAL_OK) {
        return FLASH_ERROR_ERASE;
    }
    
    return FLASH_OK;
}

/**
 * Erase entire flash bank
 */
FlashStatus_t Flash_EraseBank(uint32_t bank_num) {
    if (bank_num != FLASH_BANK_1 && bank_num != FLASH_BANK_2) {
        return FLASH_ERROR_ADDRESS;
    }
    
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError = 0;
    /* Use mass erase for whole bank */
    EraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;
    EraseInit.Banks = bank_num;
    EraseInit.Page = 0;
    EraseInit.NbPages = 0;  /* Not used for mass erase */
    
    HAL_FLASH_Unlock();
    
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
    
    HAL_FLASH_Lock();
    
    if (status != HAL_OK) {
        return FLASH_ERROR_ERASE;
    }
    
    return FLASH_OK;
}

/**
 * Write 64-bit double-word to flash
 */
FlashStatus_t Flash_WriteWord64(uint32_t address, uint64_t data) {
    if (!_is_valid_address(address)) {
        return FLASH_ERROR_ADDRESS;
    }
    if ((address % 8) != 0) {
        return FLASH_ERROR_ADDRESS;  /* Must be 8-byte aligned */
    }
    
    HAL_FLASH_Unlock();
    
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    
    HAL_FLASH_Lock();
    
    if (status != HAL_OK) {
        return FLASH_ERROR_WRITE;
    }
    
    return FLASH_OK;
}

/**
 * Write buffer to flash (multiple 64-bit words)
 */
FlashStatus_t Flash_WriteBuffer(uint32_t address, const uint8_t *data, uint32_t size) {
    if (!_is_valid_address(address)) {
        return FLASH_ERROR_ADDRESS;
    }
    if ((address % 8) != 0) {
        return FLASH_ERROR_ADDRESS;  /* Must be 8-byte aligned */
    }
    if ((size % 8) != 0) {
        return FLASH_ERROR_INVALID_SIZE;  /* Size must be multiple of 8 */
    }
    if (data == NULL) {
        return FLASH_ERROR_ADDRESS;
    }
    
    HAL_FLASH_Unlock();
    
    uint32_t num_words = size / 8;
    for (uint32_t i = 0; i < num_words; i++) {
        uint64_t word = 0;
        memcpy(&word, &data[i * 8], 8);
        
        HAL_StatusTypeDef status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_DOUBLEWORD,
            address + (i * 8),
            word
        );
        
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return FLASH_ERROR_WRITE;
        }
    }
    
    HAL_FLASH_Lock();
    
    return FLASH_OK;
}

/**
 * Read buffer from flash
 */
FlashStatus_t Flash_ReadBuffer(uint32_t address, uint8_t *data, uint32_t size) {
    if (!_is_valid_address(address)) {
        return FLASH_ERROR_ADDRESS;
    }
    if (data == NULL) {
        return FLASH_ERROR_ADDRESS;
    }
    
    /* Simple memcpy from flash address space */
    memcpy(data, (const void *)address, size);
    
    return FLASH_OK;
}

/**
 * Get page address from bank and page index
 */
uint32_t Flash_GetPageAddress(uint32_t bank_num, uint32_t page_idx) {
    /* STM32G474 has 64 pages per bank (128 pages total) */
    uint32_t pages_per_bank = FLASH_NUM_PAGES / 2;
    
    if (page_idx >= pages_per_bank) {
        return 0;  /* Invalid page index */
    }
    
    if (bank_num == FLASH_BANK_1) {
        return FLASH_BANK1_START + (page_idx * FLASH_PAGE_SIZE);
    } else if (bank_num == FLASH_BANK_2) {
        return FLASH_BANK2_START + (page_idx * FLASH_PAGE_SIZE);
    }
    
    return 0;  /* Invalid bank */
}

/**
 * Verify flash write
 */
FlashStatus_t Flash_Verify(uint32_t address, const uint8_t *data, uint32_t size) {
    if (!_is_valid_address(address)) {
        return FLASH_ERROR_ADDRESS;
    }
    if (data == NULL) {
        return FLASH_ERROR_ADDRESS;
    }
    
    uint8_t *flash_ptr = (uint8_t *)address;
    if (memcmp(flash_ptr, data, size) != 0) {
        return FLASH_ERROR_WRITE;  /* Data mismatch */
    }
    
    return FLASH_OK;
}
