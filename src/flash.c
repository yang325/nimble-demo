/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"
#include "flash.h"

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void flash_init(void)
{
  /* Unlock the Program memory */
  HAL_FLASH_Unlock();
  /* Clear all FLASH flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  /* Unlock the Program memory */
  HAL_FLASH_Lock();
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  start: start of user flash area
  * @retval FLASHIF_OK : user flash area successfully erased
  *         FLASHIF_ERASEKO : error occurred
  */
int flash_page_erase(uint32_t address)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t PageError = 0;
  FLASH_EraseInitTypeDef EraseInit = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks = FLASH_BANK_1,
    .NbPages = 1,
  };

  assert(ADDR_FLASH_PAGE_254 == address || ADDR_FLASH_PAGE_255 == address);

  /* Unlock the Flash to enable the flash control register access */ 
  HAL_FLASH_Unlock();
  EraseInit.PageAddress = address;
  status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  return (HAL_OK == status) ? FLASH_OK : FLASH_ERROR;
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  destination: start address for target location
  * @param  p_source: pointer on buffer with data to write
  * @param  length: length of data buffer (unit is 32-bit word)
  * @retval uint32_t 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
int flash_write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
  int idx, ret = FLASH_OK;
  HAL_StatusTypeDef status = HAL_OK;

  assert(destination >= ADDR_FLASH_PAGE_254);
  assert(0 == (destination & (uint32_t)0x03));

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();
  for (idx = 0; idx < length && destination < FLASH_BANK1_END; ++idx) {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destination, *p_source);
    if (HAL_OK != status) {
      console_printf("Error occurred while writing data in Flash memory\n");
      ret = FLASH_ERROR;
      break;
    }
    /* Check the written value */
    if (*p_source != *(uint32_t *)destination) {
      console_printf("Flash content doesn't match SRAM content\n");
      ret = FLASH_ERROR;
      break;
    }
    /* Increment FLASH destination address */
    ++p_source;
    destination += sizeof(uint32_t);
  }
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  return ret;
}

/**
  * @brief  Returns the write protection status of application flash area.
  * @param  None
  * @retval If a sector in application area is write-protected returned value is a combinaison
  *         of the possible values : FLASHIF_PROTECTION_WRPENABLED, FLASHIF_PROTECTION_PCROPENABLED, ...
  *         If no sector is write-protected FLASHIF_PROTECTION_NONE is returned.
  */
bool flash_write_protection_status(void)
{
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();
  /* Check if there are write protected sectors inside the user flash area */
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  if (OB_WRP_PAGES62TO255 & OptionsBytesStruct.WRPPage) {
    return true;
  } else {
    return false;
  }
}

/**
  * @brief  Configure the write protection status of user flash area.
  * @param  protectionstate : FLASHIF_WRP_DISABLE or FLASHIF_WRP_ENABLE the protection
  * @retval uint32_t FLASHIF_OK if change is applied.
  */
int flash_write_protection_config(bool enable)
{
  HAL_StatusTypeDef result = HAL_OK;
  FLASH_OBProgramInitTypeDef config_new, config_old;

  /* Get pages write protection status */
  HAL_FLASHEx_OBGetConfig(&config_old);
  /* The parameter says whether we turn the protection on or off */
  config_new.WRPState = enable ? OB_WRPSTATE_ENABLE : OB_WRPSTATE_DISABLE;
  /* We want to modify only the Write protection */
  config_new.OptionType = OPTIONBYTE_WRP;
  /* No read protection, keep BOR and reset settings */
  config_new.RDPLevel = OB_RDP_LEVEL_0;
  config_new.USERConfig = config_old.USERConfig;
  /* Get pages already write protected */
  config_new.WRPPage = config_old.WRPPage | OB_WRP_PAGES62TO255;

  /* Unlock the Flash to enable the flash control register access */ 
  HAL_FLASH_Unlock();
  /* Unlock the Options Bytes */
  HAL_FLASH_OB_Unlock();

  /* Erase all the option Bytes */
  result = HAL_FLASHEx_OBErase();
  if (HAL_OK == result) {
    result = HAL_FLASHEx_OBProgram(&config_new);
    if (HAL_OK != result) {
      console_printf("Fail to program the option bytes\n");
    }
  } else {
    console_printf("Fail to erase all the option bytes\n");
  }

  HAL_FLASH_OB_Lock();
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  return (HAL_OK == result) ? FLASH_OK : FLASH_ERROR;
}
