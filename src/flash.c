/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>
#include<string.h>

#include "stm32f1xx_hal.h"
#include "flash.h"

/* Private define ------------------------------------------------------------*/

/* Error code */
enum 
{
  FLASH_OK = 0,
  FLASH_ERROR,
};

/* Base address of the Flash sectors */
#define ADDR_FLASH_PAGE_254   ((uint32_t)0x0807F000) /* Base @ of Page 254, 2 Kbytes */
#define ADDR_FLASH_PAGE_255   ((uint32_t)0x0807F800) /* Base @ of Page 255, 2 Kbytes */

#define ADDR_FLASH_START_PAGE    ADDR_FLASH_PAGE_254

/* Private function prototypes -----------------------------------------------*/

static int block_device_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size);
static int block_device_prog(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size);
static int block_device_erase(const struct lfs_config *c, lfs_block_t block);
static int block_device_sync(const struct lfs_config *c);

/* Private variables ---------------------------------------------------------*/

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = block_device_read,
    .prog  = block_device_prog,
    .erase = block_device_erase,
    .sync  = block_device_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = FLASH_PAGE_SIZE,
    .block_count = 2,
    .cache_size = 16,
    .lookahead_size = 16,
};
// variables used by the filesystem
static lfs_t lfs;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize Flash for file system
  * @param  None
  * @retval The handle of file system
  */
lfs_t* flash_init(void)
{
  int err;
  HAL_StatusTypeDef status;
  FLASH_OBProgramInitTypeDef OBProgramInit = {
    .OptionType = OPTIONBYTE_WRP,   /* Modify only the Write protection */
    .WRPState = OB_WRPSTATE_DISABLE,   /* Turn the protection off */
    .WRPPage = OB_WRP_PAGES62TO255,
  };

  /* Unlock the Program memory */
  HAL_FLASH_Unlock();
  /* Clear all FLASH flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  /* Unlock the Options Bytes */
  HAL_FLASH_OB_Unlock();
  /* Erase all the option Bytes */
  status = HAL_FLASHEx_OBErase();
  assert(HAL_OK == status);
  /* Program new option */
  status = HAL_FLASHEx_OBProgram(&OBProgramInit);
  assert(HAL_OK == status);
  /* Lock the Options Bytes */
  HAL_FLASH_OB_Lock();

  /* Unlock the Program memory */
  HAL_FLASH_Lock();

  /* Mount the filesystem */
  err = lfs_mount(&lfs, &cfg);

  if (err) {
    /* this should only happen on the first boot */
    console_printf("Can't mount the filesystem (err %d)\n", err);
    lfs_format(&lfs, &cfg);
    err = lfs_mount(&lfs, &cfg);
  }

  return err ? NULL : &lfs;
}

static int block_device_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size)
{
  uint32_t source;

  if (NULL == c || NULL == buffer || 0 == size) {
    return LFS_ERR_INVAL;
  }

  source = ADDR_FLASH_START_PAGE + block * c->block_size + off;
  assert((source + size - 1) <= FLASH_BANK1_END);
  memcpy(buffer, (void *)source, size);

  return LFS_ERR_OK;
}

static int block_device_prog(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size)
{
  HAL_StatusTypeDef status;
  uint32_t destination, data;
  int idx;

  if (NULL == c || NULL == buffer || 0 == size) {
    return LFS_ERR_INVAL;
  }

  destination = ADDR_FLASH_START_PAGE + block * c->block_size + off;

  /* Unlock the Flash to enable the flash control register access */
  HAL_FLASH_Unlock();

  for (idx = 0; idx < size && destination < FLASH_BANK1_END; idx += sizeof(uint32_t)) {
    memcpy(&data, (char *)buffer + idx, sizeof(data));
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destination, data);
    if (HAL_OK != status) {
      console_printf("Error occurred while writing data in Flash memory\n");
      break;
    }
    if (data != *(uint32_t *)destination) {
      console_printf("Flash content doesn't match SRAM content\n");
      break;
    }
    /* Increment FLASH destination address */
    destination += sizeof(uint32_t);
  }

  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  return (idx < size) ? LFS_ERR_CORRUPT : LFS_ERR_OK;
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  start: start of user flash area
  * @retval FLASHIF_OK : user flash area successfully erased
  *         FLASHIF_ERASEKO : error occurred
  */
static int block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
  uint32_t PageError;
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef EraseInit = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks = FLASH_BANK_1,
    .NbPages = 1,
  };

  if (NULL == c) {
    return LFS_ERR_INVAL;
  }

  EraseInit.PageAddress = ADDR_FLASH_START_PAGE + block * c->block_size;

  /* Unlock the Flash to enable the flash control register access */ 
  HAL_FLASH_Unlock();
  status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
  /* Lock the Flash to disable the flash control register access */
  HAL_FLASH_Lock();

  return (HAL_OK == status) ? LFS_ERR_OK : LFS_ERR_CORRUPT;
}

static int block_device_sync(const struct lfs_config *c)
{
  return LFS_ERR_OK;
}
