/* Includes ------------------------------------------------------------------*/

#include "store_flash.h"
#include "flash.h"
#include "stm32f1xx_hal.h"

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

int store_flash_init(void)
{
    int err;
    /* Initilize flash driver */
    err = flash_init();
    if (err) {
        console_printf("Can't initilize the flash (err %d)\n", err);
        return OS_ERROR;
    }
    /* Mount the filesystem */
    err = lfs_mount(&lfs, &cfg);
    if (err) {
        /* this should only happen on the first boot */
        console_printf("Can't mount the filesystem (err %d)\n", err);
        lfs_format(&lfs, &cfg);
        err = lfs_mount(&lfs, &cfg);
        if (err) {
            console_printf("Something is wrong, abort it (err %d)\n", err);
            return OS_ERROR;
        }
    }

    return OS_OK;
}

/*
 * Append a single value to persisted config. Don't store duplicate value.
 */
int conf_save_one(const char *name, char *value)
{
    lfs_file_t file;
    int err;

    if (NULL == name) {
        return OS_INVALID_PARM;
    }

    err = lfs_file_open(&lfs, &file, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
    if (err) {
        console_printf("Can't open the file %s (err %d)\n", name, err);
        return OS_ERROR;
    }

    if (value) {
        err = lfs_file_write(&lfs, &file, value, strlen(value) + 1);
        if (err) {
            console_printf("Can't write the file %s (err %d)\n", name, err);
        }
    } else {
        err = lfs_file_truncate(&lfs, &file, 0);
        if (err) {
            console_printf("Can't truncate the file %s (err %d)\n", name, err);
        }
    }

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);
    return (LFS_ERR_OK == err) ? OS_OK : OS_ERROR;
}

int conf_load(void)
{
    return OS_OK;
}
