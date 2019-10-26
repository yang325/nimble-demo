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

void store_flash_init(void)
{
    int err;
    /* Initilize flash driver */
    err = flash_init();
    if (err) {
        console_printf("Can't initilize the flash (err %d)\n", err);
        return;
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
            return;
        }
    }
}
