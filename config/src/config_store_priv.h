/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_STORE_PRIV_H_
#define __CONFIG_STORE_PRIV_H_

/* Includes ------------------------------------------------------------------*/

#include "lfs.h"

/* Exported functions --------------------------------------------------------*/

int block_device_init(void);
int block_device_read(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, void *buffer, lfs_size_t size);
int block_device_prog(const struct lfs_config *c, lfs_block_t block,
                            lfs_off_t off, const void *buffer, lfs_size_t size);
int block_device_erase(const struct lfs_config *c, lfs_block_t block);
int block_device_sync(const struct lfs_config *c);

#endif  /* __CONFIG_STORE_PRIV_H_ */
