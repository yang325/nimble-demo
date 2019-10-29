#ifndef H_STORE_FLASH_
#define H_STORE_FLASH_

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>

#include "host/ble_hs.h"

/* Exported functions --------------------------------------------------------*/

int store_flash_init(void);

/**
 * Write a single configuration value to persisted storage (if it has
 * changed value).
 *
 * @param name Name/key of the configuration item.
 * @param var Value of the configuration item.
 *
 * @return 0 on success, non-zero on failure.
 */
int conf_save_one(const char *name, char *var);

/**
 * Load configuration from registered persistence sources. Handlers for
 * configuration subtrees registered earlier will be called for encountered
 * values.
 *
 * @return 0 on success, non-zero on failure.
 */
int conf_load(void);

#endif
