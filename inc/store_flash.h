#ifndef H_STORE_FLASH_
#define H_STORE_FLASH_

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>

#include "host/ble_hs.h"

/* Exported functions --------------------------------------------------------*/

void store_flash_init(void);
int  store_flash_read(int obj_type, const union ble_store_key *key,
                        union ble_store_value *value);
int  store_flash_write(int obj_type, const union ble_store_value *val);
int  store_flash_delete(int obj_type, const union ble_store_key *key);

#endif
