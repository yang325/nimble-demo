/* Includes ------------------------------------------------------------------*/

#include "store_flash.h"
#include "flash.h"

/* Private variables ---------------------------------------------------------*/

static lfs_t *handle = NULL;

/* Exported functions --------------------------------------------------------*/

void store_flash_init(void)
{
    /* Initilize flash driver */
    handle = flash_init();
    if (NULL == handle) {
        console_printf("Can't initilize the file system\n");
        return;
    }
}

int store_flash_read(int obj_type, const union ble_store_key *key,
                        union ble_store_value *value)
{
    switch (obj_type) {
        case BLE_STORE_OBJ_TYPE_PEER_SEC:
        case BLE_STORE_OBJ_TYPE_OUR_SEC:
        default:
            break;
    }

    return BLE_HS_ENOTSUP;
}

int store_flash_write(int obj_type, const union ble_store_value *val)
{
    switch (obj_type) {
        case BLE_STORE_OBJ_TYPE_PEER_SEC:
        case BLE_STORE_OBJ_TYPE_OUR_SEC:
        default:
            break;
    }

    return BLE_HS_ENOTSUP;
}

int store_flash_delete(int obj_type, const union ble_store_key *key)
{
    switch (obj_type) {
        case BLE_STORE_OBJ_TYPE_PEER_SEC:
        case BLE_STORE_OBJ_TYPE_OUR_SEC:
        default:
            break;
    }

    return BLE_HS_ENOTSUP;
}
