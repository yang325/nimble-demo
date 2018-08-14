#ifndef __SYS_CONFIG_H_
#define __SYS_CONFIG_H_

#include <stdio.h>

#define MYNEWT_VAL_BLE_MESH_MODEL_KEY_COUNT                             1
#define MYNEWT_VAL_BLE_MESH_MODEL_GROUP_COUNT                           2
#define MYNEWT_VAL_BLE_MESH_APP_KEY_COUNT                               2
#define MYNEWT_VAL_BLE_MESH_SUBNET_COUNT                                2
#define MYNEWT_VAL_BLE_MESH_CRPL                                        10
#define MYNEWT_VAL_BLE_MESH_MSG_CACHE_SIZE                              10
#define MYNEWT_VAL_BLE_MESH_ADV_BUF_COUNT                               20
#define MYNEWT_VAL_BLE_MESH_TX_SEG_MSG_COUNT                            1
#define MYNEWT_VAL_BLE_MESH_RX_SEG_MSG_COUNT                            1
#define MYNEWT_VAL_BLE_MESH_DEBUG_TRANS                                 1
#define MYNEWT_VAL_BLE_MESH_FRIEND                                      1
#define MYNEWT_VAL_BLE_MESH_LOW_POWER                                   0
#define MYNEWT_VAL_BLE_MESH_FRIEND_SUB_LIST_SIZE                        3
#define MYNEWT_VAL_BLE_MESH_FRIEND_SEG_RX                               1
#define MYNEWT_VAL_BLE_MESH_FRIEND_LPN_COUNT                            2
#define MYNEWT_VAL_BLE_MESH_DEBUG_BEACON                                0
#define MYNEWT_VAL_BLE_MESH_DEBUG_CRYPTO                                0
#define MYNEWT_VAL_BLE_MESH_PROV                                        1
#define MYNEWT_VAL_BLE_MESH_DEBUG_MODEL                                 0
#define MYNEWT_VAL_BLE_MESH_DEBUG_ADV                                   0
#define MYNEWT_VAL_BLE_MESH_DEBUG                                       0
#define MYNEWT_VAL_BLE_MESH_PB_GATT                                     1
#define MYNEWT_VAL_BLE_MESH_LABEL_COUN                                  3
#define MYNEWT_VAL_BLE_MESH_RELAY                                       1
#define MYNEWT_VAL_BLE_MESH_GATT_PROXY                                  1
#define MYNEWT_VAL_BLE_MESH_TESTING                                     0
#define MYNEWT_VAL_BLE_MESH_DEBUG_NET                                   0
#define MYNEWT_VAL_BLE_MESH_IV_UPDATE_TEST                              0
#define MYNEWT_VAL_BLE_MESH_IVU_DIVIDER                                 4
#define MYNEWT_VAL_BLE_MESH_DEBUG_PROV                                  0
#define MYNEWT_VAL_BLE_MESH_FRIEND_QUEUE_SIZE                           16
#define MYNEWT_VAL_BLE_MESH_DEBUG_FRIEND                                0
#define MYNEWT_VAL_BLE_MESH_FRIEND_RECV_WIN                             255

#define MYNEWT_VAL_BLE_MESH_RX_SDU_MAX                                  384
#define MYNEWT_VAL_BLE_MESH_PB_ADV                                      1

#define MYNEWT_VAL_BLE_MESH_PROXY                                       1
#define MYNEWT_VAL_BLE_MESH_LABEL_COUNT                                 3
#define MYNEWT_VAL_BLE_MESH_DEBUG_ACCESS                                0
#define MYNEWT_VAL_BLE_MESH_PROXY_FILTER_SIZE                           3
#define MYNEWT_VAL_BLE_MESH_DEBUG_PROXY                                 0
#define MYNEWT_VAL_BLE_MESH_NODE_ID_TIMEOUT                             60
#define MYNEWT_VAL_BLE_MESH_DEVICE_NAME                                 "NimBLE"

#define MYNEWT_VAL_BLE_MESH_DEBUG_SETTINGS                              0
#define MYNEWT_VAL_BLE_MESH_SEQ_STORE_RATE                              128
#define MYNEWT_VAL_BLE_MESH_RPL_STORE_TIMEOUT                           5
#define MYNEWT_VAL_BLE_MESH_STORE_TIMEOUT                               2

/**
 * Configuration handler, used to register a config item/subtree.
 */
struct conf_handler {
    /**
     * The name of the conifguration item/subtree
     */
    char *ch_name;
    /** Get configuration value */
    conf_get_handler_t ch_get;
    /** Set configuration value */
    conf_set_handler_t ch_set;
    /** Commit configuration value */
    conf_commit_handler_t ch_commit;
    /** Export configuration value */
    conf_export_handler_t ch_export;
};

#endif
