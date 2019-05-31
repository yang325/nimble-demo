#include <assert.h>
#include <string.h>

#include "service.h"

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

#include "stm32f1xx_hal.h"

/* Private function prototypes -----------------------------------------------*/

static int svc_demo_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg);
static int svc_demo_uid_read_access(struct ble_gatt_access_ctxt *ctxt);
static int svc_demo_led_write_access(struct ble_gatt_access_ctxt *ctxt);

/* Private variables ---------------------------------------------------------*/

static const struct ble_gatt_svc_def svc_demo_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SERVICE_DEMO_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(SERVICE_CHAR_UID_UUID),
                .flags = BLE_GATT_CHR_F_READ,
                .access_cb = svc_demo_access,
            },

            {
                .uuid = BLE_UUID16_DECLARE(SERVICE_CHAR_LED_UUID),
                .flags = BLE_GATT_CHR_F_WRITE,
                .access_cb = svc_demo_access,
            },

            {
                0, /* No more characteristics in this service. */
            }
        }
    },

    {
        0, /* No more services. */
    }
};

/**
 * Initialize the ANS with initial values for enabled categories
 * for new and unread alert characteristics. Bitwise or the
 * catagory bitmasks to enable multiple catagories.
 *
 * XXX: We should technically be able to change the new alert and
 *      unread alert catagories when we have no active connections.
 */
void svc_demo_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = ble_gatts_count_cfg(svc_demo_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(svc_demo_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
}

/**
 * ANS access function
 */
static int svc_demo_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg)
{
    int ret;
    uint16_t uuid16;

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
        case SERVICE_CHAR_UID_UUID:
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                ret = svc_demo_uid_read_access(ctxt);
            } else {
                ret = BLE_ATT_ERR_REQ_NOT_SUPPORTED;
            }
            break;
        case SERVICE_CHAR_LED_UUID:
            if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                ret = svc_demo_led_write_access(ctxt);
            } else {
                ret = BLE_ATT_ERR_REQ_NOT_SUPPORTED;
            }
            break;
        default:
            ret = BLE_ATT_ERR_REQ_NOT_SUPPORTED;
            break;
    }

    return ret;
}

static int svc_demo_uid_read_access(struct ble_gatt_access_ctxt *ctxt)
{
    uint32_t uid[3];
    int ret;

    HAL_GetUID(uid);
    ret = os_mbuf_append(ctxt->om, uid, sizeof(uid));

    return (ret == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int svc_demo_led_write_access(struct ble_gatt_access_ctxt *ctxt)
{
    int ret;
    uint8_t data[20];
    uint16_t len;

    ret = ble_hs_mbuf_to_flat(ctxt->om, &data[0], sizeof(data), &len);
    if (ret) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    console_printf("LED command: ");
    for (int index = 0; index < len; ++index) {
        console_printf("0x%02x ", data[index]);
    }
    console_printf("\n");

    return 0;
}
