#include <assert.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

/* Private function prototypes -----------------------------------------------*/

static int svc_demo_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt,
                            void *arg);

/* Private variables ---------------------------------------------------------*/

static const struct ble_gatt_svc_def svc_demo_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFE95),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x0001),
                .flags = BLE_GATT_CHR_F_READ,
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
    return 0;
}