/* Includes ------------------------------------------------------------------*/

#include "app.h"
#include "bsp/bsp.h"

#include "mesh/mesh.h"
#include "mesh/main.h"
#include "mesh/glue.h"
#include "mesh/porting.h"
#include "mesh/model_srv.h"

/* Private define ------------------------------------------------------------*/

#define CID_VENDOR                      0x038F
#define FAULT_ARR_SIZE                  2
#define STANDARD_TEST_ID                0x00
#define TEST_ID                         0x01

/* Private function prototypes -----------------------------------------------*/

static int  output_number(bt_mesh_output_action_t action, uint32_t number);
static void prov_complete(uint16_t net_idx, uint16_t addr);

static int  fault_get_cur(struct bt_mesh_model *model, uint8_t *test_id,
              uint16_t *company_id, uint8_t *faults, uint8_t *fault_count);
static int  fault_get_reg(struct bt_mesh_model *model, uint16_t company_id,
              uint8_t *test_id, uint8_t *faults, uint8_t *fault_count);
static int  fault_clear(struct bt_mesh_model *model, uint16_t company_id);
static int  fault_test(struct bt_mesh_model *model, uint8_t test_id, uint16_t company_id);

static void health_pub_init(void);

static int  led_model_gen_onoff_get(struct bt_mesh_model *model, u8_t *state);
static int  led_model_gen_onoff_set(struct bt_mesh_model *model, u8_t state);

/* Private variables ---------------------------------------------------------*/

static bool has_reg_fault = true;
static int recent_test_id = STANDARD_TEST_ID;

static uint8_t dev_uuid[16];

static struct bt_mesh_cfg_srv cfg_srv = {
  .relay = BT_MESH_RELAY_DISABLED,
  .beacon = BT_MESH_BEACON_ENABLED,
#if MYNEWT_VAL(BLE_MESH_FRIEND)
  .frnd = BT_MESH_FRIEND_ENABLED,
#else
  .frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if MYNEWT_VAL(BLE_MESH_GATT_PROXY)
  .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
  .gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
  .default_ttl = 7,
  /* 3 transmissions with 20ms interval */
  .net_transmit = BT_MESH_TRANSMIT(2, 20),
  .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

static const struct bt_mesh_health_srv_cb health_srv_cb = {
  .fault_get_cur = &fault_get_cur,
  .fault_get_reg = &fault_get_reg,
  .fault_clear = &fault_clear,
  .fault_test = &fault_test,
};

static struct bt_mesh_health_srv health_srv = {
  .cb = &health_srv_cb,
};

static struct bt_mesh_model_pub health_pub;

static struct bt_mesh_gen_onoff_srv_cb gen_onoff_srv_cb = {
  .get = led_model_gen_onoff_get,
  .set = led_model_gen_onoff_set,
};

static struct bt_mesh_model_pub gen_onoff_pub;

static struct bt_mesh_model root_models[] = {
  BT_MESH_MODEL_CFG_SRV(&cfg_srv),
  BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
  BT_MESH_MODEL_GEN_ONOFF_SRV(&gen_onoff_srv_cb, &gen_onoff_pub),
};

static struct bt_mesh_elem elements[] = {
  BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_prov prov = {
  .uuid = dev_uuid,
  .output_size = 4,
  .output_actions = BT_MESH_DISPLAY_NUMBER,
  .output_number = output_number,
  .complete = prov_complete,
};

static const struct bt_mesh_comp comp = {
  .cid = CID_VENDOR,
  .elem = elements,
  .elem_count = ARRAY_SIZE(elements),
};

/* Exported functions --------------------------------------------------------*/

void mesh_demo_init(uint8_t uuid[16])
{
  int ret;

  memcpy(&dev_uuid[0], &uuid[0], 16);

  health_pub_init();

  ret = bt_mesh_init(BLE_ADDR_PUBLIC, &prov, &comp);
  if (ret) {
    console_printf("Initializing mesh failed (err %d)\n", ret);
    return;
  }

  if (pdPASS != xTaskCreate(mesh_adv_thread, "adv", DEMO_TASK_MESH_ADV_SIZE,
                            NULL, DEMO_TASK_MESH_ADV_PRIORITY, NULL)) {

    console_printf("Creating advertising task failed\n");
    return;
  }

  console_printf("Mesh initialized\n");
}

/* Private functions ---------------------------------------------------------*/

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
  console_printf("OOB Number: %lu\n", number);

  return 0;
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
  console_printf("Local node provisioned, primary address 0x%04x\n", addr);
}

static int fault_get_cur(struct bt_mesh_model *model, uint8_t *test_id,
              uint16_t *company_id, uint8_t *faults, uint8_t *fault_count)
{
  uint8_t reg_faults[FAULT_ARR_SIZE] = { [0 ... FAULT_ARR_SIZE-1] = 0xff };

  console_printf("fault_get_cur() has_reg_fault %u\n", has_reg_fault);

  *test_id = recent_test_id;
  *company_id = CID_VENDOR;

  *fault_count = min(*fault_count, sizeof(reg_faults));
  memcpy(faults, reg_faults, *fault_count);

  return 0;
}

static int fault_get_reg(struct bt_mesh_model *model, uint16_t company_id,
              uint8_t *test_id, uint8_t *faults, uint8_t *fault_count)
{
  if (company_id != CID_VENDOR) {
    return -BLE_HS_EINVAL;
  }

  console_printf("fault_get_reg() has_reg_fault %u\n", has_reg_fault);

  *test_id = recent_test_id;

  if (has_reg_fault) {
    uint8_t reg_faults[FAULT_ARR_SIZE] = { [0 ... FAULT_ARR_SIZE-1] = 0xff };

    *fault_count = min(*fault_count, sizeof(reg_faults));
    memcpy(faults, reg_faults, *fault_count);
  } else {
    *fault_count = 0;
  }

  return 0;
}

static int fault_clear(struct bt_mesh_model *model, uint16_t company_id)
{
  if (company_id != CID_VENDOR) {
    return -BLE_HS_EINVAL;
  }

  has_reg_fault = false;

  return 0;
}

static int fault_test(struct bt_mesh_model *model, uint8_t test_id, uint16_t company_id)
{
  if (company_id != CID_VENDOR) {
    return -BLE_HS_EINVAL;
  }

  if (test_id != STANDARD_TEST_ID && test_id != TEST_ID) {
    return -BLE_HS_EINVAL;
  }

  recent_test_id = test_id;
  has_reg_fault = true;
  bt_mesh_fault_update(bt_mesh_model_elem(model));

  return 0;
}

static void health_pub_init(void)
{
  health_pub.msg  = BT_MESH_HEALTH_FAULT_MSG(0);
}

static int led_model_gen_onoff_get(struct bt_mesh_model *model, u8_t *state)
{
  *state = led_state() ? 0x01 : 0x00;

  return 0;
}

static int led_model_gen_onoff_set(struct bt_mesh_model *model, u8_t state)
{
  if (state) {
    led_on();
  } else {
    led_off();
  }

  return 0;
}

