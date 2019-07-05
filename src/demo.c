/* Includes ------------------------------------------------------------------*/

#include "app.h"
#include "bsp/bsp.h"

#include "mesh/mesh.h"
#include "mesh/main.h"
#include "mesh/glue.h"
#include "mesh/porting.h"
#include "mesh/model_cli.h"

/* Private define ------------------------------------------------------------*/

#define CID_VENDOR                      0x038F

/* Private function prototypes -----------------------------------------------*/

static void health_current_status(struct bt_mesh_health_cli *cli, u16_t addr,
				            u8_t test_id, u16_t cid, u8_t *faults, size_t fault_count);

/* Private variables ---------------------------------------------------------*/

static u8_t dev_uuid[16];
static const u8_t net_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u8_t dev_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u8_t app_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u16_t net_idx = 0;
static const u16_t app_idx = 0;
static const u32_t iv_index = 0x0000;
static u8_t flags = 0;
static u16_t dev_addr = 0x0001;

static struct bt_mesh_cfg_cli cfg_cli = {
};

static struct bt_mesh_health_cli health_cli = {
  .current_status = health_current_status,
};

static struct bt_mesh_model root_models[] = {
  BT_MESH_MODEL_CFG_CLI(&cfg_cli),
  BT_MESH_MODEL_HEALTH_CLI(&health_cli),
  BT_MESH_MODEL_GEN_ONOFF_CLI(),
  BT_MESH_MODEL_GEN_LEVEL_CLI(),
};

static struct bt_mesh_elem elements[] = {
  BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_prov prov = {
  .uuid = dev_uuid,
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
  ble_addr_t addr;

  memcpy(&dev_uuid[0], &uuid[0], 16);

  /* Use NRPA */
  ret = ble_hs_id_gen_rnd(1, &addr);
  if (ret) {
    console_printf("Initializing random address failed (err %d)\n", ret);
    return;
  } else {
    console_printf("The device address: %02x-%02x-%02x-%02x-%02x-%02x (type %d)\n",
                      addr.val[5], addr.val[4], addr.val[3], addr.val[2],
                      addr.val[1], addr.val[0], addr.type);
  }

  ret = ble_hs_id_set_rnd(addr.val);
  if (ret) {
    console_printf("Setting random address failed (err %d)\n", ret);
    return;
  }

  ret = bt_mesh_init(addr.type, &prov, &comp);
  if (ret) {
    console_printf("Initializing mesh failed (err %d)\n", ret);
    return;
  } else {
    console_printf("Mesh initialized\n");
  }

  ret = bt_mesh_provision(net_key, net_idx, flags, iv_index, dev_addr, dev_key);
  if (ret) {
    console_printf("Mesh provisioning failed (err %d)\n", ret);
    return;
  } else {
    console_printf("Provisioning completed\n");
  }

  /* Add Application Key */
  ret = bt_mesh_cfg_app_key_add(net_idx, dev_addr, net_idx, app_idx, app_key, NULL);
  if (ret) {
    console_printf("Adding application key failed (err %d)\n", ret);
    return;
  }

  if (pdPASS != xTaskCreate(mesh_adv_thread, "mesh", APP_TASK_BLE_MESH_SIZE,
                            NULL, APP_TASK_BLE_MESH_PRIORITY, NULL)) {

    console_printf("Creating mesh task failed\n");
    return;
  }
}

/* Private functions ---------------------------------------------------------*/

static void health_current_status(struct bt_mesh_health_cli *cli, u16_t addr,
				              u8_t test_id, u16_t cid, u8_t *faults, size_t fault_count)
{
	console_printf("Health Current Status from 0x%04x\n", addr);
}
