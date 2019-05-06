/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>

#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "service.h"
#include "bsp/bsp.h"

#include "transport/uart/ble_hci_uart.h"
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

/* Private define ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static void system_clock_config(void);
static void system_info_output(void);

static void ble_controller_init(void);
static void ble_controller_enable(void);

static void ble_app_on_sync(void);
static void ble_host_thread(void * arg);

static int  ble_gap_event_handler(struct ble_gap_event *event, void *arg);
static int  ble_gap_passkey_handler(uint16_t conn_handle, uint8_t action, uint32_t numcmp);
static int  ble_gap_disconnect_handler(int reason);

/* Private variables ---------------------------------------------------------*/


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  system_clock_config();

  /* Initialize LED */
  led_init();

  /* NimBLE host task definition */
  BaseType_t ret = xTaskCreate(ble_host_thread, "host", APP_TASK_BLE_HOST_SIZE,
                               NULL, APP_TASK_BLE_HOST_PRIORITY, NULL);
  assert(pdPASS == ret);

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  assert(0);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void system_clock_config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  HAL_StatusTypeDef status;

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  assert(HAL_OK == status);

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  assert(HAL_OK == status);

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
  * @brief Output System Information
  * @retval None
  */
static void system_info_output(void)
{
  uint8_t varient, revision;

  console_printf("\n");

  varient = (SCB->CPUID & SCB_CPUID_VARIANT_Msk) >> SCB_CPUID_VARIANT_Pos;
  revision = (SCB->CPUID & SCB_CPUID_REVISION_Msk) >> SCB_CPUID_REVISION_Pos;
  SystemCoreClockUpdate();

  console_printf("- ARM Cortex-M3 r%dp%d Core -\n", varient, revision);
  console_printf("- Core Frequency = %lu Hz -\n", SystemCoreClock);
}

static void ble_controller_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Reset the controller */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(5);
}

static void ble_controller_enable(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(5);
}

/**
  * @brief  Handle BLE sync event
  * @param  None
  * @retval None
  */
static void ble_app_on_sync(void)
{
  int ret;
  uint32_t seed;
  ble_addr_t addr;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name;

  console_printf("The host and controller are in sync\n");

  /* Use NRPA */
  ret = ble_hs_id_gen_rnd(1, &addr);
  if (ret) {
    console_printf("Initializing random address failed (err %d)\n", ret);
    return;
  }

  /* Use random address as PRNG seed */
  memcpy(&seed, addr.val, sizeof(seed));
  srand(seed);

  ret = ble_hs_id_set_rnd(addr.val);
  if (ret) {
    console_printf("Setting random address failed (err %d)\n", ret);
    return;
  }

  /**
   *  Set the advertisement data included in our advertisements:
   *     o Flags (indicates advertisement type and other general info).
   *     o Advertising tx power.
   *     o Device name.
   *     o 16-bit service UUIDs (alert notifications).
   */
  memset(&fields, 0, sizeof fields);

  /* Advertise two flags:
   *     o Discoverability in forthcoming advertisement (general)
   *     o BLE-only (BR/EDR unsupported).
   */
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  name = ble_svc_gap_device_name();
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;

  fields.uuids16 = (ble_uuid16_t[]){
    BLE_UUID16_INIT(SERVICE_DEMO_UUID)
  };
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;

  ret = ble_gap_adv_set_fields(&fields);
  if (ret) {
    console_printf("Setting advertisement data failed (err %d)\n", ret);
    return;
  }

  /* Begin advertising. */
  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ret = ble_gap_adv_start(addr.type, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event_handler, NULL);
  if (ret) {
    console_printf("Enabling advertisement failed (err %d)\n", ret);
    return;
  }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleprph uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleprph.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
  int ret = 0;

  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      console_printf("connection %s; status = %d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      ret = ble_gap_disconnect_handler(event->disconnect.reason);
      break;
    case BLE_GAP_EVENT_CONN_UPDATE:
      console_printf("connection updated; status = %d\n",
                    event->conn_update.status);
      break;
    case BLE_GAP_EVENT_CONN_UPDATE_REQ:
      console_printf("connection update request\n");
      break;
    case BLE_GAP_EVENT_ENC_CHANGE:
      console_printf("encryption change event; status = %d\n",
                    event->enc_change.status);
      break;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
      ret = ble_gap_passkey_handler(event->passkey.conn_handle,
                                    event->passkey.params.action,
                                    event->passkey.params.numcmp);
      break;
    default:
      console_printf("un-handled event type = %d\n", event->type);
      break;
  }

  return ret;
}

static int ble_gap_passkey_handler(uint16_t conn_handle, uint8_t action, uint32_t numcmp)
{
  int ret = 0;
  struct ble_sm_io pk;

  switch (action) {
    case BLE_SM_IOACT_OOB:
      break;
    case BLE_SM_IOACT_INPUT:
      break;
    case BLE_SM_IOACT_DISP:
      pk.passkey = rand() % 1000000;             /* Max value is 999999 */
      pk.action = BLE_SM_IOACT_DISP;
      console_printf("passkey display event, number = %06u\n", pk.passkey);
      ret = ble_sm_inject_io(conn_handle, &pk);
      break;
    case BLE_SM_IOACT_NUMCMP:
      console_printf("passkey number compare event, numcmp = %u\n", numcmp);
      break;
    default:
      break;
  }

  return ret;
}

static int ble_gap_disconnect_handler(int reason)
{
  int ret;
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  const char *name;

  console_printf("disconnect; reason = 0x%x\n", reason);

  /**
   *  Set the advertisement data included in our advertisements:
   *     o Flags (indicates advertisement type and other general info).
   *     o Advertising tx power.
   *     o Device name.
   *     o 16-bit service UUIDs (alert notifications).
   */
  memset(&fields, 0, sizeof fields);

  /* Advertise two flags:
   *     o Discoverability in forthcoming advertisement (general)
   *     o BLE-only (BR/EDR unsupported).
   */
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  name = ble_svc_gap_device_name();
  fields.name = (uint8_t *)name;
  fields.name_len = strlen(name);
  fields.name_is_complete = 1;

  fields.uuids16 = (ble_uuid16_t[]){
    BLE_UUID16_INIT(SERVICE_DEMO_UUID)
  };
  fields.num_uuids16 = 1;
  fields.uuids16_is_complete = 1;

  ret = ble_gap_adv_set_fields(&fields);
  if (ret) {
    console_printf("Setting advertisement data failed (err %d)\n", ret);
    return ret;
  }

  /* Begin advertising. */
  memset(&adv_params, 0, sizeof adv_params);
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ret = ble_gap_adv_start(BLE_ADDR_RANDOM, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event_handler, NULL);
  if (ret) {
    console_printf("Enabling advertisement failed (err %d)\n", ret);
    return ret;
  }

  return 0;
}

/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent from on_ble_evt().
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void ble_host_thread(void * arg)
{
  /* Output System Information */
  system_info_output();

  /* Initialize BLE controller */
  ble_controller_init();

  /* Initialize UART as HCI */
  ble_hci_uart_init();

  /* Initialize NimBLE host */
  nimble_port_init();

  /* Enable BLE controller */
  ble_controller_enable();

  /* Initialize GAP, GATT and Mesh related services */
  ble_svc_gap_init();
  ble_svc_gatt_init();
  svc_demo_init();

  /* Set the default device name. */
  int ret = ble_svc_gap_device_name_set(MYNEWT_VAL(BLE_SVC_GAP_DEVICE_NAME));
  assert(ret == 0);

  /* Initialize the NimBLE host configuration */
  ble_hs_cfg.sync_cb = ble_app_on_sync;

  /* Handle NimBLE events */
  nimble_port_run();
}

/**
  * @}
  */
void __assert_func(const char *file, int line, const char *func, const char *condition)
{
  console_printf("Assert failed in %s, %s:%d (%s)", func, file, line, condition);
  /* Generate breakpoint if debugger is connected */
  __BKPT(0);
  /* Disable IRQ */
  __disable_irq();
  while(1)
  {
    led_toggle();
    uint32_t delay = 500000;
    while (delay --)
    {
      __asm("nop");
    }
  }
}

/**
  * @}
  */
void prvGetRegistersFromStack(uint32_t * pulFaultStackAddress)
{
  console_printf("R0 = 0x%08x\n", pulFaultStackAddress[0]);
  console_printf("R1 = 0x%08x\n", pulFaultStackAddress[1]);
  console_printf("R2 = 0x%08x\n", pulFaultStackAddress[2]);
  console_printf("R3 = 0x%08x\n", pulFaultStackAddress[3]);

  console_printf("R12 = 0x%08x\n", pulFaultStackAddress[4]);
  console_printf("LR = 0x%08x\n", pulFaultStackAddress[5]);
  console_printf("PC = 0x%08x\n", pulFaultStackAddress[6]);
  console_printf("PSR = 0x%08x\n", pulFaultStackAddress[7]);

  assert(0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
