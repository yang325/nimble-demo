/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "cmd.h"
#include "bsp/bsp.h"
#include "hal/hal_uart.h"
#include "app_fifo.h"

#include "cli_api.h"
#include "cli_auth.h"

#include "stm32f1xx_hal.h"

/* Private define ------------------------------------------------------------*/

#define CMD_SHELL_UART_PORT                      (1)
#define CMD_SHELL_UART_BAUD                      (115200)
#define CMD_SHELL_UART_DATA_BITS                 (8)
#define CMD_SHELL_UART_STOP_BITS                 (1)
#define CMD_SHELL_UART_PARITY                    (HAL_UART_PARITY_NONE)
#define CMD_SHELL_UART_FLOW_CTRL                 (HAL_UART_FLOW_CTL_NONE)

/* Private typedef -----------------------------------------------------------*/

typedef enum {
    CLI_NODE_ID_SDK = CLI_NODE_ID_USER_START,
    CLI_NODE_ID_TEST,
    CLI_NODE_ID_MAX,
} cli_demo_node_id_t;

/* Private function prototypes -----------------------------------------------*/

static int cmd_shell_uart_tx_char(void *arg);
static int cmd_shell_uart_rx_char(void *arg, uint8_t data);

/* Private variables ---------------------------------------------------------*/

static app_fifo_t tx_fifo, rx_fifo;

/* Exported functions --------------------------------------------------------*/

CLI_DEFINE(cli_handle_system_info_handler, system_info_handler_cmd, "info",
        "Output system infomation\n")
{
    uint8_t varient, revision;

    varient = (SCB->CPUID & SCB_CPUID_VARIANT_Msk) >> SCB_CPUID_VARIANT_Pos;
    revision = (SCB->CPUID & SCB_CPUID_REVISION_Msk) >> SCB_CPUID_REVISION_Pos;
    SystemCoreClockUpdate();

    cli_print(cli, "- ARM Cortex-M3 r%dp%d Core -\r\n", varient, revision);
    cli_print(cli, "- Core Frequency = %lu Hz -\r\n", SystemCoreClock);

    return CLI_OK;
}

void cmd_shell_thread(void * arg)
{
    int ret;
    cli_status_t status;
    uint8_t tx_buf[64], rx_buf[4];

    ret = app_fifo_init(&tx_fifo, tx_buf, sizeof(tx_buf));
    assert_param(0 == ret);

    ret = app_fifo_init(&rx_fifo, rx_buf, sizeof(rx_buf));
    assert_param(0 == ret);

    ret = hal_uart_init_cbs(CMD_SHELL_UART_PORT,
                            cmd_shell_uart_tx_char, NULL,
                            cmd_shell_uart_rx_char, NULL);
    assert_param(0 == ret);

    ret = hal_uart_config(CMD_SHELL_UART_PORT,
                          CMD_SHELL_UART_BAUD,
                          CMD_SHELL_UART_DATA_BITS,
                          CMD_SHELL_UART_STOP_BITS,
                          CMD_SHELL_UART_PARITY,
                          CMD_SHELL_UART_FLOW_CTRL);
    assert_param(0 == ret);

    status = cli_init(CLI_NODE_ID_MAX, FALSE);
    assert_param(CLI_OK == status);

    cli_hostname_set("MINI-CLI", strlen("MINI-CLI"));

    status = cli_global_cmd_node_list(CLI_NODE_ID_CONFIG,
                                      CLI_NODE_ID_SDK,
                                      CLI_NODE_ID_TEST,
                                      CLI_NODE_ID_INVALID);
    assert_param(CLI_OK == status);

    status = cli_install_global_cmd(&system_info_handler_cmd);
    assert_param(CLI_OK == status);

    status = cli_global_cmd_node_range(CLI_NODE_ID_CONFIG, CLI_NODE_ID_TEST);
    assert_param(CLI_OK == status);

    status = cli_add_node(CLI_NODE_ID_SDK);
    assert_param(CLI_OK == status);

    while (1) {
        cli_main(CLI_SESSION_CONSOLE, stdin, stdout);
    }
}

int _write(int file, char *ptr, int len)
{
    uint32_t size = 0;

    if (len > 0) {
        size = len;
        app_fifo_write(&tx_fifo, (uint8_t const *)ptr, &size);
        hal_uart_start_tx(CMD_SHELL_UART_PORT);
    }

    return size;
}

int _read(int file, char *ptr, int len)
{
    uint32_t ret, size;

    do {
        size = len;
        taskENTER_CRITICAL();
        ret = app_fifo_read(&rx_fifo, (uint8_t *)ptr, &size);
        taskEXIT_CRITICAL();
        if (0 != ret) {
            vTaskDelay(100);
        }
    } while(0 != ret);

    return size;
}

static int cmd_shell_uart_tx_char(void *arg)
{
    uint8_t data;

    if (0 == app_fifo_get(&tx_fifo, &data)) {
        return data;
    } else {
        return -1;
    }
}

static int cmd_shell_uart_rx_char(void *arg, uint8_t data)
{
    taskENTER_CRITICAL();
    app_fifo_put(&rx_fifo, data);
    taskEXIT_CRITICAL();

    return 0;
}


