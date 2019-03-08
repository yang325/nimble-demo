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

static void system_info_output(void);

static int cmd_shell_uart_tx_char(void *arg);
static int cmd_shell_uart_rx_char(void *arg, uint8_t data);
static int cmd_shell_printf(const char *fmt, ...);

/* Private variables ---------------------------------------------------------*/

static app_fifo_t tx_fifo, rx_fifo;

/* Exported functions --------------------------------------------------------*/

void cmd_shell_thread(void * arg)
{
    int ret;
    cli_status_t status;
    uint8_t tx_buf[64], rx_buf[4], data;

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

    /* Output system information */
    system_info_output();

    status = cli_init(CLI_NODE_ID_MAX, FALSE);
    assert_param(CLI_OK == status);

    cli_hostname_set("MINI-CLI", strlen("MINI-CLI"));

    status = cli_global_cmd_node_list(CLI_NODE_ID_CONFIG,
                                      CLI_NODE_ID_SDK,
                                      CLI_NODE_ID_TEST,
                                      CLI_NODE_ID_INVALID);
    assert_param(CLI_OK == status);

    while (1) {
        cli_main(CLI_SESSION_CONSOLE, stdin, stdout);
        taskENTER_CRITICAL();
        ret = app_fifo_get(&rx_fifo, &data);
        taskEXIT_CRITICAL();
        if (0 == ret) {
            app_fifo_put(&tx_fifo, data);
            hal_uart_start_tx(CMD_SHELL_UART_PORT);
        } else {
            vTaskDelay(100);
        }
    }
}

/**
 * Prints the specified format string to the console.
 *
 * @return                      The number of characters that would have been
 *                                  printed if the console buffer were
 *                                  unlimited.  This return value is analogous
 *                                  to that of snprintf.
 */
static int cmd_shell_printf(const char *fmt, ...)
{
    va_list args;
    int len;

    va_start(args, fmt);
    len = vprintf(fmt, args);
    va_end(args);

    if (len > 0) {
        hal_uart_start_tx(CMD_SHELL_UART_PORT);
    }

    return len;
}

int _write(int file, char *ptr, int len)
{
    uint32_t size = len;

    app_fifo_write(&tx_fifo, (uint8_t const *)ptr, &size);

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

/**
  * @}
  */
static void system_info_output(void)
{
    uint8_t varient, revision;

    cmd_shell_printf("\n");

    varient = (SCB->CPUID & SCB_CPUID_VARIANT_Msk) >> SCB_CPUID_VARIANT_Pos;
    revision = (SCB->CPUID & SCB_CPUID_REVISION_Msk) >> SCB_CPUID_REVISION_Pos;
    SystemCoreClockUpdate();

    cmd_shell_printf("- ARM Cortex-M3 r%dp%d Core -\n", varient, revision);
    cmd_shell_printf("- Core Frequency = %u Hz -\n", SystemCoreClock);
}

