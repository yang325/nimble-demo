/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "cmd.h"
#include "bsp/bsp.h"
#include "hal/hal_uart.h"
#include "app_fifo.h"

#include "stm32f1xx_hal.h"

/* Private define ------------------------------------------------------------*/

#define CMD_SHELL_UART_PORT                      (1)
#define CMD_SHELL_UART_BAUD                      (115200)
#define CMD_SHELL_UART_DATA_BITS                 (8)
#define CMD_SHELL_UART_STOP_BITS                 (1)
#define CMD_SHELL_UART_PARITY                    (HAL_UART_PARITY_NONE)
#define CMD_SHELL_UART_FLOW_CTRL                 (HAL_UART_FLOW_CTL_NONE)

/* Private function prototypes -----------------------------------------------*/

static int cmd_shell_uart_tx_char(void *arg);
static int cmd_shell_uart_rx_char(void *arg, uint8_t data);

/* Private variables ---------------------------------------------------------*/

static app_fifo_t tx_fifo, rx_fifo;

/* Exported functions --------------------------------------------------------*/

void cmd_shell_thread(void * arg)
{
    int ret;
    uint8_t tx_buf[64], rx_buf[4], data;

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

    ret = app_fifo_init(&tx_fifo, tx_buf, sizeof(tx_buf));
    assert_param(0 == ret);

    ret = app_fifo_init(&rx_fifo, rx_buf, sizeof(rx_buf));
    assert_param(0 == ret);

    while (1) {
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

