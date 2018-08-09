/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdlib.h>

#include "hal/hal_uart.h"
#include "hal/hal_gpio.h"
#include "bsp/bsp.h"
#include "stm32f1xx_hal.h"

#define UART_CNT           (sizeof(uarts) / sizeof(uarts[0]))

struct hal_uart {
    USART_TypeDef *u_regs;
    uint8_t u_open:1;
    uint8_t u_rx_stall:1;
    uint8_t u_tx_end:1;
    uint8_t u_rx_data;
    hal_uart_rx_char u_rx_func;
    hal_uart_tx_char u_tx_func;
    hal_uart_tx_done u_tx_done;
    void *u_func_arg;
};

struct hal_uart uarts[] = {
    {.u_regs = USART1}
};

struct hal_uart_irq {
    struct hal_uart *ui_uart;
    volatile uint32_t ui_cnt;
};


int hal_uart_init_cbs(int port, hal_uart_tx_char tx_func, hal_uart_tx_done tx_done,
                      hal_uart_rx_char rx_func, void *arg)
{
    struct hal_uart *u;

    u = &uarts[port];
    if (port >= UART_CNT || u->u_open) {
        return -1;
    }
    u->u_rx_func = rx_func;
    u->u_tx_func = tx_func;
    u->u_tx_done = tx_done;
    u->u_func_arg = arg;
    return 0;
}

void hal_uart_start_rx(int port)
{
    struct hal_uart *u;
    int sr;
    int rc;

    u = &uarts[port];
    if (u->u_rx_stall) {
        __HAL_DISABLE_INTERRUPTS(sr);
        rc = u->u_rx_func(u->u_func_arg, u->u_rx_data);
        if (rc == 0) {
            u->u_rx_stall = 0;
            u->u_regs->CR1 |= USART_CR1_RXNEIE;
        }
        __HAL_ENABLE_INTERRUPTS(sr);
    }
}

void hal_uart_start_tx(int port)
{
    struct hal_uart *u;
    int sr;

    u = &uarts[port];
    __HAL_DISABLE_INTERRUPTS(sr);
    u->u_regs->CR1 &= ~USART_CR1_TCIE;
    u->u_regs->CR1 |= USART_CR1_TXEIE;
    u->u_tx_end = 0;
    __HAL_ENABLE_INTERRUPTS(sr);
}

int hal_uart_config(int port, int32_t baudrate, uint8_t databits, uint8_t stopbits,
                    enum hal_uart_parity parity, enum hal_uart_flow_ctl flow_ctl)
{
    struct hal_uart *u;
    const struct stm32_uart_cfg *cfg;
    uint32_t cr1, cr2, cr3;

    if (port >= UART_CNT) {
        return -1;
    }

    u = &uarts[port];
    if (u->u_open) {
        return -1;
    }
    cfg = u->u_cfg;
    assert(cfg);

    

    switch (databits) {
    case 8:
        cr1 |= UART_WORDLENGTH_8B;
        break;
    case 9:
        cr1 |= UART_WORDLENGTH_9B;
        break;
    default:
        assert(0);
        return -1;
    }

    switch (stopbits) {
    case 1:
        cr2 |= UART_STOPBITS_1;
        break;
    case 2:
        cr2 |= UART_STOPBITS_2;
        break;
    default:
        return -1;
    }

    switch (parity) {
    case HAL_UART_PARITY_NONE:
        cr1 |= UART_PARITY_NONE;
        break;
    case HAL_UART_PARITY_ODD:
        cr1 |= UART_PARITY_ODD;
        break;
    case HAL_UART_PARITY_EVEN:
        cr1 |= UART_PARITY_EVEN;
        break;
    }

    switch (flow_ctl) {
    case HAL_UART_FLOW_CTL_NONE:
        cr3 |= UART_HWCONTROL_NONE;
        break;
    case HAL_UART_FLOW_CTL_RTS_CTS:
        cr3 |= UART_HWCONTROL_RTS_CTS;
        if (cfg->suc_pin_rts < 0 || cfg->suc_pin_cts < 0) {
            /*
             * Can't turn on HW flow control if pins to do that are not
             * defined.
             */
            assert(0);
            return -1;
        }
        break;
    }

    return 0;
}

int
hal_uart_close(int port)
{
    struct hal_uart *u;

    if (port >= UART_CNT) {
        return -1;
    }
    u = &uarts[port];

    u->u_open = 0;
    u->u_regs->CR1 = 0;

    return 0;
}
