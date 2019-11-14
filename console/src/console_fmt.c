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
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "bsp/bsp.h"
#include "console/printk.h"
#include "stm32f1xx_hal.h"

static SemaphoreHandle_t console_mutex = NULL;

static int console_output(int value)
{
    ITM_SendChar(value);
    return value;
}

void console_init(void)
{
    __printk_hook_install(console_output);

    console_mutex = xSemaphoreCreateMutex();
    assert(NULL != console_mutex);
}

/**
 * Prints the specified format string to the console.
 *
 * @return                      The number of characters that would have been
 *                                  printed if the console buffer were
 *                                  unlimited.  This return value is analogous
 *                                  to that of snprintf.
 */
int console_printf(const char *fmt, ...)
{
    va_list args;
    int len;

    xSemaphoreTake(console_mutex, portMAX_DELAY);

    va_start(args, fmt);
    len = vprintk(fmt, args);
    va_end(args);

    xSemaphoreGive(console_mutex);

    return len;
}

/**
  * @}
  */
void error_handler(const char *fmt, ...)
{
    va_list args;

    /* Disable IRQ */
    __disable_irq();
    /* Output information */
    va_start(args, fmt);
    vprintk(fmt, args);
    va_end(args);

    /* Generate breakpoint if debugger is connected */
    __BKPT(0);
    /* Toggle the LED if no debugger connected */
    while(1)
    {
        led_toggle();
        uint32_t delay = 500000;
        while (-- delay)
        {
            __asm("nop");
        }
    }
}
