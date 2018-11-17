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

#include "console/printk.h"
#include "stm32f1xx_hal.h"

static int console_output(int value)
{
    ITM_SendChar(value);
    return value;
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

    if (__printk_get_hook() != console_output) {
        __printk_hook_install(console_output);
    }

    va_start(args, fmt);
    len = vprintk(fmt, args);
    va_end(args);

    return len;
}

