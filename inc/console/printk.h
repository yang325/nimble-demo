/* printk.h - low-level debug output */

/*
 * Copyright (c) 2010-2012, 2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _PRINTK_H_
#define _PRINTK_H_

#include <stddef.h>
#include <stdarg.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * @brief Print kernel debugging message.
 *
 * This routine prints a kernel debugging message to the system console.
 * Output is send immediately, without any mutual exclusion or buffering.
 *
 * A basic set of conversion specifier characters are supported:
 *   - signed decimal: \%d, \%i
 *   - unsigned decimal: \%u
 *   - unsigned hexadecimal: \%x (\%X is treated as \%x)
 *   - pointer: \%p
 *   - string: \%s
 *   - character: \%c
 *   - percent: \%\%
 *
 * No other conversion specification capabilities are supported, such as flags,
 * field width, precision, or length attributes.
 *
 * @param fmt Format string.
 * @param ... Optional list of format arguments.
 *
 * @return N/A
 */

extern int printk(const char *fmt, ...);
extern int vprintk(const char *fmt, va_list ap);
extern int snprintk(char *str, size_t size, const char *fmt, ...);
extern int vsnprintk(char *str, size_t size, const char *fmt, va_list ap);
extern void _vprintk(int (*out)(int, void *), void *ctx, const char *fmt, va_list ap);

extern void __printk_hook_install(int (*fn)(int));

#ifdef __cplusplus
}
#endif

#endif
