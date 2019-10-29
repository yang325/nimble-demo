#ifndef __CONSOLE_H__
#define __CONSOLE_H__

void console_init(void);
int  console_printf(const char *fmt, ...);
void error_handler(const char *fmt, ...);

#endif