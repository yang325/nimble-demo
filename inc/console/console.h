#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <stdio.h>

#define console_printf(_fmt_, ...)                 printf(_fmt_, ##__VA_ARGS__)

#endif
