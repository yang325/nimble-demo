#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include "modlog/modlog.h"

#define console_printf(_fmt_, ...) \
    MODLOG_DEBUG(_fmt_, ##__VA_ARGS__)

#endif
