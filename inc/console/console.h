#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include "modlog/modlog.h"

#define console_printf(_fmt_, ...) \
    MODLOG_DEBUG(0, _fmt_, ##__VA_ARGS__)

#endif
