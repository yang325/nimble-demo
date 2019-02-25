/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "cmd.h"
#include "bsp/bsp.h"

/* Exported functions --------------------------------------------------------*/

void cmd_shell_thread(void * arg)
{
    while (1) {
        led_toggle();
        vTaskDelay(500);
    }
}

