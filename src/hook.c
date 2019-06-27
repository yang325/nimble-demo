/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

#include "console/console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "bsp/bsp.h"

/**
 * @brief Loop forever if stack overflow is detected.
 *
 * If configCHECK_FOR_STACK_OVERFLOW is set to 1,
 * this hook provides a location for applications to
 * define a response to a stack overflow.
 *
 * Use this hook to help identify that a stack overflow
 * has occurred.
 *
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
  console_printf("Application %s stack overflow\n", pcTaskName);
  assert(0);
}

void vApplicationTickHook(void)
{
  HAL_IncTick();
}

void vApplicationIdleHook(void)
{
  HAL_Delay(1000);
  led_toggle();
}

