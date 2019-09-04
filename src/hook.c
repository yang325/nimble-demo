/* Includes ------------------------------------------------------------------*/

#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "bsp/bsp.h"
#include "console/printk.h"

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
  printk("Application %s stack overflow\n", pcTaskName);
  /* Generate breakpoint if debugger is connected */
  __BKPT(0);
  /* Disable IRQ */
  __disable_irq();
  while(1)
  {
    led_toggle();
    uint32_t delay = 500000;
    while (delay --)
    {
      __asm("nop");
    }
  }
}

void vApplicationTickHook(void)
{
  HAL_IncTick();
}

void vApplicationIdleHook(void)
{
  led_handler();
}

/**
  * @}
  */
void __assert_func(const char *file, int line, const char *func, const char *condition)
{
  printk("Assert failed in %s, %s:%d (%s)", func, file, line, condition);
  /* Generate breakpoint if debugger is connected */
  __BKPT(0);
  /* Disable IRQ */
  __disable_irq();
  while(1)
  {
    led_toggle();
    uint32_t delay = 500000;
    while (delay --)
    {
      __asm("nop");
    }
  }
}
