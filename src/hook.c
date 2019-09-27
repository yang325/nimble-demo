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
  error_handler("Application %s stack overflow\n", pcTaskName);
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
  while(1) {
    error_handler("Assert failed in %s, %s:%d (%s)", func, file, line, condition);
  }
}

/**
  * @}
  */
void error_handler(const char *fmt, ...)
{
  va_list ap;

  /* Disable IRQ */
  __disable_irq();
  /* Output information */
	va_start(ap, fmt);
	vprintk(fmt, ap);
	va_end(ap);

  /* Generate breakpoint if debugger is connected */
  __BKPT(0);
  /* Toggle the LED if no debugger connected */
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
