/* Includes ------------------------------------------------------------------*/

#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include "bsp/bsp.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint32_t on_delay = 600, off_delay = 400;

/* Exported functions --------------------------------------------------------*/

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void led_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

  /* Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void led_on(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
  on_delay = 0;
  off_delay = 0;
}

void led_off(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  on_delay = 0;
  off_delay = 0;
}

void led_toggle(void)
{
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
}

bool led_state(void)
{
  return GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
}

void led_handler(void)
{
  if (on_delay && off_delay) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(on_delay);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(off_delay);
  }
}
