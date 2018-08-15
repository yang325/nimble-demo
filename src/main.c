
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Private define ------------------------------------------------------------*/

/**< Size of the BLE application task. */
#define APP_TASK_BLE_APP_SIZE           (configMINIMAL_STACK_SIZE * 4)
/**< Priority of the BLE application task. */
#define APP_TASK_BLE_APP_PRIORITY       (configMAX_PRIORITIES - 2)
/**< Size of the BLE host task.*/
#define APP_TASK_BLE_HS_SIZE            (configMINIMAL_STACK_SIZE * 4)
/**< Priority of the BLE host task. */
#define APP_TASK_BLE_HS_PRIORITY        (configMAX_PRIORITIES - 1)

/* Private variables ---------------------------------------------------------*/

static uint32_t dev_uuid[4];

/* Private function prototypes -----------------------------------------------*/

static void system_clock_config(void);
static void system_info_output(void);

static void led_init(void);
static void led_handler(TimerHandle_t timer);

static void ble_app_on_sync(void);

static void ble_host_thread(void * arg);

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  system_clock_config();

  /* Output system information */
  system_info_output();

  /* Initialize LED */
  led_init();

  /* LED handler definition */
  if (NULL == xTimerCreate("led", 500 / portTICK_PERIOD_MS, pdTRUE, NULL, 
                            led_handler))
  {
    Error_Handler();
  }

  /* NimBLE host task definition */
  if (pdPASS != xTaskCreate(ble_host_thread, "host", APP_TASK_BLE_HS_SIZE,
                            NULL, APP_TASK_BLE_HS_PRIORITY, NULL)) 
  {
    Error_Handler();
  }

  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */
  Error_Handler();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void system_clock_config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / configTICK_RATE_HZ);

  /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @}
  */
static void system_info_output(void)
{
  uint8_t varient, revision;

  printf("\n");

  HAL_GetUID(&dev_uuid[0]);
  dev_uuid[3] = HAL_GetHalVersion();

  varient = (SCB->CPUID & SCB_CPUID_VARIANT_Msk) >> SCB_CPUID_VARIANT_Pos;
  revision = (SCB->CPUID & SCB_CPUID_REVISION_Msk) >> SCB_CPUID_REVISION_Pos;
  SystemCoreClockUpdate();

  printf("- ARM Cortex-M3 r%dp%d Core -\n", varient, revision);
  printf("- Core Frequency = %lu Hz -\n", SystemCoreClock);
  printf("- Device UUID = %08x:%08x:%08x:%08x -\n",
         dev_uuid[3], dev_uuid[2], dev_uuid[1], dev_uuid[0]);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void led_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void led_handler(TimerHandle_t timer)
{
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
}

/**
  * @brief  Handle BLE sync event
  * @param  None
  * @retval None
  */
static void ble_app_on_sync(void)
{
  printf("The host and controller are in sync\n");
}

/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent 
from on_ble_evt().
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (
context) from the
 *                    osThreadCreate() call to the thread.
 */
static void ble_host_thread(void * arg)
{
  ble_hci_uart_init();
  nimble_port_init();

  /* Initialize the NimBLE host configuration */
  ble_hs_cfg.sync_cb = ble_app_on_sync;

  nimble_port_run();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  printf("Error occurred: file %s on line %d\n", file, line);
  /* Generate breakpoint if debugger is connected */
  __BKPT(0);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  vTaskSuspendAll();
  printf("Wrong parameters value: file %s on line %lu\n", file, line);
  while(1)
  {
    HAL_Delay(100);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
  }
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */
int _write(int file, char *ptr, int len)
{
  int idx = 0;

  while(idx < len)
  {
    ITM_SendChar(ptr[idx++]);
  }

  return len;
}

/**
  * @}
  */
void __assert_func(const char *file, int line, const char *func, const char *condition)
{
  vTaskSuspendAll();
  printf("Wrong parameters value: file %s on line %d\n", file, line);
  while(1)
  {
    HAL_Delay(100);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
