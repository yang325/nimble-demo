/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

/* Private define ------------------------------------------------------------*/

/* Error code */
enum 
{
  FLASH_OK = 0,
  FLASH_ERROR,
};

/* Exported constants --------------------------------------------------------*/

/* Base address of the Flash sectors */
#define ADDR_FLASH_PAGE_254   ((uint32_t)0x0807F000) /* Base @ of Page 254, 2 Kbytes */
#define ADDR_FLASH_PAGE_255   ((uint32_t)0x0807F800) /* Base @ of Page 255, 2 Kbytes */

/* Exported functions --------------------------------------------------------*/

void flash_init(void);
int  flash_page_erase(uint32_t address);
int  flash_write(uint32_t destination, uint32_t *p_source, uint32_t length);
int  flash_write_protection_config(bool enable);
bool flash_write_protection_status(void);

#endif  /* __FLASH_IF_H */
