
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define true 1
#define false 0

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include <string.h>

#define SD_CS_Pin GPIO_PIN_0
#define SD_CS_GPIO_Port GPIOB

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(char *file, int line);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
