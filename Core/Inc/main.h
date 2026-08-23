/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D1_LED_Pin GPIO_PIN_1
#define D1_LED_GPIO_Port GPIOA
#define D2_LED_Pin GPIO_PIN_2
#define D2_LED_GPIO_Port GPIOA
#define Flash_CLK_Pin GPIO_PIN_3
#define Flash_CLK_GPIO_Port GPIOA
#define Flash_IO3_Pin GPIO_PIN_6
#define Flash_IO3_GPIO_Port GPIOA
#define Flash_IO2_Pin GPIO_PIN_7
#define Flash_IO2_GPIO_Port GPIOA
#define Flash_IO1_Pin GPIO_PIN_0
#define Flash_IO1_GPIO_Port GPIOB
#define Flash_IO0_Pin GPIO_PIN_1
#define Flash_IO0_GPIO_Port GPIOB
#define Flash_CS_Pin GPIO_PIN_10
#define Flash_CS_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_13
#define CLK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define MAG_CS_Pin GPIO_PIN_8
#define MAG_CS_GPIO_Port GPIOA
#define MAG_INT_Pin GPIO_PIN_9
#define MAG_INT_GPIO_Port GPIOA
#define ASM330_INT2_Pin GPIO_PIN_10
#define ASM330_INT2_GPIO_Port GPIOA
#define ASM330_CS_Pin GPIO_PIN_11
#define ASM330_CS_GPIO_Port GPIOA
#define ASM330_INT1_Pin GPIO_PIN_12
#define ASM330_INT1_GPIO_Port GPIOA
#define MS5611_CS_Pin GPIO_PIN_15
#define MS5611_CS_GPIO_Port GPIOA
#define ADXL_INT2_Pin GPIO_PIN_5
#define ADXL_INT2_GPIO_Port GPIOB
#define ADXL_INT1_Pin GPIO_PIN_6
#define ADXL_INT1_GPIO_Port GPIOB
#define ADXL_CS_Pin GPIO_PIN_7
#define ADXL_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
