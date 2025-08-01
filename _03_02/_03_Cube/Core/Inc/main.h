/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
extern volatile int analogMode;
extern volatile int lightBrightness;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_A1_Pin GPIO_PIN_1
#define BTN_A1_GPIO_Port GPIOA
#define BTN_A1_EXTI_IRQn EXTI1_IRQn
#define BTN_A2_Pin GPIO_PIN_4
#define BTN_A2_GPIO_Port GPIOA
#define BTN_A2_EXTI_IRQn EXTI4_IRQn
#define Overflow_LED_Pin GPIO_PIN_6
#define Overflow_LED_GPIO_Port GPIOA
#define BTN_A3_Pin GPIO_PIN_0
#define BTN_A3_GPIO_Port GPIOB
#define BTN_A3_EXTI_IRQn EXTI0_IRQn
#define CLOCK_Pin GPIO_PIN_8
#define CLOCK_GPIO_Port GPIOA
#define DATA_Pin GPIO_PIN_9
#define DATA_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Zoomer_Pin GPIO_PIN_3
#define Zoomer_GPIO_Port GPIOB
#define LATCH_Pin GPIO_PIN_5
#define LATCH_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
