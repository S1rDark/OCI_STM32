/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 
	    a
	  -----
  f|     |b
   |  g  |
    -----
  e|     |c
   |     |
    ----- .h
	    d
*/		
//		a b c d e f g h		invert
//	0 	1 1 1 1 1 1 0 0		0000 0011
//	1 	0 1 1 0 0 0 0 0		1001 1111
//	2	  1 1 0 1 1 0 1 0		0010 0101
//	3	  1 1 1 1 0 0 1 0		0000 1101
//	4 	0 1 1 0 0 1 1 0		1001 1001
//	5 	1 0 1 1 0 1 1 0		0100 1001
//	6  	1 0 1 1 1 1 1 0		0100 0001
//	7	  1 1 1 0 0 0 0 0		0001 1111
//	8 	1 1 1 1 1 1 1 0		0000 0001
//	9	  1 1 1 1 0 1 1 0		0000 1001

//                           0     1     2     3     4     5     6     7     8     9
const uint8_t DIGITS[] = {0x03, 0x9F, 0x25, 0x0D, 0x99, 0x49, 0x41, 0x1F, 0x01, 0x09};

//                             0     1     2     3
const uint8_t SEGMENTS[] = {0x10, 0x20, 0x40, 0x80};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ZoomerInc(void);
void ZoomerDec(void);
void ZoomerReset(void);
void ZoomerPlaySound(int);
void BeginDisplayWrite(void);
void EndDisplayWrite(void);
void WriteDataToRegister(uint8_t dataBits);
void WriteDataToSegment(uint8_t, uint8_t);
void WriteNumberToDisplay(uint32_t);
void setPWM(uint16_t pwmValue);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int zoomerValue = 0;
const int zoomerDefaultValue = 0;

const int BUTTON_LONG_CLICK_MILLISEC = 3000;
const int LONG_CLICK_UPDATE_MILLISEC = 1000;

volatile int Btn1Click = 0;
volatile int Btn1ClickTime = 0;
volatile int Btn1LongClick = 0;
volatile int Btn2Click = 0;
volatile int Btn2ClickTime = 0;
volatile int Overflow = 0;
volatile int Btn3Click = 0;
volatile int Btn3ClickTime = 0;
volatile int Btn3LongClick = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	int currentTime = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int const BUFFER_LEN = 256;
  char buffer[BUFFER_LEN];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		currentTime = HAL_GetTick();
		if (Btn1Click && currentTime - Btn1ClickTime > BUTTON_LONG_CLICK_MILLISEC)
		{
			Btn1LongClick = 1;
			Btn1ClickTime += BUTTON_LONG_CLICK_MILLISEC;
			ZoomerDec();
		}
		if (Btn1LongClick && currentTime - Btn1ClickTime > LONG_CLICK_UPDATE_MILLISEC)
		{
			Btn1ClickTime += LONG_CLICK_UPDATE_MILLISEC;
			ZoomerDec();
		}
		
		if (Btn3Click && currentTime - Btn3ClickTime > BUTTON_LONG_CLICK_MILLISEC)
		{
			Btn3LongClick = 1;
			Btn3ClickTime += BUTTON_LONG_CLICK_MILLISEC;
			ZoomerInc();
		}
		if (Btn3LongClick && currentTime - Btn3ClickTime > LONG_CLICK_UPDATE_MILLISEC)
		{
			Btn3ClickTime += LONG_CLICK_UPDATE_MILLISEC;
			ZoomerInc();
		}
		
		if (Overflow) 
		{
			HAL_GPIO_WritePin(GPIOA, Overflow_LED_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, Overflow_LED_Pin, GPIO_PIN_SET);			
		}
		
		snprintf(buffer, BUFFER_LEN, "X=%04d\n", (int)zoomerValue);
	  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
		ZoomerPlaySound(zoomerValue);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BeginDisplayWrite()
{
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);
}

void EndDisplayWrite()
{
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);
}

void WriteDataToRegister(uint8_t dataBits)
{
	for (int i = 0; i < 8; i++)
	{
		HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, dataBits % 2);
		dataBits >>= 1;
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, GPIO_PIN_RESET);
	}
}

void WriteDataToSegment(uint8_t dataBits, uint8_t segmentBits)
{
	BeginDisplayWrite();
	WriteDataToRegister(dataBits);
	WriteDataToRegister(segmentBits);
	EndDisplayWrite();
}

void WriteNumberToDisplay(uint32_t number)
{
	for (int i = 0; i < 4; i++) 
	{
		WriteDataToSegment(DIGITS[number % 10], SEGMENTS[i]);
		number /= 10;
	}
}

const int BUTTON_STEP = 1;
const int BUTTON_DEBOUNCE_MILLISEC = 150;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	int currentTime = HAL_GetTick();
	switch (GPIO_Pin)
	{
		case BTN_A1_Pin:
			if (HAL_GPIO_ReadPin(GPIOA, BTN_A1_Pin) == 0)
			{
				if (currentTime - Btn1ClickTime > BUTTON_DEBOUNCE_MILLISEC)
				{
					Btn1ClickTime = currentTime;
					Btn1Click = 1;
					Btn1LongClick = 0;
					ZoomerDec();
				}
			}
			else
			{
				Btn1Click = 0;
				Btn1LongClick = 0;
			}			
			break;
		case BTN_A2_Pin:
			if (HAL_GPIO_ReadPin(GPIOA, BTN_A2_Pin) == 0)
			{				
				if (currentTime - Btn2ClickTime > BUTTON_DEBOUNCE_MILLISEC)
				{
					ZoomerReset();
					Overflow = 1;	
				}
			}
			else
			{
				Btn2ClickTime = currentTime;
				Btn2Click = 1;
				Btn2Click = 0;
				Overflow = 0;
			}
			break;
		case BTN_A3_Pin:
			if (HAL_GPIO_ReadPin(GPIOB, BTN_A3_Pin) == 0)
			{
				if (currentTime - Btn3ClickTime > BUTTON_DEBOUNCE_MILLISEC)
				{
					Btn3ClickTime = currentTime;	
					Btn3Click = 1;
					Btn3LongClick = 0;
					ZoomerInc();
				}
			}
			else
			{
				Btn3Click = 0;
				Btn3LongClick = 0;
			}
			break;
	}
}

void ZoomerInc()
{				
	Overflow = 0;
	zoomerValue += BUTTON_STEP;
	if (zoomerValue > 254)
	{		
		ZoomerReset();
		Overflow = 1;
	}
}

void ZoomerDec()
{
	Overflow = 0;
	zoomerValue -= BUTTON_STEP;
	if (zoomerValue < 0) zoomerValue = 0;
}

void ZoomerReset()
{
	Overflow = 0;
	zoomerValue = zoomerDefaultValue;
}

void ZoomerPlaySound(int zoomerValue)
{
	HAL_GPIO_TogglePin(GPIOB, Zoomer_Pin);
	WriteNumberToDisplay(zoomerValue);
	HAL_Delay(zoomerValue);
	WriteNumberToDisplay(zoomerValue);
	HAL_GPIO_TogglePin(GPIOB, Zoomer_Pin);
	WriteNumberToDisplay(zoomerValue);
	HAL_Delay(2);	
	WriteNumberToDisplay(zoomerValue);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
