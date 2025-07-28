/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
const uint8_t SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0X80,0X90};
const uint8_t SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};
const int IRQ_BLOCK_TIME = 300;
volatile uint8_t flag_irq = 0;
volatile uint32_t time_irq = 0;

int isSetted = 0;
int alarmTimeConfiguration = 0;
int currentTimeConfiguration = 0;
int hourConfiguration = 0;
int minConfiguration = 0;
int isAlarming = 0;
int isAlarmOn = 0;
int isBlinking = 0;
RTC_TimeTypeDef alarmTime, currentTime;


struct timeType
{
	int hour;
	int minute;
}; //alarmTime, currentTime;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void resetLeds();
void writeNumberToSegment(uint8_t value, uint8_t segment, uint8_t MSBFIRST, uint8_t lightDot);
void getDigits(uint8_t value, uint8_t* digits);
void showTime(RTC_TimeTypeDef time);
void resetIRQ();
void setLedState();
void incHours();
void decHours();
void incMinutes();
void decMinutes();
IRQn_Type getButtonIRQ(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void writeNumberToSegment(uint8_t value, uint8_t segment, uint8_t MSBFIRST, uint8_t lightDot)
{
  HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_SET);
	uint8_t command = value > 9 ? 255 : SEGMENT_MAP[value];
	
	if (lightDot)
	{
		command &= 0x7F;
	}
	for (int i = 0; i < 8; i++)
	{
			 uint8_t output = 0;
			 if (MSBFIRST)
			 {
					 output = command & 128;
					 command = command << 1;
			 }
			 else
			 {
					 output = command & 1;
					 command = command >> 1;
			 }
			 
			 HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, output);
			 HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, 0);
			 HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, 1);
	}
	
	command = SEGMENT_SELECT[segment];
	for (int i = 0; i < 8; i++)
	{
			 uint8_t output = 0;
			 if (MSBFIRST)
			 {
					 output = command & 128;
					 command = command << 1;
			 }
			 else
			 {
					 output = command & 1;
					 command = command >> 1;
			 }
			 
			 HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, output);
			 HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, 0);
			 HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, 1);
	}

  HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_RESET);
}

void resetIRQ()
{
		if (flag_irq && (HAL_GetTick() - time_irq > IRQ_BLOCK_TIME))
		{
			__HAL_GPIO_EXTI_CLEAR_IT(BTN_A1_Pin);
			__HAL_GPIO_EXTI_CLEAR_IT(BTN_A2_Pin);
			__HAL_GPIO_EXTI_CLEAR_IT(BTN_A3_Pin);
			
			NVIC_ClearPendingIRQ(BTN_A1_EXTI_IRQn);
			NVIC_ClearPendingIRQ(BTN_A2_EXTI_IRQn);
			NVIC_ClearPendingIRQ(BTN_A3_EXTI_IRQn);
			
			HAL_NVIC_EnableIRQ(BTN_A1_EXTI_IRQn);
			HAL_NVIC_EnableIRQ(BTN_A2_EXTI_IRQn);
			HAL_NVIC_EnableIRQ(BTN_A3_EXTI_IRQn);
			
			flag_irq = 0;
		}
}

void setLedState()
{
	if (isAlarmOn)
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void setDefaultTime()
{
	currentTime.Hours = 12;
	currentTime.Minutes = 0;
	currentTime.Seconds = 0;
	alarmTime.Hours = -1;
	alarmTime.Minutes = -1;
	alarmTime.Seconds = 0;
}

void resetLeds()
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
}

void showTime(RTC_TimeTypeDef time)
{
	uint8_t hourDigits[2] = {10, 10};
	uint8_t minuteDigits[2] = {10, 10};
	
	if ((!hourConfiguration && !isAlarming) || isBlinking)
		getDigits(time.Hours, hourDigits);
	if ((!minConfiguration && !isAlarming) || isBlinking)
		getDigits(time.Minutes, minuteDigits); 
	
	writeNumberToSegment(hourDigits[1], 0, 1, 0);
	writeNumberToSegment(hourDigits[0], 1, 1, 1);
	writeNumberToSegment(minuteDigits[1], 2, 1, 0);
	writeNumberToSegment(minuteDigits[0], 3, 1, 0);
}

void getDigits(uint8_t value, uint8_t* digits)
{
	for (int i = 0; i < 2; i++)
	{
		digits[i] = value % 10;
		value /= 10;
	}
}

void incHours()
{
	uint8_t* target = (alarmTimeConfiguration) 
		? &alarmTime.Hours 
		: &currentTime.Hours; 
	
	if (*target == 23)
		*target = 0;
	else
		(*target)++;
}

void decHours()
{
	uint8_t* target = (alarmTimeConfiguration) 
		? &alarmTime.Hours 
		: &currentTime.Hours;  
	
	if (*target == 0)
		*target = 23;
	else
		(*target)--;
}

void incMinutes()
{
	uint8_t* target = (alarmTimeConfiguration)
		? &alarmTime.Minutes
		: &currentTime.Minutes;
	
	if (*target == 59)
	{
		*target = 0;
		incHours();
	}
	else
	{
		(*target)++;
	}
}

void decMinutes()
{
	uint8_t* target = (alarmTimeConfiguration)
		? &alarmTime.Minutes
		: &currentTime.Minutes;
	
	if (*target == 0)
	{
		*target = 59;
		decHours();
	}
	else
	{
		(*target)--;
	}
}

IRQn_Type getButtonIRQ(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case BTN_A1_Pin:
			return BTN_A1_EXTI_IRQn;
		case BTN_A2_Pin:
			return BTN_A2_EXTI_IRQn;
		case BTN_A3_Pin:
			return BTN_A3_EXTI_IRQn;
		default:
			return 0;
	}
}


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
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Stop(&htim1);
	setDefaultTime();
	HAL_RTC_SetTime(&hrtc, &currentTime, RTC_FORMAT_BCD);
	resetLeds();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		resetIRQ();
		
		if (isAlarmOn && !isAlarming)
		{
			if (currentTime.Hours == alarmTime.Hours && currentTime.Minutes == alarmTime.Minutes)
			{
				isAlarming = 1;
				HAL_TIM_Base_Start(&htim1);
			}
		}
		
		if (alarmTimeConfiguration)
		{
			showTime(alarmTime);
		} 
		else if (currentTimeConfiguration)
		{
			showTime(currentTime);
		}
		else
		{
			HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
			showTime(currentTime);
		}
		
		setLedState();
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Prescaler = 6399;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED4_Pin|LED3_Pin|LED2_Pin|SHCP_Pin
                          |DS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STCP_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN_A1_Pin BTN_A2_Pin */
  GPIO_InitStruct.Pin = BTN_A1_Pin|BTN_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin LED2_Pin SHCP_Pin
                           DS_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|LED2_Pin|SHCP_Pin
                          |DS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_A3_Pin */
  GPIO_InitStruct.Pin = BTN_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STCP_Pin LED1_Pin */
  GPIO_InitStruct.Pin = STCP_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_NVIC_DisableIRQ(getButtonIRQ(GPIO_Pin));
	
	if (hourConfiguration)
	{
		if (GPIO_Pin == BTN_A1_Pin)
		{
			hourConfiguration = 0;
			minConfiguration = 1;
		}
		else if (GPIO_Pin == BTN_A2_Pin)
		{
			decHours();
		}	
		else if (GPIO_Pin == BTN_A3_Pin)
		{
			incHours();
		}		
	}
	else if (minConfiguration)
	{
		if (GPIO_Pin == BTN_A1_Pin)
		{
			HAL_TIM_Base_Stop(&htim1);
			isBlinking = 0;
			minConfiguration = 0;
			if (alarmTimeConfiguration)
			{
				isSetted = 1;
				alarmTimeConfiguration = 0;
				isAlarmOn = 1;
			} 
			else if (currentTimeConfiguration)
			{
				currentTimeConfiguration = 0;
				HAL_RTC_SetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
			}
		}
		else if (GPIO_Pin == BTN_A2_Pin)
		{
			decMinutes();
		}	
		else if (GPIO_Pin == BTN_A3_Pin)
		{
			incMinutes();
		}	
	}
	else
	{
		if (GPIO_Pin == BTN_A1_Pin)
		{
			hourConfiguration = 1;
			currentTimeConfiguration = 1;
			HAL_TIM_Base_Start(&htim1);
		}
		else if (GPIO_Pin == BTN_A2_Pin)
		{
			alarmTimeConfiguration = 1;
			hourConfiguration = 1;
			HAL_TIM_Base_Start(&htim1);
			if (!isSetted)
			{
				alarmTime.Hours = currentTime.Hours;
				alarmTime.Minutes = currentTime.Minutes;
			}
		}
		else if (GPIO_Pin == BTN_A3_Pin)
		{
			if (isSetted)
				isAlarmOn = !isAlarmOn;
				if (isAlarming)
				{
					isAlarming = 0;
				}
		}
	}

	flag_irq = 1;
	time_irq = HAL_GetTick();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{
		isBlinking = !isBlinking;
	}
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
