/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
DHT_DataTypedef DHT11_Data;
float T1 , T2 , H1 , H2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FAN_ON 1
#define FAN_OFF 0
#define e 2.718281828459

#define FAN_GPIO GPIOC
#define FAN_PIN GPIO_PIN_9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char serialOutBuffer[200];

uint8_t nowFanState = FAN_OFF;

uint64_t timerTick = 0;
void timerIRQcallback() {
//	uint16_t nowTimer =  __HAL_TIM_GET_COUNTER(&htim1);
//	sprintf(serialOutBuffer,"Timer interupted : nowTIme -> %d" , nowTimer);
//	HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
	timerTick++;

}

uint64_t getTickTimer1() {
	return (timerTick - 1) * 65536 + (uint64_t) __HAL_TIM_GET_COUNTER(&htim1);
}

uint64_t getTimer1Second() {
	return getTickTimer1() / 1000;
}

uint8_t fanState = FAN_OFF;
uint64_t lastFanOffTimeSecond = 0;
uint64_t lastFanOnTimeSecond = 0;

uint8_t switchToOn() {
	double triggerOnProbs = 1;
	uint64_t timediff = getTimer1Second() - lastFanOffTimeSecond;
	timediff = timediff > 300 ? 300 : timediff;
	float timeFromLastOff = timediff; // Second Max = 300 ??

	float insideRHNow = H2;
	float outsideRHNow = H1;
	float diffNowRH = insideRHNow - outsideRHNow;

	double probDiffNowRh = (double) (1 / (1 + pow(e, - 0.34 * diffNowRH + 1)) );
	double probTimeFromLastOff = (double) (1 / (1 + pow(e, - 0.1 * timeFromLastOff + 1)) );
	double probInsideRh = (double) (1 / (1 + pow(e, - 0.34 * (insideRHNow - 60) + 1)) );

	triggerOnProbs = probDiffNowRh * probTimeFromLastOff * probInsideRh;
	return triggerOnProbs > 0.6;
}

uint8_t switchToOff() {
	double triggerOffProbs = 1;
	uint64_t timediff = getTimer1Second() - lastFanOnTimeSecond;
	timediff = timediff > 300 ? 300 : timediff;
	float timeFromLastOn = timediff; // Second Max = 300 ??

	float insideRHNow = H2;
	float outsideRHNow = H1;
	float diffNowRH = insideRHNow - outsideRHNow;

	double probDiffNowRh = (double) (1 / (1 + pow(e, - 0.3 * diffNowRH + 0.3)) );

	double probTimeFromLastOff = (double) (1 / (1 + pow(e, - 0.2 * timeFromLastOn + 10)) );
	probTimeFromLastOff = 1 - probTimeFromLastOff;

	double probInsideRh = (double) (1 / (1 + pow(e, - 0.4 * (insideRHNow - 45) + 3.5)) );

	triggerOffProbs = probDiffNowRh * probTimeFromLastOff * probInsideRh;
	return triggerOffProbs < 0.5;
}

void setFan(uint8_t targetFanState) {
	if (targetFanState == nowFanState) return;

}

void fanControlLoop() {
//	float insideRH = 50;
//	float outsideRH = 60;
//	uint64_t timeInSecond = getTimer1Second();

	switch (fanState) {
		case FAN_ON :
			state_FanOn();
			break;
		case FAN_OFF :
			state_FanOff();
			break;
		default :
			break;
	}

}

void state_FanOn() {
	if (switchToOff() > 0) {
		lastFanOffTimeSecond = getTimer1Second();
		fanState = FAN_OFF;
		return;
	}

	  sprintf(serialOutBuffer,"FAN ON                                                                                                     \n" );
	  HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
	  HAL_GPIO_WritePin(FAN_GPIO, FAN_PIN, GPIO_PIN_RESET);

}

void state_FanOff() {
	if (switchToOn() > 0) {
		lastFanOnTimeSecond = getTimer1Second();
		fanState = FAN_ON;
		return;
	}
	  sprintf(serialOutBuffer,"FAN OFF                                                                                                      \n" );
	  HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
	  HAL_GPIO_WritePin(FAN_GPIO, FAN_PIN, GPIO_PIN_SET);
}

void tim1Delay(uint32_t mills) {
	uint64_t startTime = getTickTimer1();
	//mills *= 1000;
	while (getTickTimer1() - startTime < mills);
	return;
}

void fanSystemHandle() {
	  setDhtPin(GPIO_PIN_15);
	  DHT_GetData(&DHT11_Data);
	  T1 = DHT11_Data.Temperature;
	  H1 = DHT11_Data.Humidity;
	  //sprintf(serialOutBuffer,"Sensor 1 : Temperature : %.2f C , Humidity : %.2f %% \n",Temperature ,Humidity );
	  //HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 100 , 100 );

	  tim1Delay(100);
	  setDhtPin(GPIO_PIN_12);
	  DHT_GetData(&DHT11_Data);
	  T2 = DHT11_Data.Temperature;
	  H2 = DHT11_Data.Humidity + 4; // Shift up 4 %
	  //sprintf(serialOutBuffer,"Sensor 2 : Temperature : %.2f C , Humidity : %.2f %% \n",Temperature ,Humidity );
	  sprintf(serialOutBuffer,"Sensor 1 : Temperature : %.2f C , Humidity : %.2f %%    Sensor 2 : Temperature : %.2f C , Humidity : %.2f %% \n",T1 ,H1 , T2 , H2 );
	  HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
	  fanControlLoop();
	  tim1Delay(1000);
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
  // HAL_GPIO_WritePin(FAN_GPIO, FAN_PIN, GPIO_PIN_SET);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  setDhtPin(GPIO_PIN_15);
//	  DHT_GetData(&DHT11_Data);
//  	  T1 = DHT11_Data.Temperature;
//	  H1 = DHT11_Data.Humidity;
//	  //sprintf(serialOutBuffer,"Sensor 1 : Temperature : %.2f C , Humidity : %.2f %% \n",Temperature ,Humidity );
//	  //HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 100 , 100 );
//
//	  HAL_Delay(100);
//	  setDhtPin(GPIO_PIN_12);
//	  DHT_GetData(&DHT11_Data);
//  	  T2 = DHT11_Data.Temperature;
//	  H2 = DHT11_Data.Humidity + 4; // Shift up 4 %
//	  //sprintf(serialOutBuffer,"Sensor 2 : Temperature : %.2f C , Humidity : %.2f %% \n",Temperature ,Humidity );
//	  sprintf(serialOutBuffer,"Sensor 1 : Temperature : %.2f C , Humidity : %.2f %%    Sensor 2 : Temperature : %.2f C , Humidity : %.2f %% \n",T1 ,H1 , T2 , H2 );
//	  HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
//	  fanControlLoop();
//	  HAL_Delay(1000);


//	  uint64_t nowTimer = getTickTimer1();
//	  sprintf(serialOutBuffer,"nowTIme -> %d                                                                                                  \n" , nowTimer);
//	  HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
//	  HAL_Delay(100);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 40000 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
