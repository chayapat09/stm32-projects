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

#define RELAY_ON 1
#define RELAY_OFF 0
#define RELAY_GPIO GPIOA
#define RELAY_PIN GPIO_PIN_5

#define LED_WHITE 2
#define LED_NORMAL 1
#define LED_WATERING 0

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char serialOutBuffer[200]; // From temp code

char rxData[20];

uint8_t forceRelayOn_FLAG = 0;
uint8_t autoWatering_FLAG = 1;
uint8_t debugMode_FLAG = 0;
uint8_t lightOn_FLAG = 1;


void setLed(uint8_t lightingState) { // 1 == normal , 0 == watering
	if (!lightOn_FLAG) {
		// Lighting mode 'c'
		char x = 'c';
		HAL_UART_Transmit(&huart1,&x,1,100);
		return;
	}
	if (lightingState == LED_NORMAL) {
		// Lighting mode 'a'
		char x = 'a';
		HAL_UART_Transmit(&huart1,&x,1,100);
	}
	else if (lightingState == LED_WATERING) {
		// Lighting mode 'b'
		char x = 'b';
		HAL_UART_Transmit(&huart1,&x,1,100);
	}
	else {
		// Lighting mode 'c' -> white light
		char x = 'c';
		HAL_UART_Transmit(&huart1,&x,1,100);
	}
}

void setRelay(uint8_t relayState) {
	if (forceRelayOn_FLAG) {
		setLed(LED_NORMAL);
		HAL_GPIO_WritePin(RELAY_GPIO, RELAY_PIN, GPIO_PIN_SET);
		return;
	}
	if (autoWatering_FLAG) {
		if (relayState == RELAY_ON) {
			setLed(LED_WATERING);
			HAL_GPIO_WritePin(RELAY_GPIO, RELAY_PIN, GPIO_PIN_SET); // Lighting mode 'b'
		}
		else {
			setLed(LED_NORMAL);
			HAL_GPIO_WritePin(RELAY_GPIO, RELAY_PIN, GPIO_PIN_RESET); // Lighting mode 'a'
		}
	}
	else {
		setLed(LED_NORMAL);
		HAL_GPIO_WritePin(RELAY_GPIO, RELAY_PIN, GPIO_PIN_RESET); // Lighting mode 'a'
	}
}

double dp = 0;
double alpha = 0.9;
uint16_t timer_last_Dry;
uint16_t timer_StartRelay = 0;
uint8_t relayFlag = 0;
uint16_t PumpWorkingTime = 10000;
uint16_t lastProcessTime = 0;
void adcCallback(){

	if (__HAL_TIM_GET_COUNTER(&htim1) - lastProcessTime < 200) { // IRQ block other codes thus 100ms delay for each execute
		return;
	}
	lastProcessTime = __HAL_TIM_GET_COUNTER(&htim1);
	if (relayFlag){ // && __HAL_TIM_GET_COUNTER(&htim1) - timer_StartRelay < PumpWorkingTime) {
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		setRelay(RELAY_ON);
	}
	else {
		relayFlag = 0;
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		setRelay(RELAY_OFF);
	}

	uint32_t readValue = HAL_ADC_GetValue(&hadc1);



	// Some Dynamic Programming
	// f(n) = alpha* f(n-1) + nowValue * (1 - alpha);
	double smoothValue = dp * alpha + readValue * (1 - alpha);
	dp = smoothValue;
	//
	if (debugMode_FLAG){
		char out[50];
		for (int i = 0 ; i< 50 ; i++) out[i] = 0;

		sprintf(out,"Raw : %d",readValue);
		HAL_UART_Transmit(&huart2,(uint32_t *) out,50,100);

		sprintf(out," Smooth: %d\n\r",(int) smoothValue);
		HAL_UART_Transmit(&huart2,(uint32_t *) out,50,100);
	}

	// Mode 1 Force Pump to turn ON
	// Mode 2 Water when Moisure Value less than xxx  until yyy value
	// if 500 < val < 900 for 5 second .. Trigger pump until val < 300
	// Mode 3 Force pump to turn off
	// Make sure that measure value not A noise
	if (!relayFlag && ! (smoothValue > 500 && smoothValue < 900)) {
		timer_last_Dry = __HAL_TIM_GET_COUNTER(&htim1);
		relayFlag = 0;
	}
	else if (relayFlag && ! (smoothValue > 300 && smoothValue < 900)){
		timer_last_Dry = __HAL_TIM_GET_COUNTER(&htim1);
		relayFlag = 0;
	}
	else {
		// Dry Case
		uint16_t escapeTime = __HAL_TIM_GET_COUNTER(&htim1) - timer_last_Dry;
		// 1 timer tick = 0.001 second
		if (escapeTime > 5000) {// (5 Second) / (0.001 Second/Tick)
			relayFlag = 1;
			timer_StartRelay = __HAL_TIM_GET_COUNTER(&htim1);
		}
	}

}

// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------
// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------
// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------
// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------
// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------
// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------
// -----------------------  END PUMP-SOIL SYSTEM ---------------------------------------

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


void fanSystemHandle() {
	  setDhtPin(GPIO_PIN_15);
	  DHT_GetData(&DHT11_Data);
	  T1 = DHT11_Data.Temperature;
	  H1 = DHT11_Data.Humidity;
	  //sprintf(serialOutBuffer,"Sensor 1 : Temperature : %.2f C , Humidity : %.2f %% \n",Temperature ,Humidity );
	  //HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 100 , 100 );
	  HAL_Delay(100);
//	  delay(100000);
	  int i = 20;
	  setDhtPin(GPIO_PIN_12);
	  DHT_GetData(&DHT11_Data);
	  T2 = DHT11_Data.Temperature;
	  H2 = DHT11_Data.Humidity + 4; // Shift up 4 %
	  int x = 30;
	  //sprintf(serialOutBuffer,"Sensor 2 : Temperature : %.2f C , Humidity : %.2f %% \n",Temperature ,Humidity );
	  sprintf(serialOutBuffer,"Sensor 1 : Temperature : %.2f C , Humidity : %.2f %%    Sensor 2 : Temperature : %.2f C , Humidity : %.2f %% \n",T1 ,H1 , T2 , H2 );
	  HAL_UART_Transmit(&huart2 , (uint32_t *) serialOutBuffer , 200 , 100 );
	  fanControlLoop();
}




// -----------------------  END HUMUDITY-FAN SYSTEM ------------------------------------

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//if (rxData[0] == '1') debugMode_FLAG = 1;
	//else if (rxData[0] == '0') debugMode_FLAG = 0;
	char out[150];
	for (int i = 0 ; i < 150 ; i++) out[i] = 0;
	switch (rxData[0] - 'a') {
		case 0:
			forceRelayOn_FLAG = 1;
			break;
		case 1:
			forceRelayOn_FLAG = 0;
			break;
		case 2:
			autoWatering_FLAG = 1;
			break;
		case 3:
			autoWatering_FLAG = 0;
			break;
		case 4:
			debugMode_FLAG = 1;
			break;
		case 5:
			debugMode_FLAG = 0;
			break;
		case 6:
			lightOn_FLAG = 1;
			break;
		case 7:
			lightOn_FLAG = 0;
			break;
		default :
			break;
	}
	sprintf(out,"--------Information---------\n\rforcePumpOn (a,b) : %d\n\rautoWatering (c,d) : %d\n\rdebugMode_FLAG (e,f): %d\n\rlightOn (g,h) : %d\n\r",
			(int) forceRelayOn_FLAG,
			(int) autoWatering_FLAG,
			(int) debugMode_FLAG ,
			(int) lightOn_FLAG
			);
	HAL_UART_Transmit(&huart2,(uint32_t *) out,150,200);
	HAL_UART_Receive_IT(&huart2, rxData, 10);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart2, rxData, 10);
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Stop adc irq
	  HAL_ADC_Stop_IT(&hadc1);
	  fanSystemHandle();
	  HAL_ADC_Start_IT(&hadc1);
	  // send serial here //

	  //

	  HAL_Delay(1000);
	  // start adc irq

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Period = 65535;
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
  htim3.Init.Prescaler = 40000 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
