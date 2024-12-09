/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 * D0 / D1 (PA2/3) used for USART2 for debugging / husart2
 * D2 (PA10) used for RX for GPS Module to get the current time for the system / huart1
 * D5 (PB4) GPS Power pin. Turn off to reset. (check if the module has a dedicated reset)
 * D13 used for onboard LED
 *
 * PB12 (CN10/Pin16) Flash SPI CS
 * PB13 (CN10/Pin30) Flash SPI CLK
 * PB14 (CN10/Pin28) Flash SPI MISO
 * PB15 (CN10/Pin26) Flash SPI MOSI
 *
 * PA8/D7 Test Pin
 *
 * PD2 (CN6/Pin4) LORA SPI NSS
 * PC10 (CN7/Pin1) LORA SPI CLK
 * PC11 (CN6/Pin2) LORA SPI MISO
 * PC12 (CN7/Pin3) LORA SPI MOSI
 * PA15 (CN7/Pin17) LORA RESET
 * PB5/D4 (CN9/Pin5) DIO0 External interrupt
 *
 * A1 (PA1) for Turbidity sensor
 * */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "buffer.h"
#include "lora_sx1276.h"
#include "gps.h"
#include "logger.h"
#include <time.h>
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart1;
USART_HandleTypeDef husart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for loggingTask */
osThreadId_t loggingTaskHandle;
const osThreadAttr_t loggingTask_attributes = {
  .name = "loggingTask",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for commProcessTask */
osThreadId_t commProcessTaskHandle;
const osThreadAttr_t commProcessTask_attributes = {
  .name = "commProcessTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for commandQueue */
osMessageQueueId_t commandQueueHandle;
const osMessageQueueAttr_t commandQueue_attributes = {
  .name = "commandQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);
void StartSensorTask(void *argument);
void StartLoggingTask(void *argument);
void StartCommandTask(void *argument);
void StartCommandProcessTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char debugBuffer[64] = {0};

uint8_t gps_rxBuffer[16] = {0};
volatile uint16_t gpsBytesInBuffer = 0;
volatile uint8_t startWaterSensing = 0;
volatile uint16_t waterSensorValue = 0;

lora_sx1276 lora;
uint8_t RXBuffer[64];
volatile uint8_t RXBufferStatus = 0; //1 is good and ready to read.

volatile uint8_t interruptTriggerDebug = 0; //For testing interrupts

//last recorded readings and data
uint16_t lastSensorS1Reading = 0;
logData_t lastLogLineRecorded;

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  //Add any pre-RTOS code here before it takes over
  HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, GPIO_PIN_SET);

  // SX1276 compatible module connected to SPI1, NSS pin connected to GPIO with label LORA_NSS
  uint8_t res = lora_init(&lora, &hspi3, SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, LORA_BASE_FREQUENCY_US);

  if (res != LORA_OK) {
  	DebugOutput("RFM95 init failed", res);
  } else {
	DebugOutput("RFM95 init success", res);
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &sensorQueue_attributes);

  /* creation of commandQueue */
  commandQueueHandle = osMessageQueueNew (16, 8, &commandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of loggingTask */
  loggingTaskHandle = osThreadNew(StartLoggingTask, NULL, &loggingTask_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(StartCommandTask, NULL, &commandTask_attributes);

  /* creation of commProcessTask */
  commProcessTaskHandle = osThreadNew(StartCommandProcessTask, NULL, &commProcessTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  DebugOutput("Warning! Should not reach here under FreeRTOS. ", 999);
	  HAL_Delay(3000);


    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OnboardLED_Pin|TestPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|GPS_Power_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_NSS_GPIO_Port, SPI3_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OnboardLED_Pin TestPin_Pin LORA_RESET_Pin */
  GPIO_InitStruct.Pin = OnboardLED_Pin|TestPin_Pin|LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS_Pin GPS_Power_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|GPS_Power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_EXTI5_Pin */
  GPIO_InitStruct.Pin = DIO0_EXTI5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_EXTI5_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Might need to consider:
//Before starting the next receive operation, make sure the DR register is empty (no data received on the UART interface after the last DR register read) to avoid UART overrun error:
//you can use this macro to flush the DR register: __HAL_UART_FLUSH_DRREGISTER()
//I notice with a reset on the power pin, this callback never recovers.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //call every 16 bytes
//	okToOutput = 1;
//	DebugOutput("HAL_UART_RxCpltCallback", 1);
	gpsBuffer_WriteStreamDataChunkToCircularBuffer(gps_rxBuffer);
	gpsBytesInBuffer += 16;

	if (gps_TimeSyncRequired(hrtc)){
		HAL_UART_Receive_IT(&huart1, gps_rxBuffer, 16);
	}

}

void ADC_IRQHandler(){ //do I need this? Test without it.
    HAL_ADC_IRQHandler(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	waterSensorValue = HAL_ADC_GetValue(&hadc1);
}

//Get time/date as string for logging.
void getSystemDateTimeFormatted(char* buffer){

	RTC_TimeTypeDef currentTime;
	RTC_DateTypeDef currentDate;
	HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

	sprintf(buffer, "%02d:%02d:%02d %02d/%02d/%02d",
		  currentTime.Hours, currentTime.Minutes, currentTime.Seconds,
		  currentDate.Date, currentDate.Month, currentDate.Year
	);
}

void DebugOutput(const char *message, int16_t number){
	//max: 93 message chars, 5 number chars, 2 lf/cr
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "%s%d\r\n", message, number);

	int length = strlen(buffer);
	HAL_USART_Transmit(&husart2, (uint8_t *)buffer, length, 10U);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == DIO0_EXTI5_Pin){
		interruptTriggerDebug = 1;

		if (RXBufferStatus == 1)
			return; //don't accept another packet until the first is taken and processed.

		uint8_t len = lora_receive_packet(&lora, RXBuffer, sizeof(RXBuffer), NULL); //no storing error info yet

		if (len > 0) {
			RXBufferStatus = 1;
			interruptTriggerDebug = len;
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  HAL_UART_Receive_IT(&huart1, gps_rxBuffer, 16);

  /* Infinite loop */
  for(;;)
  {
	DebugOutput("Running Default Task.", 1);

//	sprintf(debugBuffer, "gpsBytesInBuffer: %d\r\n", gpsBytesInBuffer);
//	HAL_USART_Transmit(&husart2, (uint8_t*)debugBuffer, strlen(debugBuffer), 10U);

	int gpsTimeSyncRequired = gps_TimeSyncRequired(hrtc);

	if (gpsTimeSyncRequired){

		if 	(gpsBytesInBuffer >= 128){
			char *cBuffer;
			cBuffer = gpsBuffer_GetCircularBuffer();
			HAL_USART_Transmit(&husart2, (uint8_t*)cBuffer, 128U, 100U);
			HAL_USART_Transmit(&husart2, (uint8_t*)"\r\n", 2U, 10U);
			DebugOutput("GPS", 1);

			gps_DownloadDateTimeViaSatellite(hrtc);

			gpsBytesInBuffer = 0;
		}

	} else {
//		RTC_HandleTypeDef rtc;
//		RTC_DateTypeDef currentDate;
//		HAL_RTC_GetDate(&rtc, &currentDate, RTC_FORMAT_BIN);

		DebugOutput("Timesync not required or timed out. Skipping.", 1);
		startWaterSensing = 1;
	}


	osDelay(3000);
	//DebugOutput("Finished Default Task.", 1);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  /* Infinite loop */

  for(;;)
  {
	//DebugOutput("StartSensorTask: ", 99);
	//DebugOutput("startWaterSensing --> : ", startWaterSensing);
	if (startWaterSensing == 1){
		//perform task, sense and push to queue

		//if there is an existing sense value, log it
//		if (waterSensorValue > 0){
			int16_t waterSensorValueForQueue = waterSensorValue;
			lastSensorS1Reading = waterSensorValue;
//			DebugOutput("WATER VALUE TO LOG -->: ", waterSensorValueForQueue);
			osStatus_t status = osMessageQueuePut(sensorQueueHandle, &waterSensorValueForQueue, 0, 0);
			DebugOutput("Water value sent to the queue: ", waterSensorValueForQueue);
			DebugOutput("Sensor value queuing result: ", status);
//		}


		//Start the sensor

		HAL_ADC_Start_IT(&hadc1);

	}
    osDelay(2000);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartLoggingTask */
/**
* @brief Only purpose is to monitor the logging queue and therefore log to SD card storage
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggingTask */
void StartLoggingTask(void *argument)
{
  /* USER CODE BEGIN StartLoggingTask */

  uint16_t receivedNumber = 0;

  /* Infinite loop */
  for(;;)
  {
	//DebugOutput("Logging from the queue: ", 1);

	if (startWaterSensing == 1){

		uint32_t queueCount = osMessageQueueGetCount(sensorQueueHandle);

		DebugOutput("SensorQueueCount: ", queueCount);

		if (queueCount > 0){
			osStatus_t status = osMessageQueueGet(sensorQueueHandle, &receivedNumber, NULL, 2);

			if (status == osOK){
//				waterSensorValue = 0; //clear value for next reading.

//				DebugOutput("Received...: ", receivedNumber);

				char timeDateBuffer[30];
				getSystemDateTimeFormatted(timeDateBuffer);

				logData_t data;
				strcpy(data.dateTime, timeDateBuffer);
				data.typeCode = 'W';
				data.value = receivedNumber;

				sprintf(debugBuffer, "Logging sensor data: %s, %c, %d\r\n", data.dateTime, data.typeCode, data.value);
				HAL_USART_Transmit(&husart2, (uint8_t*)debugBuffer, strlen(debugBuffer), 10U);

				logger_log(data);
				lastLogLineRecorded = data;

				//Light LED to indicate safe period to reset or unplug
				HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, GPIO_PIN_SET);
				DebugOutput("5 seconds to remove card.", 0);
				osDelay(5000);
				HAL_GPIO_WritePin(OnboardLED_GPIO_Port, OnboardLED_Pin, GPIO_PIN_RESET);

				//DebugOutput("End Log. Do not remove card.", 0);


				receivedNumber = 0;
			} else {
				DebugOutput("Error from queue...: ", status);
			}
		}


	}
    osDelay(1000);

  }
  /* USER CODE END StartLoggingTask */
}

/* USER CODE BEGIN Header_StartCommandTask */
/**
* @brief Function implementing the commandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandTask */
void StartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCommandTask */
  //lora_mode_receive_single(&lora);
  //lora_enable_interrupt_tx_done(&lora);
  lora_enable_interrupt_rx_done(&lora);
  lora_mode_receive_continuous(&lora); //if receive once, make sure this is set again

  /* Infinite loop */
  for(;;)
  {
//	  DebugOutput("Running Command Task.", 1);

	// Receive buffer
	if (RXBufferStatus == 1){
		DebugOutput("===========> INTERRUPT TRIGGERED LoRa Command Received RXBufferStatus: ", RXBufferStatus);

		HAL_USART_Transmit(&husart2, (uint8_t*)RXBuffer, 6U, 10U);
		HAL_USART_Transmit(&husart2, (uint8_t*)"\r\n", 2U, 10U);

		osStatus_t status = osMessageQueuePut(commandQueueHandle, &RXBuffer, 0, 0);
		if (status != osOK){
//			DebugOutput("Command FAILED to queue: ", status);
		} else {
//			DebugOutput("Command OK to queue: ", status);
		}

		memset(RXBuffer, '\0', sizeof(RXBuffer));
		RXBufferStatus = 0; //clear: ready for interrupt to accept the next LoRa packet.
		osDelay(200);
	} else { //lora rx text. No buffer yet

	}

//	if (interruptTriggerDebug > 0){
//		DebugOutput("------------ INTERRUPT TRIGGERED ------------", interruptTriggerDebug);
//		interruptTriggerDebug = 0;
//	}

	//DebugOutput("Ending Command Task.", 1);

    osDelay(1000);
  }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartCommandProcessTask */
/**
* @brief Function implementing the commProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandProcessTask */
void StartCommandProcessTask(void *argument)
{
  /* USER CODE BEGIN StartCommandProcessTask */
  /* Infinite loop */

  uint8_t gatewayResponseBuffer[64];

  for(;;)
  {
    uint8_t commandQueueBuffer[6] = {0};
    uint32_t queueCount = osMessageQueueGetCount(commandQueueHandle);

//	DebugOutput("CommandQueueCount: ", queueCount);

	if (queueCount > 0){
		osStatus_t status = osMessageQueueGet(commandQueueHandle, &commandQueueBuffer, NULL, 0);

		if (status == osOK){
			HAL_USART_Transmit(&husart2, commandQueueBuffer, 6U, 100U);
			DebugOutput(" ", 999);

			if (strcmp((const char *)commandQueueBuffer, "GET S1") == 0){
				DebugOutput("GET S1 COMPARED", 999);

				sprintf((char *)gatewayResponseBuffer, "Sensor S1 last reading was: %d", lastSensorS1Reading);
				uint8_t res = lora_send_packet_blocking(&lora, gatewayResponseBuffer, strlen((char*)gatewayResponseBuffer), 200U);
				if (res == LORA_OK) {
					DebugOutput("Successful send to the gateway", res);
				} else {
					DebugOutput("Unsuccessful send to the gateway", res);
				}

				lora_mode_receive_continuous(&lora);
			}

			if (strcmp((const char *)commandQueueBuffer, "GET L") == 0){
				DebugOutput("GET L COMPARED", 999);

				sprintf((char *)gatewayResponseBuffer, "LOG: %c | %d | %s ", lastLogLineRecorded.typeCode, lastLogLineRecorded.value, lastLogLineRecorded.dateTime);
				uint8_t res = lora_send_packet_blocking(&lora, gatewayResponseBuffer, strlen((char*)gatewayResponseBuffer), 200U);
				if (res == LORA_OK) {
					DebugOutput("Successful send to the gateway", res);
				} else {
					DebugOutput("Unsuccessful send to the gateway", res);
				}

				lora_mode_receive_continuous(&lora);
			}

			if (strcmp((const char *)commandQueueBuffer, "PING") == 0){
				DebugOutput("PING COMPARED", 999);

				uint8_t res = lora_send_packet_blocking(&lora, (uint8_t *)"PONG", 4U, 200U);
				if (res == LORA_OK) {
					HAL_USART_Transmit(&husart2, (uint8_t *)"Success ack from system\r\n", 25U, 100U);
				} else {
					HAL_USART_Transmit(&husart2, (uint8_t *)"Error ack from system\r\n", 23U, 100U);
				}

				lora_mode_receive_continuous(&lora);
			}

			if (strcmp((const char *)commandQueueBuffer, "RES G") == 0){
				DebugOutput("RES G COMPARED", 999);

				HAL_GPIO_WritePin(GPS_Power_GPIO_Port, GPS_Power_Pin, GPIO_PIN_RESET);
				osDelay(1000);
				HAL_GPIO_WritePin(GPS_Power_GPIO_Port, GPS_Power_Pin, GPIO_PIN_SET);
				gps_ResetSyncAttempts();
				HAL_UART_Receive_IT(&huart1, gps_rxBuffer, 16);

				uint8_t res = lora_send_packet_blocking(&lora, (uint8_t *)"GPS module reset", 16U, 200U);
				if (res == LORA_OK) {
					HAL_USART_Transmit(&husart2, (uint8_t *)"Success ack from system\r\n", 25U, 100U);
				} else {
					HAL_USART_Transmit(&husart2, (uint8_t *)"Error ack from system\r\n", 23U, 100U);
				}

				lora_mode_receive_continuous(&lora);
			}

		} else {
			DebugOutput("Error from command queue...: ", status);
		}
	}

	osDelay(4000);
  }
  /* USER CODE END StartCommandProcessTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
