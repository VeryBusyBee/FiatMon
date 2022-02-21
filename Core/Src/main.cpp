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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "screens.h"
#include "can.h"
#include "obd.h"
#include "usart.h"
#include "sleep.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
size_t freemem;
bool testMode;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;


#define CANMSG_BUF_SIZE 16
uint8_t CANMsgQueueBuffer[ CANMSG_BUF_SIZE * sizeof( CANMsg_t ) ];
osMessageQId CANMsgQueueHandle;
osStaticMessageQDef_t CANMsgQueueControlBlock;

osThreadId ScreenTaskHandle;
osThreadId CANTaskHandle;

/* USER CODE BEGIN PV */
/* Stores the handle of the task that will be notified when the CAN message received. */
TaskHandle_t xCANTaskHandle = NULL;
TaskHandle_t xScrTaskHandle = NULL;

/* The index within the target task's array of task notifications to use. */
const UBaseType_t xArrayIndex = 1;

bool carBrake;
bool carConn = 1;
bool carIgnON;
bool carEngON;
bool carLightON;
int16_t carSpeed;	//integer part of speed value
int16_t carRPM;		//RPM value
float carFSpeed;	//full speed value - including fractional part
int16_t carTemp;
int16_t DispDBri = 0xb000;	//Day brightness
int16_t DispNBri = 0x6000;	//Night brightness
uint64_t carDate;
int32_t carEcon1;
int32_t carEcon2;
bool econMode;	//0 - LPH (Liters Per Hour), 1 - LPK (Liters Per 100 Km)

bool CANHacker_active;

uint16_t canQueueMax = 0;

extern CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
extern CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
extern uint32_t TxMailbox;
extern uint8_t rxData[16]; //declare a receive data buffer
extern CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

extern BitmapItem *noConnItem;//No CAN connection

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
//static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void StartScreenTask(void const * argument);
void StartCANTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SendDebugMsg(char *msg)
{
	uint16_t size;
	size = strlen(msg);
	while (huart1.gState != HAL_UART_STATE_READY){}
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, size);
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
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
//  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	SetBklit(DispDBri);

  UARTWait4Char();

  //  freemem=xPortGetFreeHeapSize();
  /* USER CODE END 2 */

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
  /* definition and creation of CANMsgQueue */
  osMessageQStaticDef(CANMsgQueue, CANMSG_BUF_SIZE, CANMsg_t, CANMsgQueueBuffer, &CANMsgQueueControlBlock);
  CANMsgQueueHandle = osMessageCreate(osMessageQ(CANMsgQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CANTask */
  osThreadDef(CANTask, StartCANTask, osPriorityNormal, 0, 128);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* definition and creation of ScreenTask */
  osThreadDef(ScreenTask, StartScreenTask, osPriorityNormal, 0, 128);
  ScreenTaskHandle = osThreadCreate(osThread(ScreenTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  //CAN mode depends on the jumper position
  if (HAL_GPIO_ReadPin(Norm_LB_GPIO_Port, Norm_LB_Pin) == 1) testMode = false;
  else testMode = true;

  if (testMode) hcan.Init.Mode = CAN_MODE_LOOPBACK;
  else hcan.Init.Mode = CAN_MODE_NORMAL;
//  hcan.Init.Mode = CAN_MODE_LOOPBACK;

  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    pHeader.DLC=1; //give message size of 1 byte
	pHeader.IDE=CAN_ID_EXT; //set identifier to extended
	pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
	pHeader.ExtId=CAR_SPEED_ID; //define a standard identifier, used for message identification by filters

	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //set filter mode
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x0000;
	sFilterConfig.FilterIdLow=0x0000;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterActivation=ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO1; //set fifo assignment
	sFilterConfig.FilterBank=1;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	HAL_CAN_Start(&hcan); //start CAN
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING); //enable interrupts

	/*
	//filter one (stack light blink)
	sFilterConfig.FilterBank=0;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=(((CAR_DATE_ID << 5)  | (CAR_DATE_ID >> (32 - 5))) & 0xFFFF); //Date
	sFilterConfig.FilterIdLow=((CAR_DATE_ID >> (11 - 3)) & 0xFFF8);// EXID[12:5] & 3
	sFilterConfig.FilterMaskIdHigh=(((CAR_SPEED_ID << 5)  | (CAR_SPEED_ID >> (32 - 5))) & 0xFFFF);//Speed
	sFilterConfig.FilterMaskIdLow=((CAR_SPEED_ID >> (11 - 3)) & 0xFFF8);
//	sFilterConfig.FilterMaskIdHigh=0x0000;
//	sFilterConfig.FilterMaskIdLow=0x0000;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; //set filter mode
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO1; //set fifo assignment
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	sFilterConfig.FilterBank=1;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=(((CAR_STAT1_ID << 5)  | (CAR_STAT1_ID >> (32 - 5))) & 0xFFFF); //the ID that the filter looks for
	sFilterConfig.FilterIdLow=((CAR_STAT1_ID >> (11 - 3)) & 0xFFF8);// EXID[12:5] & 3
	sFilterConfig.FilterMaskIdHigh=(((CAR_ECTEMP_ID << 5)  | (CAR_ECTEMP_ID >> (32 - 5))) & 0xFFFF);
	sFilterConfig.FilterMaskIdLow=((CAR_ECTEMP_ID >> (11 - 3)) & 0xFFF8);
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //set filter mode
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO1; //set fifo assignment
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	sFilterConfig.FilterBank=1;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=(((CAR_ECON_ID << 5)  | (CAR_ECON_ID >> (32 - 5))) & 0xFFFF); //the ID that the filter looks for
	sFilterConfig.FilterIdLow=((CAR_ECON_ID >> (11 - 3)) & 0xFFF8);// EXID[12:5] & 3
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; //set filter mode
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;

	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO1; //set fifo assignment
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig); //configure CAN filter

	HAL_CAN_Start(&hcan); //start CAN
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING); //enable interrupts
*/
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

	  /* USER CODE BEGIN TIM2_Init 0 */

	  /* USER CODE END TIM2_Init 0 */

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM2_Init 1 */

	  /* USER CODE END TIM2_Init 1 */
	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 36000-1;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = 250-1;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM2_Init 2 */

	  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
/*
static void MX_TIM3_Init(void)
{

*/  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

/*  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

*/  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
/*  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
*/  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
/*  HAL_TIM_MspPostInit(&htim3);

}
*/
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
  huart1.Init.BaudRate = 500000;
//  huart1.Init.BaudRate = 115200;
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
//  USART1->CR1 |= USART_CR1_RXNEIE;//enable RX interrupts
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DISP_RST_Pin|DISP_DC_Pin|DISP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOARD_LED_Pin */
  GPIO_InitStruct.Pin = BOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BOARD_LED_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DISP_RST_Pin DISP_DC_Pin DISP_CS_Pin */
  GPIO_InitStruct.Pin = DISP_RST_Pin|DISP_DC_Pin|DISP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Norm_LB_Pin */
  GPIO_InitStruct.Pin = Norm_LB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Norm_LB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TJA1050 S-Pin */
  GPIO_InitStruct.Pin = TJA_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TJA_S_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(TJA_S_GPIO_Port, TJA_S_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartScreenTask */
/**
  * @brief  Function implementing the ScreenTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartScreenTask */
void StartScreenTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	InitScreen();
	SwitchScreen(NORM_SCREEN);

//	uint8_t message[] = {0x0,0x3D,0x32,0x14,0x0,0x0,0x0,0x00};
//	uint32_t address = CAR_ECTEMP_ID;

    /* Store the handle of the calling task. */
	xScrTaskHandle = xTaskGetCurrentTaskHandle();
	/* Infinite loop */
  for(;;)
  {
	  vTaskSuspend( NULL );//wait for a time to refresh the screen

	  FormatItems();
	  UpdateScreen();

	  RequestOBD();

//	  HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void const * argument)
{
  /* USER CODE BEGIN StartCANTask */
	CANMsg_t CANMsg;
	portBASE_TYPE xStatus = pdFALSE;

    /* Store the handle of the calling task. */
	xCANTaskHandle = xTaskGetCurrentTaskHandle();

//	uint8_t message[] = {0x20,0x05,0x26,0x09,0x20,0x20};
//	uint32_t address = CAR_DATE_ID;

	uint8_t message[] = {0x5,0x62,0x19,0x42,0x3,0xDA};
	uint32_t address = CAR_ECON_ID;

//	uint8_t message[] = {0x20,0x00,0x48,0x00,0x00,0x18,0x20,0x00};
//	uint32_t address = CAR_STAT1_ID;
//	uint32_t address2 = CAR_STAT3_ID;

//	uint8_t message[] = {0x0,0x3D,0x32,0x8C,0x0,0x0,0x0,0x00};
//	uint32_t address = CAR_ECTEMP_ID;

//	uint8_t message[] = {0x01,0x75,0x01,0x23,0x16,0xFB,0x40,0x00};
//	uint32_t address = CAR_SPEED_ID;

//	carSpeed = 5;
//	carFSpeed = 5;
//	carTemp = 125;
//	carEcon1 = 675;
//	carBrake = 0;

//	osDelay(5);

	CAN_Send(message, sizeof(message), address);
//	  SendDebugMsg((char *)"Started\n");

//	CAN_Send(message2, sizeof(message2), address2);
//	xQueueReset(CANMsgQueueHandle); //empty the queue
/* Infinite loop */
  for(;;)
  {
	  if (testMode)
	  {
//		message[6] = 0x20;
//		message[2] = 0x48;
//		CAN_Send(message, sizeof(message), address);

//		message[0] = 0x80;
//		message[5] = 0x18;
//		CAN_Send(message, sizeof(message), address2);
	  }

	  xStatus = xQueueReceive(CANMsgQueueHandle, &CANMsg, 500);
	  if (xStatus == pdPASS) {
		  HandleOBDMsg(CANMsg.pRxHeader, CANMsg.rxData);
		  carConn = 1;

//		  if (CANMsg.pRxHeader.ExtId > 0x18D00000) Send2CANSniffer(CANMsg);

	  }
	  else carConn = 0;

	  UBaseType_t qSize = uxQueueMessagesWaiting(CANMsgQueueHandle);
	  if (qSize > canQueueMax) canQueueMax = qSize;

		//No CAN connection
	  if (carConn == 0)
	  {
		  noConnItem->clrOption(DISPLAY_HIDE);

		  GotoSleep();
		  ResumeFromSleep();
		  SystemClock_Config(); // restart system clock

	  }
	  else noConnItem->setOption(DISPLAY_HIDE);

  }
  /* USER CODE END StartCANTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
//	SendDebugMsg("Er\n");

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
