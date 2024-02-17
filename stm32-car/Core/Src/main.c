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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "rc522.h"
#include "lcd1602_i2c.h"
#include "string.h"
#include "stdlib.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

unsigned char CardID[5];    // bien luu gia tri ID cua the RFID
char Send_UART[28];
//unsigned char MyID[8][5] = {{0x73, 0x35, 0xB2, 0x06, 0xF2},{0x53,0xC0,0xE1,0xAD,0XDF},{0x03,0x2F,0x76,0x94,0XCE},{0x73,0x86,0x7A,0xA7,0X28},
//{0x53,0x31,0x31,0x95,0XC6},{0xC3,0x95,0x6D,0x94,0XAF},{0x33,0x8E,0x93,0xA7,0X89},{0x83,0x28,0x8D,0xA7,0X81}};	  // luu ID cac the duoc dinh danh
//uint8_t location[10] = {0};

//uint16_t Count_Close = 0, check_card = 0,Count_Open = 0;     // bien check_card xem ID the co hop le khong



char buffer1[16];   // luu gia tri gio phut giay
char buffer2[16];   // luu gia tri ngay thang nam
char buffer4[2];
uint8_t Setup_Time[7];


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_SIZE 24
uint8_t receive_data[RX_SIZE] = {0};    // luu gia tri gui den
uint8_t size = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DS3231_ADDRESS 0xD0

uint8_t TIME_CLOSE = 0;

uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

typedef struct{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME time;

void Get_Time (void)  // ham lay gia tri thoi gian tu DS3231
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c2, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
}

// ham setup gia tri ngay thang nam (fix cung)
void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)  
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c2, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);

}


uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);

void Set_Time_UART(char* str);
void RESET_CardID(void);
void Display_Time(void);
void Set_Barrier();
void Check_CardID_IN(void);
void Check_CardID_OUT(void);
void Check_CardID_OUT(void);
void SET_TIME_CLOSE(char* str);
void Cal_money(void);
void Check_Car_In(void);


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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

MFRC522_Init();

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
	LCD1602_Init(&hi2c1);
	LCD1602_SetFunction(BACKLIGHT_ON);
	
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_data, RX_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	
	HAL_UART_Receive_DMA(&huart2,receive_data,24);
	
	LCD1602_SetCursor(0, 0);
	LCD1602_SendString(" CHECK CARD ID");
	
	HAL_Delay(500);
	uwTick = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		Get_Time();
//		Display_Time();
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0){
			Set_Barrier();
			Check_CardID_IN();
			
		}
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 1){
			Set_Barrier();
			Check_CardID_OUT();
			
		}
		
//		Close_Barrier(TIME_CLOSE);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Check_CardID_IN(void)
{
	LCD1602_SetCursor(0, 1);
	LCD1602_SendString("CAR IN");
	if (MFRC522_Check(CardID) == MI_OK)
		{
			strcpy(Send_UART, "IN:");
			for(uint8_t i = 0; i<4; i++){
				sprintf(Send_UART + strlen(Send_UART),"%02x", CardID[i]);
			}
			for(int i = 0; i < 14; i++){
				if(Send_UART[i]>96 && Send_UART[i]<123){
					Send_UART[i]-=32;
				}
			}
			HAL_UART_Transmit_IT(&huart2,Send_UART,12);
			receive_data[0] = 0x00;
			Check_Car_In();
			RESET_CardID();
			HAL_Delay(2000);
		}			
}

void Check_CardID_OUT(void)
{
	LCD1602_SetCursor(0, 1);
	LCD1602_SendString("     CAR OUT    ");
	if (MFRC522_Check(CardID) == MI_OK)
		{
			strcpy(Send_UART, "OUT:");
			for(uint8_t i = 0; i<4; i++){
				sprintf(Send_UART + strlen(Send_UART),"%02x", CardID[i]);
			}
			for(int i = 0; i < 14; i++){
				if(Send_UART[i]>96 && Send_UART[i]<123){
					Send_UART[i]-=32;
				}
			}
			HAL_UART_Transmit_IT(&huart2,Send_UART,11);
			receive_data[0] = 0x00;
			Cal_money();
			RESET_CardID();
			HAL_Delay(2000);
		}
}

void Set_Time_UART(char* str)
{
	strncpy(buffer4, str+1, 2 );
	Setup_Time[6] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+4, 2 );
	Setup_Time[5] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+7, 2 );
	Setup_Time[4] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+10, 2 );
	Setup_Time[3] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+13, 2 );
	Setup_Time[2] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+16, 2 );
	Setup_Time[1] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+19, 2 );
	Setup_Time[0] = (uint8_t)atoi(buffer4);
	strncpy(buffer4, str+22, 2 );
	TIME_CLOSE = (uint8_t)atoi(buffer4);
	Set_Time(Setup_Time[0],Setup_Time[1],Setup_Time[2],Setup_Time[3],Setup_Time[4],Setup_Time[5],Setup_Time[6]);
	receive_data[0] = 0x00;

}



void Display_Time(void)
{
	sprintf(buffer1 ,"TIME: %02d:%02d:%02d", time.hour, time.minutes, time.seconds);
	LCD1602_SetCursor(0,1);
	LCD1602_SendString(buffer1);
		
//	sprintf(buffer2 ,"\nDATE: %02d-%02d-%02d\n", time.dayofmonth, time.month, time.year);
//	LCD1602_SetCursor(0,1);
//	LCD1602_SendString(buffer2+1);
}

void RESET_CardID(void)   // RESET cac phan tu trong mang CardID
{
	for(uint8_t i = 0; i < 5; i++)
	{
		CardID[i] = 0x00;
	}
}

void Set_Barrier()
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1){
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,200);
	}
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0){
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,200);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);
	}
}

void Cal_money(void)
{
	LCD1602_SetFunction(CLEAR_DISPLAY);
	LCD1602_SetCursor(0,0);
	LCD1602_SendString(" WAITTING....");
	uint16_t delayTime = 10;
	while(receive_data[0] == 0)
	{
		HAL_Delay(10);
		delayTime+=10;
		if(delayTime>7000){
			break;
		}
	}
	if(receive_data[0] =='M'){
		uint32_t money = 0;
		uint8_t i = 10, Time_IN[3] = {0};
		strncpy(buffer4, (char*)receive_data+1, 2 );
		Time_IN[0] = (uint8_t)atoi(buffer4);
		strncpy(buffer4, (char*)receive_data+4, 2 );
		Time_IN[1] = (uint8_t)atoi(buffer4);
		strncpy(buffer4, (char*)receive_data+7, 2 );
		Time_IN[2] = (uint8_t)atoi(buffer4);
	
		while(receive_data[i]!='E'){
			money = money*10 + (receive_data[i]-48);
			i++;
	
		}
		memset(buffer1,0x00,16);
		LCD1602_SetFunction(CLEAR_DISPLAY);
		sprintf(buffer1 ,"  IN:%02d:%02d:%02d", Time_IN[0], Time_IN[1], Time_IN[2]);
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);
		memset(buffer1,0x00,16);
		sprintf(buffer1 ,"TOTAL: %ld", money);
		LCD1602_SetCursor(0,1);
		LCD1602_SendString(buffer1);	
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == 1){
			HAL_Delay(20);
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);// mo barrier 2 cho xe ra
		HAL_Delay(5000); // thoi gian xe ra ngoai
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10) == 0||HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == 0){
			HAL_Delay(100);
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,200);//khong con xe thi dong barrier 2
		memset(receive_data,0x00,24);
	} else if (receive_data[0] =='N') {
		sprintf(buffer1 ," NOT FOUND CARD");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	} else if (receive_data[0] =='F') {
		sprintf(buffer1 ,"FORBIDDEN!");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	}  else if (receive_data[0] =='C') {
		sprintf(buffer1 ,"CONFLICT");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	}  else if (receive_data[0] =='U') {
		sprintf(buffer1 ,"UNAUTHORIZED");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	} else {
		sprintf(buffer1 ,"TIME OUT");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);
		sprintf(buffer1 ,"PLEASE RECHECK");
		LCD1602_SetCursor(0,1);
		LCD1602_SendString(buffer1);
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	}
	receive_data[0] = 0x00;
	LCD1602_SetFunction(CLEAR_DISPLAY);
	LCD1602_SetCursor(0, 0);
	LCD1602_SendString(" CHECK CARD ID");
}

void Check_Car_In(void){
	LCD1602_SetFunction(CLEAR_DISPLAY);
	LCD1602_SetCursor(0,0);
	LCD1602_SendString(" WAITTING....");
	
	uint16_t delayTime = 10;
	while(receive_data[0] == 0)
	{
		HAL_Delay(10);
		delayTime+=10;
		if(delayTime>7000){
			break;
		}
	}
	if(receive_data[0] =='L'){
		char location[5] = {0};
		for(uint8_t i = 9; i < 14; i++){
			location[i-9] = receive_data[i];
		}
		LCD1602_SetFunction(CLEAR_DISPLAY);
		LCD1602_SetCursor(2,0);
		LCD1602_SendString("LOCATION CAR");
		LCD1602_SetCursor(6,1);
		LCD1602_SendString(location);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,100);//mo barrier 1 cho xe vao
		HAL_Delay(5000);  // thoi gian xe di qua barrier
		while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12) == 0||HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11) == 0){
			HAL_Delay(100);
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,200);// khi khong con xe dong barrier 1
		memset(receive_data,0x00,24);
	} else if (receive_data[0] =='N') {
		sprintf(buffer1 ," NOT FOUND CARD");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	} else if (receive_data[0] =='F') {
		sprintf(buffer1 ,"FORBIDDEN!");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	}  else if (receive_data[0] =='C') {
		sprintf(buffer1 ,"CONFLICT");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	}  else if (receive_data[0] =='U') {
		sprintf(buffer1 ,"UNAUTHORIZED");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);	
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	} else {
		sprintf(buffer1 ,"TIME OUT");
		LCD1602_SetCursor(0,0);
		LCD1602_SendString(buffer1);
		sprintf(buffer1 ,"PLEASE RECHECK");
		LCD1602_SetCursor(0,1);
		LCD1602_SendString(buffer1);
		HAL_Delay(3000);
		memset(receive_data,0x00,24);
	}
	receive_data[0] = 0x00;
	LCD1602_SetFunction(CLEAR_DISPLAY);
	LCD1602_SetCursor(0, 0);
	LCD1602_SendString(" CHECK CARD ID");
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)    // thoi gian ngat la 100ms
//{
//	if(htim->Instance == htim2.Instance)
//	{
//		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == 0){
//			Count_Open = 1;
//			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);
//		}
//		if(Count_Close > 0)
//			Count_Close++;
//		if(Count_Open >0)
//			Count_Open++;
//		if(receive_data[0] =='T')
//		{
//			Set_Time_UART((char*)receive_data);
//		}
//	}
//	
//}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, receive_data, RX_SIZE);
    size = Size;
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
