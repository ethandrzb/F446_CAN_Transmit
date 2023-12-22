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
#include <stdio.h>
#include "../../Drivers/ssd1306/ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define EXAMPLE_1
//#define EXAMPLE_2
//#define HEARTBEAT_EXAMPLE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;

uint8_t txData[8];
uint8_t rxData[8];

uint32_t txMailbox;

uint8_t expectedHeartbeatData = 0;

uint16_t angle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void updateManualValues();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Disable button interrupts
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

	// Start debounce timer
	HAL_TIM_Base_Start_IT(&htim6);

	static uint8_t servoID = 0;
	static uint8_t segID = 0x10;

	switch(GPIO_Pin)
	{
		case BTN_SERVO_ANGLE_DOWN_Pin:
			angle = (angle <= 0) ? 270: angle - 10;
			break;
		case BTN_SERVO_ANGLE_UP_Pin:
			angle += 10;
			angle %= 270;
			break;
		case BTN_SERVO_ID_DOWN_Pin:
			servoID = (servoID <= 0) ? 0x03 : servoID - 1;
			break;
		case BTN_SERVO_ID_UP_Pin:
			servoID = (servoID >= 0x03) ? 0x00 : servoID + 1;
			break;
		case BTN_SEG_ID_UP_Pin:
			segID += 0x10;
			break;
		case BTN_SEG_ID_DOWN_Pin:
			segID -= 0x10;
			break;
	}

	txData[0] = angle >> 8;
	txData[1] = angle & 0x00FF;

	txHeader.StdId = segID + servoID;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 2;

	HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	updateManualValues();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData);
	// Check header length
	// Will likely need to be converted to a switch statement as more commands are implemented
	if((rxHeader.DLC == 1) && (rxData[0] == expectedHeartbeatData))
	{
		// Heart beat received if data matches expected value
		HAL_GPIO_TogglePin(CAN_HEARTBEAT_LED_GPIO_Port, CAN_HEARTBEAT_LED_Pin);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim6)
	{
		// Re-enable button interrupts after debounce period
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

		HAL_TIM_Base_Stop_IT(&htim6);
	}
}

void updateManualValues()
{
	char buffer[11];

	uint16_t angleDecimal = (txData[0] << 8) + txData[1];

	sprintf(buffer, "%2X %2X  %3d", (unsigned int) txHeader.StdId >> 4, (unsigned int) txHeader.StdId & 0x0F, angleDecimal);
	ssd1306_SetCursor(9, 30);
	ssd1306_WriteString(buffer, Font_11x18, White);
	ssd1306_UpdateScreen();
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
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize SSD1306 display
  ssd1306_Init();
  ssd1306_Fill(Black);

  // Display number labels
  ssd1306_SetCursor(9, 10);
  ssd1306_WriteString("ID  M   A", Font_11x18, White);

  ssd1306_SetCursor(9, 30);
  ssd1306_WriteString("XX YY  ZZZ", Font_11x18, White);
  ssd1306_UpdateScreen();


  // Initialize CAN Tx header
  txHeader.DLC = 1;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x10;
  txHeader.ExtId = 0;
  txHeader.TransmitGlobalTime = DISABLE;

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef EXAMPLE_1
//	  Example 1
	for (uint8_t j = 0; j < 4; j++)
	{
		for(uint8_t i = 0x10; i <= 0x13; i++)
		{
			txHeader.StdId = i;
			txData[0] = anglesByte0[j];
			txData[1] = anglesByte1[j];

			txHeader.DLC = 2;
			HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			HAL_Delay(500);
		}
	}
#endif

#ifdef EXAMPLE_2
//	  Example 2
	for(uint8_t i = 3; i <= 8; i++)
	{
		txData[0] = i;
		txHeader.DLC = 1;
		HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		HAL_Delay(10000);
	}
#endif

#ifdef HEARTBEAT_EXAMPLE
	for(uint8_t id = 0x10; id <= 0x20; id += 0x10)
	{
		// Store expected response for comparison in RxFifo0Callback
		// Expected value should be the CAN ID of the node you want to check
		expectedHeartbeatData = id;

		txHeader.StdId = id;
		txHeader.DLC = 1;
		txHeader.RTR = CAN_RTR_REMOTE;

		HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		updateManualValues();

		HAL_Delay(1000);
	}
#endif
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 72;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canFilterConfig;

  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank = 0; // 18
  canFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;

  canFilterConfig.FilterIdHigh = 0x020 << 5;	// Can be any value if FilterMaskIDHigh is 0
  canFilterConfig.FilterIdLow = 0;
//  canFilterConfig.FilterMaskIdHigh = 0x7FC << 5;	// Filter by ID, ignoring 2 LSBs
  canFilterConfig.FilterMaskIdHigh = 0x000 << 5;	// Allow all IDs through filter (i.e., don't check any ID bits)
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.SlaveStartFilterBank = 20;

  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* USER CODE END CAN1_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 90-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 60000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|CAN_HEARTBEAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin CAN_HEARTBEAT_LED_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|CAN_HEARTBEAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_SEG_ID_DOWN_Pin */
  GPIO_InitStruct.Pin = BTN_SEG_ID_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_SEG_ID_DOWN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_SERVO_ID_UP_Pin BTN_SERVO_ID_DOWN_Pin BTN_SEG_ID_UP_Pin */
  GPIO_InitStruct.Pin = BTN_SERVO_ID_UP_Pin|BTN_SERVO_ID_DOWN_Pin|BTN_SEG_ID_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_SERVO_ANGLE_DOWN_Pin BTN_SERVO_ANGLE_UP_Pin */
  GPIO_InitStruct.Pin = BTN_SERVO_ANGLE_DOWN_Pin|BTN_SERVO_ANGLE_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
