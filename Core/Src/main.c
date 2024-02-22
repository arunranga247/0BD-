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
#include "stdio.h"
#include "string.h"
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
FDCAN_HandleTypeDef hfdcan1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct OBD2{
	int Speed;
	int RPM;
	int temp;
	int MAF;
	int Odometer;
};
struct OBD2 OBD2_data = {0,0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t _RPM(uint8_t _PID);
uint8_t _RPM_11(uint8_t _PID);
uint8_t CAN_Protocal_detect(void);
uint8_t Wait_Tx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout);
int8_t Wait_Rx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];

int ing = 0;
uint8_t buf[150] = {0};
char buf1[150] = {0};
int pid_Response = 0;
int cal = 0;
uint8_t can_active_mode = 0;
uint8_t rchar = 50;
uint8_t ing_count = 0;
HAL_StatusTypeDef error_code;
FDCAN_ProtocolStatusTypeDef can_status;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs)
{
	if ( HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK){
		if(rchar == 49 || rchar == 50 )
		{
			if(RxData[1] == 0x41)
			{
				pid_Response = 1;
				ing = 1;
				if(RxData[2] == 0x0C){
					OBD2_data.RPM = ( RxData[4]|(RxData[3] << 8) )/4 ;
				}
				else if(RxData[2] == 0x0D){
					OBD2_data.Speed = RxData[3];
				}
				else if(RxData[2] == 0x10){
					OBD2_data.MAF = ( RxData[4]|(RxData[3] << 8) )/100 ;
				}
				else if(RxData[2] == 0x05){
					OBD2_data.temp = (RxData[3] - 40);
				}
				else if(RxData[2] == 0xA6){
					OBD2_data.Odometer = (0xFFFFFFFF & ( RxData[3] << 24) ) ;
					OBD2_data.Odometer = OBD2_data.Odometer |(0xFFFFFFFF & ( RxData[4] << 16) );
					OBD2_data.Odometer = OBD2_data.Odometer |(0xFFFFFFFF & ( RxData[5] << 8) );
					OBD2_data.Odometer = OBD2_data.Odometer|RxData[6];
				}

			}
			else if(RxData[0] == 0x00 && RxData[1] == 0x00)
			{
				if(cal == 0)
				{
					pid_Response = 1;
				}
			}
		}
		else{
			if(RxData[1] == 0x00)
			{
				if(RxData[2] == 0x00)
				{
//					sprintf((char *)buf,"0X%lX %02X %02X %02X %02X %02X %02X %02X %02X\n",RxHeader.Identifier ,RxData[0],RxData[1],RxData[2],RxData[3],RxData[4],RxData[5],RxData[6],RxData[7]);
//					HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen((char *)buf), 1000);
					if(cal == 0)
					{
						pid_Response = 1;
						can_active_mode = 0;
					}
				}
			}
		}
	}
}

uint32_t current_time = 0;

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
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*) "OBD2 G0B1 Test\n",strlen("OBD2 G0B1 Test\n"), 1000);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0 ); //enable interrupts
  HAL_FDCAN_Start(&hfdcan1); //start CAN
  HAL_UART_Receive_IT(&huart1, &rchar, 1);

  HAL_UART_Transmit(&huart1, (uint8_t*) "Protocol searching\n",strlen("Protocol searching\n"), 1000);
  current_time  = HAL_GetTick();
  rchar = CAN_Protocal_detect();
  sprintf((char *)buf,"Protocol detect %d\n",rchar);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen((char *)buf), 1000);
  if(rchar == 0)
  {
	  can_active_mode = 0;
  }
  else{
	  can_active_mode = 1;
  }
	  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if( HAL_GetTick() - current_time > 1000 )
	  {
		  current_time  = HAL_GetTick();
		  if(can_active_mode == 1){
			   if(rchar == 49)
				{
					pid_Response = 0;
					_RPM(0X0C);  // RPM
					HAL_Delay (10);
					_RPM(0X0D);	// SPEED
					HAL_Delay (10);
					_RPM(0X10);	// MAF
					HAL_Delay (10);
					_RPM(0X05);	// temp
					HAL_Delay (10);
					_RPM(0XA6);	// Odometer
				}
				else if(rchar == 50)
				{
					pid_Response = 0;
					_RPM_11(0X0C);  // RPM
					HAL_Delay (10);
					_RPM_11(0X0D);	// SPEED
					HAL_Delay (10);
					_RPM_11(0X10);	// MAF
					HAL_Delay (10);
					_RPM_11(0X05);	// temp
					HAL_Delay (10);
					_RPM_11(0XA6);	// Odometer
				}
				else{
					HAL_UART_Transmit(&huart1, (uint8_t*)"rec\n", strlen("rec\n"), 1000);
				}
			   if(pid_Response ==  0){
					ing_count++;
					if(ing_count > 10)
					{
						ing = 0;
						can_active_mode = 0;
						ing_count = 0;
					}
			   }
			HAL_FDCAN_GetProtocolStatus(&hfdcan1, &can_status);
			if( can_status.LastErrorCode == 3 ||can_status.Warning == 1 )
			{
				ing = 0;
			}
			sprintf((char *)buf,"%d %d %d RPM %d SPEED %d MAF %d Temp %d Odometer %f\n",ing_count,rchar ,ing , OBD2_data.RPM, OBD2_data.Speed, OBD2_data.MAF, OBD2_data.temp ,OBD2_data.Odometer/10.0);
			HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen((char *)buf), 1000);
		  }
		  else{
			  if(pid_Response == 1)
			  {
				  can_active_mode = 1;
				  rchar = CAN_Protocal_detect();
			  }
			  sprintf((char *)buf,"%d Img %d %d Car OFF\n",rchar, can_active_mode, ing);
			  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen((char *)buf), 1000);
		  }
		  if(rchar == 'r')
		  {
			  NVIC_SystemReset();
		  }
		  pid_Response = 0;
	   }
	  HAL_Delay(10);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 12;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 4;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef canfilterconfig;
  canfilterconfig.IdType = FDCAN_EXTENDED_ID;
  canfilterconfig.FilterIndex = 0;
  canfilterconfig.FilterType = FDCAN_FILTER_MASK;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  canfilterconfig.FilterID1 = 0X10;
  canfilterconfig.FilterID2 = 0X18DAF110;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &canfilterconfig);

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
  		Error_Handler();
  }
  if( HAL_FDCAN_EnableISOMode(&hfdcan1) == HAL_OK)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"****EnableISOMode\n", strlen("****EnableISOMode\n"), 1000);
  }
  else{
	  HAL_UART_Transmit(&huart1, (uint8_t*)"****Fail ISOMode\n", strlen("****Fail ISOMode\n"), 1000);
  }
  /* USER CODE END FDCAN1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t _RPM(uint8_t _PID)
{
	TxHeader.Identifier = 0x18DB33F1;   // ID
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // data length
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	TxHeader.MessageMarker = 0x52;

	TxData[0] = 0x02;
	TxData[1] = 0X01;
	TxData[2] = _PID;
	error_code = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	error_code = Wait_Tx_Timeout(&hfdcan1, 1000);
	if (error_code == 1)
	{
		return 1;
	}
	return 0;
}

uint8_t _RPM_11(uint8_t _PID)
{
	TxHeader.Identifier = 0x7DF;   // ID
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;  // data length
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	TxHeader.MessageMarker = 0x52;

	TxData[0] = 0x02;
	TxData[1] = 0X01;
	TxData[2] = _PID;
	error_code = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
	error_code = Wait_Tx_Timeout(&hfdcan1, 1000);
	if (error_code == 1)
	{
		return 1;
	}
	return 0;
}

uint8_t Wait_Tx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout)
{

	uint32_t aTickstart = 0; /* 10 */
	uint32_t aTotalTick = 0;
	aTickstart = HAL_GetTick(); /* Get tick */
	/* Check transmission occurred before timeout */
	while (HAL_FDCAN_IsTxBufferMessagePending(hfdcan, FDCAN_TX_BUFFER0) != 0) {
		aTotalTick = HAL_GetTick() - aTickstart;
		if (aTotalTick > set_timeout) {
			HAL_UART_Transmit(&huart1, (uint8_t*) "Tx Buf Msg Pnd-> timeout\n",strlen("Tx Buf Msg Pnd-> timeout\n"), 1000);
			return 0;
		}
	}
//	sprintf((char *)buf,"Tx Buf Msg Pnd->time take %d\n",(int)aTotalTick);
//	HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen((char *)buf), 1000);
	return 1;
}

int8_t Wait_Rx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout)
{
	uint32_t aTickstart = 0; /* 10 */
	uint32_t aTotalTick = 0;
	aTickstart = HAL_GetTick(); /* Get tick */
	/* Check one message is received in Rx FIFO 0 */
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) < 1) {
		aTotalTick = HAL_GetTick() - aTickstart;
		if (aTotalTick > set_timeout) {
			HAL_UART_Transmit(&huart1, (uint8_t*) "Rx Fifo Fill Lev-> timeout\n", strlen("Rx Fifo Fill Lev-> timeout\n"), 1000);
			return 0;
		}
	}
	sprintf((char *)buf,"Rx Fifo Fill Lev-> time take %d\n",(int)aTotalTick);
	HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen((char *)buf), 1000);
	return 1;
}

uint8_t CAN_Protocal_detect(void){
	cal = 1;
	pid_Response = 0;
	int i = 0;
	rchar = 49;
	HAL_Delay (1000);
	for(i = 0; i<5; i++)
	{
		_RPM(0X0C);
		 HAL_Delay (1000);
		 if(pid_Response ==  1){
			 cal = 0;
			 return 49;
		 }
	}
	rchar = 50;
	for(i = 0; i<5; i++)
	{
		_RPM_11(0X0C);
		 HAL_Delay (1000);
		 if(pid_Response ==  1){
			 cal = 0;
			 return 50;
		 }
	}
	return 0;
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
