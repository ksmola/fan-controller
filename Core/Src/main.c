/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMPMAX 210
#define TEMPMIN 160

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef	sFilterConfig;
CAN_TxHeaderTypeDef	TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t				TxData[8];
uint8_t				RxData[8];
uint32_t			TxMailbox;


uint16_t msg_length = 0;
uint32_t irq_counter = 0;

uint32_t time_now = 0;
uint16_t temp_clt_f = 0;
uint16_t fan_speed_perc = 0;

/* Buffer used for reception */
//uint8_t aRxBuffer[RXBUFFERSIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Initialize();
void user_pwm_out(uint16_t speed);
//void Compose_CAN_msg(uint8_t offset, uint8_t mtype, uint8_t fromid, uint8_t toid, uint8_t table, uint8_t length, uint64_t data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//	  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_13);
//	  HAL_Delay(500);

	  user_pwm_out(fan_speed_perc);

//	  fan_speed_perc += 1;
//
//	  if (fan_speed_perc > 99)
//	  	  fan_speed_perc = 0;

	  // check if we started the turn-on-delay

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  sFilterConfig.FilterBank = 0; // 0..13 for CAN1, 14..27 for CAN2
  //sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.SlaveStartFilterBank = 14;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterActivation = ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  // now start the can peripheral
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
  	Error_Handler();
  }
  // activate CAN RX IRQ
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
  	Error_Handler();
  }
  // send test message
//  TxHeader.DLC = 4;
//  TxHeader.StdId = 0x123;
//  TxData[0] = 0x01;
//  TxData[1] = 0x02;
//  TxData[2] = 0x03;
//  TxData[3] = 0x04;
//  TxMailbox = 0;
//  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData, &TxMailbox) != HAL_OK)
//  {
//  	Error_Handler();
//  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SERIAL_GPIO_Port, SERIAL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_GPIO_Port, CAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SERIAL_Pin */
  GPIO_InitStruct.Pin = SERIAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERIAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_Pin */
  GPIO_InitStruct.Pin = CAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void Initialize()
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	fan_speed_perc = 0;
}

void user_pwm_out(uint16_t speed)
{
    TIM_OC_InitTypeDef sConfigOC;

    htim4.Instance->CCR1 = speed;
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = speed;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
}

uint16_t linear_interp_a(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x)
{
	uint16_t t;

    if (x <= x0) { return y0; }
    if (x >= x1) { return y1; }

    t =  (x-x0);
    t /= (x1-x0);

    return y0 + t*(y1-y0);
}

void Parse_CAN_data()
{
  switch (RxHeader.IDE)
  {
  case CAN_ID_STD:
    switch (RxHeader.StdId)
    {
    case 1520: // Group 0
      if ((RxData[6] << 8 | RxData[7]) >= 1000)
      {

      }
      break;

    case 1522:
    	temp_clt_f = (RxData[6] << 8 | RxData[7]);
    	fan_speed_perc = linear_interp_a(TEMPMIN, 0, TEMPMAX, 100, temp_clt_f);
    	break;

    default:
      break;
    }
    break;

  case CAN_ID_EXT:
    switch (RxHeader.ExtId)
    {
    case 33416: // request for AFR data
                //    if ((RxData[2] & 0xF) == 8)
//      Compose_CAN_msg((RxData[1] << 3 | RxData[2] >> 5), 2, 5, 0, (RxData[0] & 0x1F), (RxData[2] & 0xF), 1);
      break;

    default:
      break;
    }
    break;

  default:
    break;
  }

  //  switch (RxHeader.ExtId)
  //  {
  //  case 0x8308: // from id 0, to id 6, table 1, offset 0, mtype 1;  (MS requesting button status)
  //    // we got data request from MS
  //	  //Compose_CAN_msg();
  //  case
  //    break;
  //
  //  default:
  //    break;
  //  }
}

//void Compose_CAN_msg(uint8_t offset, uint8_t mtype, uint8_t fromid, uint8_t toid, uint8_t table, uint8_t length, uint64_t data)
//{
//  TxData[0] = AFR_to_ADC_counts(o2_sensor[0].AFR) >> 8;
//  TxData[1] = AFR_to_ADC_counts(o2_sensor[0].AFR) & 0xFF;
////  TxData[2] = AFR_to_ADC_counts(o2_sensor[1].AFR) >> 8;
////  TxData[3] = AFR_to_ADC_counts(o2_sensor[1].AFR) & 0xFF;
////  TxData[4] = AFR_to_ADC_counts(o2_sensor[2].AFR) >> 8;
////  TxData[5] = AFR_to_ADC_counts(o2_sensor[2].AFR) & 0xFF;
////  TxData[6] = AFR_to_ADC_counts(o2_sensor[3].AFR) >> 8;
////  TxData[7] = AFR_to_ADC_counts(o2_sensor[3].AFR) & 0xFF;
//   TxData[2] = o2_sensor[0].function_code >> 8;
//   TxData[3] = o2_sensor[0].function_code & 0xFF;
//   TxData[4] = o2_sensor[0].warmup >> 8;
//   TxData[5] = o2_sensor[0].warmup & 0xFF;
//   TxData[6] = o2_sensor[0].error_code >> 8;
//   TxData[7] = o2_sensor[0].error_code & 0xFF;
//  TxHeader.DLC = length;
//  TxMailbox = 0;
//  TxHeader.ExtId = (offset << 18) | (mtype << 15) | (fromid << 11) | (toid << 7) | (table << 3);
//  TxHeader.IDE = CAN_ID_EXT;
//  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, &TxData, &TxMailbox) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		Error_Handler();
	}
	else
	{
		HAL_GPIO_TogglePin(GPIOB, CAN_Pin);
		Parse_CAN_data();
		//got valid can data
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
