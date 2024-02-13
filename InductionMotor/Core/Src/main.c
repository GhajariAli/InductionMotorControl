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
#include <stdio.h>
#include <math.h>
#include "Sbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FREQUENCY 200
#define MIN_FREQUENCY 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
tsbus receivedSBUS;
uint32_t StepChangeTime=0;
uint32_t FrequencyChangeTime=0;
uint32_t ScreenUpdateTime=0;
int State=1;
int32_t EncoderValue=0;
int32_t EncoderMeasureTime=0;
int32_t PreviousEncoderValue=0;
double ActualSpeed=0;
int Enable=0;
int ToggleEnable=0;
int UpdateState = 0;
uint32_t  VoltageAmplitude = 1000;
uint32_t  Frequency = MIN_FREQUENCY;
uint32_t  RequestedFrequency = 40;
uint32_t  SPWM_Update = 0;
uint32_t  PWM_U = 0;
uint32_t  PWM_V = 0;
uint32_t  PWM_W = 0;
int Start_CH1 = 0;
int Start_CH2 = 0;
int Start_CH3 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim3){
		EncoderValue = __HAL_TIM_GET_COUNTER(htim);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM10){
		SPWM_Update=1;
		}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart6){
		ParseSBUS(&receivedSBUS);
	}
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_UART_Receive_DMA(&huart6, &receivedSBUS.ReceivedData[0], SBUS_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  State = 1;

  while (1)
  {
	  //enable/disable by push button
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) ToggleEnable=1;
	  while (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) && ToggleEnable){
		  if (Enable) Enable=0;
		  else Enable=1;
		  ToggleEnable=0;
	  }
	  if(Enable){
		  //Generating Sinusoidal PWM
		  if (SPWM_Update){
			  if (Start_CH1){
			  	PWM_U=trunc(sin((3*M_PI*Start_CH1*Frequency)/(10000))*VoltageAmplitude);
			  	Start_CH1=Start_CH1+1;
			  }
			  else PWM_U=0;
			  if (Start_CH2){
			    PWM_V=trunc(sin((3*M_PI*Start_CH2*Frequency)/(10000))*VoltageAmplitude);
			    Start_CH2=Start_CH2+1;
			  }
			  else PWM_V=0;
			  if (Start_CH3){
			    PWM_W=trunc(sin((3*M_PI*Start_CH3*Frequency)/(10000))*VoltageAmplitude);
			    Start_CH3=Start_CH3+1;
			  }
			  else PWM_W=0;
			  SPWM_Update=0;
		  }
		  //Calculate RPM
		  //read every 10ms so *100*60 to be per minute
		  //1024*4 pulse / revolution on encoder
		  //Pully ratio 20:50
		  if ((HAL_GetTick()-EncoderMeasureTime)>=10 ){
				  ActualSpeed=(EncoderValue-PreviousEncoderValue)*((60*100)*20)/(1024*4*50);
				  PreviousEncoderValue=EncoderValue;
				  EncoderMeasureTime= HAL_GetTick();
		  }
		  //Ramp Frequency
		  if ((RequestedFrequency > Frequency) && ((HAL_GetTick()-FrequencyChangeTime)>=200 )){
			  Frequency++;
			  FrequencyChangeTime= HAL_GetTick();
		  }
		  //Change State
		  if (Frequency != 0){
			  if ((HAL_GetTick() - StepChangeTime ) >= (1000/(Frequency*6))){
				  if(State<6){ State++; }
				  else { State=1; }
				  UpdateState=1;
				  StepChangeTime= HAL_GetTick();
			  }
		  }
		  if(Frequency >=MIN_FREQUENCY && Frequency <= MAX_FREQUENCY && UpdateState==1){
			  switch (State){
			  case 1:
				  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_RESET);
				  Start_CH2=0;
				  Start_CH3=0;
				  //1+4
				  Start_CH1=1;
				  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_SET);
				  UpdateState=0;
				  break;
			  case 2:
				  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_RESET);
				  Start_CH2=0;
				  Start_CH3=0;
				  //1+6
				  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_SET);
				  UpdateState=0;
				  break;
			  case 3:
				  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_RESET);
				  Start_CH1=0;
				  Start_CH3=0;
				  //3+6
				  Start_CH2=1;
				  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_SET);
				  UpdateState=0;
				  break;
			  case 4:
				  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_RESET);
				  Start_CH1=0;
				  Start_CH3=0;
				  //3+2
				  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_SET);
				  UpdateState=0;
				  break;
			  case 5:
				  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_RESET);
				  Start_CH1=0;
				  Start_CH2=0;
				  //5+2
				  Start_CH3=1;
				  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_SET);
				  UpdateState=0;
				  break;
			  case 6:
				  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_RESET);
				  Start_CH1=0;
				  Start_CH2=0;
				  //4+5
				  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_SET);
				  UpdateState=0;
				  break;
			  }
		  }
	  }
	  else {
		  Start_CH1=0;
		  Start_CH2=0;
		  Start_CH3=0;
		  PWM_U=0;
		  PWM_V=0;
		  PWM_W=0;
		  Frequency=MIN_FREQUENCY;
		  HAL_GPIO_WritePin(U_Lo_GPIO_Port, U_Lo_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(V_Lo_GPIO_Port, V_Lo_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(W_Lo_GPIO_Port, W_Lo_Pin, GPIO_PIN_RESET);
	  }

	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,PWM_U);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,PWM_V);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,PWM_W);
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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 100000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_2;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, U_Lo_Pin|V_Lo_Pin|W_Lo_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : U_Lo_Pin V_Lo_Pin W_Lo_Pin */
  GPIO_InitStruct.Pin = U_Lo_Pin|V_Lo_Pin|W_Lo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
