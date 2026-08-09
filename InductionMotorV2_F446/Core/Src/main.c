/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <arm_math.h>
#include <stdlib.h>
#include "Sbus.h"
#include "Encoder.h"
#include "SineWave.h"
#include "PID.h"
#include "string.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	Off=0,
	Forward=1,
	Reverse=2
}StateMachine;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Bench bring-up: run open loop at a fixed frequency, ignoring pot / button / PID.
//Set to 0 for normal operation.
#define BENCH_FORCE_RUN		0
#define BENCH_FREQUENCY		20	//Hz

//Bench: force Forward and skip the pot-zero interlock, so the pot ALONE gates the drive.
//Button is ignored while this is set. Set to 0 for normal operation.
#define BENCH_SKIP_INTERLOCK	1
#define BENCH_SKIP_RAMP			1	//bench: jump straight to the requested frequency, no ramp
#define FREQ_RAMP_MS			50	//ms per 1Hz step when ramping; 50 = 20Hz/s

#define CLOSED_LOOP_SPEED	0		//1 = PID on encoder feedback, 0 = open-loop V/F from pot
#define POT_DEADBAND		10.0f	//% of travel below which the drive stays off
#define POT_FILTER_ALPHA	0.2f	//one pole IIR at the 10ms tick, ~50ms time constant
#define BUTTON_DEBOUNCE_TICKS	3	//consecutive 10ms samples needed to accept a level
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
tsbus receivedSBUS;
uint32_t PreviousStepChangeTime=0;
uint32_t FrequencyChangeTime=0;
uint32_t ScreenUpdateTime=0;
int Step=1;
StateMachine State=Off;
StateMachine PreviousState=Reverse;	//so the first press gives Forward
encoder_data Encoder;
int32_t EncoderMeasureTime=0;
volatile int Enable=0;				//EXTI2
volatile int ToggleState=0;			//EXTI0
int UpdateState = 0;
uint32_t  RequestedFrequency = 0;
StateMachine Direction=0;
ST_SineWave SineWave;
volatile int FiftyMicroSecond=0;	//TIM10 update ISR
PID_Controller PID;
volatile uint16_t ADCRawValues[7];	//written by DMA2_Stream0
volatile uint8_t ADCReady=0;		//ADC conversion complete ISR
float Potentiameter=0;
float PotFiltered=0;
float MCUTemp=0;
float DriveTemp =0,Current_U =0,Current_V =0,Current_W =0,Current_N =0;
uint8_t PotZeroed =0;
volatile int Test=0;				//EXTI0
volatile uint32_t TestTime=0;		//EXTI0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM3){
		Encoder.EncoderValue = __HAL_TIM_GET_COUNTER(htim);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM10){
		FiftyMicroSecond=1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart == &huart2){
		ParseSBUS(&receivedSBUS);
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	ADCReady=1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //PC0 (button) is polled and debounced in the main loop, not handled here.
  if(GPIO_Pin== GPIO_PIN_2){
	  Enable=0;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  //  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  //  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  //  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  //  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    TIM1->BDTR|=1<<15;//Enables timer 1 outputs that are set in CCER and CCER register
    TIM1->CR1|=1<<0;//Enables the counting in timer 1
    TIM1->CCER|=(1<<0)|(1<<2)|(1<<4)|(1<<6)|(1<<8)|(1<<10);//Enable all 6 complementary outputs
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
    HAL_UART_Receive_DMA(&huart2, &receivedSBUS.ReceivedData[0], SBUS_LEN);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCRawValues, 7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

     CDC_Transmit_FS((uint8_t*)"Induction Driver V2.0\n", strlen("Induction Driver V2.0\n"));
     HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
     HAL_Delay(500);
     HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
     HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
     HAL_Delay(500);
     HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
     HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
     HAL_Delay(500);
     HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
     HAL_Delay(500);

     SineWave.WaveFrequency=MIN_FREQUENCY;
     SineWave.FrequencyA=MIN_FREQUENCY;
     SineWave.FrequencyB=MIN_FREQUENCY;
     SineWave.FrequencyC=MIN_FREQUENCY;

     PID.ControlMode=Velocity;
     PID.Kp=5;
     PID.Ki=0.1;
     PID.Kd=0;
     PID.dt=10;
     PID.integral=0;
     PID.min_output= 5;
     PID.max_output= 55;
     PID.min_Integral= 5;
     PID.max_Integral= 55;
     PID.output=10;

     HAL_GPIO_WritePin(ShutDown_GPIO_Port, ShutDown_Pin, 0);

  while (1)
  {
	  if(ADCReady==1){
	  		  ADCReady=0;
	  		  DriveTemp 	= ADCRawValues[0];
	  		  Current_U 	= ADCRawValues[1];
	  		  Current_V 	= ADCRawValues[2];
	  		  Current_W 	= ADCRawValues[3];
	  		  Current_N 	= ADCRawValues[4];
	  		  Potentiameter	= ADCRawValues[5] *100.0/4096.0;
	  		  MCUTemp		= ADCRawValues[6];
	  	  }
	  	  if (Potentiameter<5.0 && !PotZeroed) PotZeroed=1;
	  	  (PotZeroed==1)? (HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0)): (HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1));

	  	  if (PotZeroed==1 && Potentiameter >=4.0 ){
	  		  //RequestedFrequency= Potentiometer * MAX_FREQUENCY/100.0;
	  		  PID.target= (Potentiameter * MAX_FREQUENCY/100.0)*1735/60;
	  		  //if (State==Off) ToggleState=1;
	  	  }

	  	  //V/F for 208V 60Hz motor under test:
	  	  double Voltage = ( SineWave.WaveFrequency * (208.0/60.0)*5.0f) ;
	  	  if ( Voltage < 600) Voltage=600;
	  	  else if (Voltage >= 1000) Voltage = 1000;
	  	  SineWave.VoltageAmplitude= trunc(Voltage);
	  	  //Calculate RPM
	  	  //read every 10ms so *100*60 to be per minute
	  	  //1024*4 pulse / revolution on encoder
	  	  //Pulley ratio 20:50
	  	  //GetEncoderValue(&Encoder); 	//Obsolete since not using GPIO and using timer to capture encoder value
	  	  if ((HAL_GetTick()-EncoderMeasureTime)>=10){
	  		  Encoder.SpeedRPM=(Encoder.EncoderValue-Encoder.PreviousEncoderValue)*((60*100)*20)/(1024*4*50);
	  		  Encoder.PreviousEncoderValue=Encoder.EncoderValue;

	  		  //Speed setpoint
	  		  PotFiltered += (Potentiameter - PotFiltered) * POT_FILTER_ALPHA;
#if CLOSED_LOOP_SPEED
	  		  updatePID(&PID, (double)abs(Encoder.SpeedRPM));
	  		  RequestedFrequency=PID.output;
#else
	  		  //Open loop V/F: pot travel above the deadband maps to MIN..MAX_FREQUENCY
	  		  float PotSpan = (PotFiltered - POT_DEADBAND) / (100.0f - POT_DEADBAND);
	  		  if (PotSpan < 0.0f) PotSpan = 0.0f;
	  		  else if (PotSpan > 1.0f) PotSpan = 1.0f;
	  		  RequestedFrequency = MIN_FREQUENCY + (uint32_t)(PotSpan*(MAX_FREQUENCY-MIN_FREQUENCY) + 0.5f);
#endif
	  		  //Button: polled debounce. EXTI is unreliable here because R5 sits in series with
	  		  //SW3, so a press only pulls PC0 to ~2.6V through the internal pulldown.
	  		  static uint8_t BtnLast=0, BtnStable=0, BtnCount=0;
	  		  uint8_t BtnRaw = HAL_GPIO_ReadPin(PB1_INT_GPIO_Port, PB1_INT_Pin);
	  		  if (BtnRaw != BtnLast){ BtnLast=BtnRaw; BtnCount=0; }
	  		  else if (BtnCount < BUTTON_DEBOUNCE_TICKS) BtnCount++;
	  		  if (BtnCount >= BUTTON_DEBOUNCE_TICKS && BtnStable != BtnLast){
	  			  BtnStable = BtnLast;
	  			  if (BtnStable){ ToggleState=1; Test=1; TestTime=HAL_GetTick(); }	//press
	  		  }

	  		  //Report on USB CDC
	  		  static char msg[128];	//static: USB stack keeps the pointer after this scope exits
	  		  int len= snprintf(msg,sizeof(msg),"Pot=%5.1f ReqF=%3lu WaveF=%3lu Amp=%4lu RPM=%6ld St=%d\n",
	  				  PotFiltered, RequestedFrequency, SineWave.WaveFrequency, SineWave.VoltageAmplitude, Encoder.SpeedRPM, State);
	  		  if (len > (int)sizeof(msg)-1) len = sizeof(msg)-1;	//snprintf returns untruncated length
			  CDC_Transmit_FS((uint8_t*)msg, len);

	  		  EncoderMeasureTime= HAL_GetTick();
	  	  }
	  	  //enable/disable by push button: Off -> Forward -> Off -> Reverse -> Off ...
	  	  if (ToggleState){
	  		  if (State==Forward || State==Reverse) State=Off;
	  		  else if (State==Off && PreviousState==Reverse) State=PreviousState=Forward;
	  		  else if (State==Off && PreviousState==Forward) State=PreviousState=Reverse;
	  		  ToggleState=0;
	  	  }
	  	  if (HAL_GetTick()-TestTime>=100 && Test) Test=0;
#if BENCH_SKIP_INTERLOCK
	  	  PotZeroed = 1;
	  	  State = Forward;
#endif
	  	  //State Machine
	  	  switch(State){
	  	  	  case	Off:
	  	  		  Enable=0;
	  	  		  Direction=Off;
	  	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	  	  		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	  	  		  break;
	  	  	  case Forward:
	  	  		  Enable=1;
	  	  		  Direction=Forward;
	  	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
	  	  		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	  	  		  break;
	  	  	  case Reverse:
	  	  		  Enable=1;
	  	  		  Direction=Reverse;
	  	  		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	  	  		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	  	  		  break;
	  	  }
	  	  //Run motor if enabled
	  	  int RunDrive = (Enable && State!= Off && PotZeroed && PotFiltered > POT_DEADBAND && RequestedFrequency>=MIN_FREQUENCY);
#if BENCH_FORCE_RUN
	  	  RunDrive				= 1;
	  	  Direction				= Forward;
	  	  RequestedFrequency	= BENCH_FREQUENCY;	//equal to WaveFrequency so the ramp stays a no-op
	  	  SineWave.WaveFrequency= BENCH_FREQUENCY;
#endif
	  	  if(RunDrive){
	  		  //Generating Sinusoidal PWM
	  		  GenerateSine(&SineWave, &FiftyMicroSecond);
	  		  //Ramp Frequency
#if BENCH_SKIP_RAMP
	  		  SineWave.WaveFrequency = RequestedFrequency;
#else
	  		  if ((HAL_GetTick()-FrequencyChangeTime)>=FREQ_RAMP_MS && RequestedFrequency != SineWave.WaveFrequency){
	  			  if (RequestedFrequency > SineWave.WaveFrequency) SineWave.WaveFrequency++;
	  			  else if (RequestedFrequency < SineWave.WaveFrequency) SineWave.WaveFrequency--;
	  			  FrequencyChangeTime= HAL_GetTick();
	  		  }
#endif
	  	  }
	  	  //if not enabled then stop everything
	  	  else {
	  		  SineWave.PhaseA	=SineWave.PhaseB	=SineWave.PhaseC	=0;
	  		  TIM1->CCR1=TIM1->CCR2=TIM1->CCR3=PWM_MAX_VALUE/2;  // neutral, no net current
	  		  SineWave.WaveFrequency=MIN_FREQUENCY;
	  	  }
	  	  //send PWM values out (centered complementary SPWM)
	  	  if(Direction==Forward){
	  		  TIM1->CCR1 = (PWM_MAX_VALUE + SineWave.PhaseA) / 2;
	  		  TIM1->CCR2 = (PWM_MAX_VALUE + SineWave.PhaseB) / 2;
	  		  TIM1->CCR3 = (PWM_MAX_VALUE + SineWave.PhaseC) / 2;
	  	  }
	  	  else if (Direction==Reverse){
	  		  TIM1->CCR1 = (PWM_MAX_VALUE + SineWave.PhaseB) / 2;  // swap A<->B to reverse rotation
	  		  TIM1->CCR2 = (PWM_MAX_VALUE + SineWave.PhaseA) / 2;
	  		  TIM1->CCR3 = (PWM_MAX_VALUE + SineWave.PhaseC) / 2;
	  	  }
	  	  else{ TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = PWM_MAX_VALUE / 2; }  // neutral, no net current
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 7;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 150;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
  htim10.Init.Prescaler = 144-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 50-1;
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
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_RX;
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
  huart2.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ShutDown_Pin|LD1_Pin|LD2_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1_INT_Pin */
  GPIO_InitStruct.Pin = PB1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PB1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ShutDown_Pin LD1_Pin LD2_Pin LD3_Pin */
  GPIO_InitStruct.Pin = ShutDown_Pin|LD1_Pin|LD2_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DriveFault_INT_Pin */
  GPIO_InitStruct.Pin = DriveFault_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DriveFault_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
