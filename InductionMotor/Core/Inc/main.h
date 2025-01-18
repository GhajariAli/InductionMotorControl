/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PB1_Pin GPIO_PIN_0
#define PB1_GPIO_Port GPIOC
#define ShutDown_Pin GPIO_PIN_1
#define ShutDown_GPIO_Port GPIOC
#define DriveFault_Pin GPIO_PIN_2
#define DriveFault_GPIO_Port GPIOC
#define Potentiameter_Pin GPIO_PIN_3
#define Potentiameter_GPIO_Port GPIOC
#define Drive_Temp_Pin GPIO_PIN_1
#define Drive_Temp_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define I_U_Pin GPIO_PIN_6
#define I_U_GPIO_Port GPIOA
#define I_V_Pin GPIO_PIN_7
#define I_V_GPIO_Port GPIOA
#define I_W_Pin GPIO_PIN_0
#define I_W_GPIO_Port GPIOB
#define I_N_Pin GPIO_PIN_1
#define I_N_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_10
#define LD1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_11
#define LD2_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_12
#define LD3_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
