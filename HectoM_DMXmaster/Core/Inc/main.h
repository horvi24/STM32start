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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>

#include "dbg.h"
#include "dmx_transmitter.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_CLK_MHZ 48
#define DMX_SLOT 44
#define DMX_BREAK 176
#define DMX_MBS 0
#define DMX_MAB 12
#define DBG_TX4_Pin GPIO_PIN_0
#define DBG_TX4_GPIO_Port GPIOA
#define DBG_RX4_Pin GPIO_PIN_1
#define DBG_RX4_GPIO_Port GPIOA
#define DMX_TX2_Pin GPIO_PIN_2
#define DMX_TX2_GPIO_Port GPIOA
#define DMX_TX_XXX_Pin GPIO_PIN_3
#define DMX_TX_XXX_GPIO_Port GPIOA
#define DMX_TX_BREAK_Pin GPIO_PIN_4
#define DMX_TX_BREAK_GPIO_Port GPIOA
#define DBG_OUT1_Pin GPIO_PIN_15
#define DBG_OUT1_GPIO_Port GPIOB
#define DMX_DE_Pin GPIO_PIN_12
#define DMX_DE_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOD
#define SW3_Pin GPIO_PIN_2
#define SW3_GPIO_Port GPIOD
#define SW4_Pin GPIO_PIN_3
#define SW4_GPIO_Port GPIOD
#define KEY_Pin GPIO_PIN_6
#define KEY_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */