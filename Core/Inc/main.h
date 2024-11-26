/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOC
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOC
#define IN3_Pin GPIO_PIN_2
#define IN3_GPIO_Port GPIOC
#define TM1652_CLK_Pin GPIO_PIN_0
#define TM1652_CLK_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_A0_Pin GPIO_PIN_4
#define LCD_A0_GPIO_Port GPIOC
#define LCD_RESET_Pin GPIO_PIN_5
#define LCD_RESET_GPIO_Port GPIOC
#define W25Qxx_CS_Pin GPIO_PIN_0
#define W25Qxx_CS_GPIO_Port GPIOB
#define KEY7_Pin GPIO_PIN_8
#define KEY7_GPIO_Port GPIOE
#define KEY6_Pin GPIO_PIN_9
#define KEY6_GPIO_Port GPIOE
#define KEY5_Pin GPIO_PIN_10
#define KEY5_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_11
#define KEY4_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_12
#define KEY3_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_14
#define KEY1_GPIO_Port GPIOE
#define RS485_EN_Pin GPIO_PIN_15
#define RS485_EN_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
