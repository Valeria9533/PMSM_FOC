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
#include "PE_freemaster.h"
#include "serial.h"
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
#define ADC3_IN0_UDC_Pin GPIO_PIN_0
#define ADC3_IN0_UDC_GPIO_Port GPIOA
#define ADC1_IN1_IU_Pin GPIO_PIN_1
#define ADC1_IN1_IU_GPIO_Port GPIOA
#define ADC2_IN2_IV_Pin GPIO_PIN_2
#define ADC2_IN2_IV_GPIO_Port GPIOA
#define ADC3_IN3_IW_Pin GPIO_PIN_3
#define ADC3_IN3_IW_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define H1_Pin GPIO_PIN_6
#define H1_GPIO_Port GPIOC
#define H2_Pin GPIO_PIN_7
#define H2_GPIO_Port GPIOC
#define H3_Pin GPIO_PIN_8
#define H3_GPIO_Port GPIOC
#define POWER_OK_Pin GPIO_PIN_2
#define POWER_OK_GPIO_Port GPIOD
#define RUN_Pin GPIO_PIN_3
#define RUN_GPIO_Port GPIOD
#define ERROR_Pin GPIO_PIN_4
#define ERROR_GPIO_Port GPIOD
#define USART2_DIR_Pin GPIO_PIN_7
#define USART2_DIR_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define SPI2_NSS_SET	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET)
#define SPI2_NSS_CLR	HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
