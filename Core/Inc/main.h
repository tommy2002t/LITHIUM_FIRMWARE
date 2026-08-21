/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define BQ_INT_Pin GPIO_PIN_0
#define BQ_INT_GPIO_Port GPIOA
#define BQ_INT_EXTI_IRQn EXTI0_1_IRQn
#define BQ_PG_Pin GPIO_PIN_1
#define BQ_PG_GPIO_Port GPIOA
#define BQ_PG_EXTI_IRQn EXTI0_1_IRQn
#define BQ_CE_Pin GPIO_PIN_2
#define BQ_CE_GPIO_Port GPIOA
#define CAN_STB_Pin GPIO_PIN_3
#define CAN_STB_GPIO_Port GPIOA
#define BQ2_ALERT_Pin GPIO_PIN_4
#define BQ2_ALERT_GPIO_Port GPIOA
#define BQ2_ALERT_EXTI_IRQn EXTI4_15_IRQn
#define BQ2_SCLK_Pin GPIO_PIN_5
#define BQ2_SCLK_GPIO_Port GPIOA
#define BQ2_MISO_Pin GPIO_PIN_6
#define BQ2_MISO_GPIO_Port GPIOA
#define BQ2_MOSI_Pin GPIO_PIN_7
#define BQ2_MOSI_GPIO_Port GPIOA
#define BQ_STAT1_Pin GPIO_PIN_0
#define BQ_STAT1_GPIO_Port GPIOB
#define BQ_STAT2_Pin GPIO_PIN_1
#define BQ_STAT2_GPIO_Port GPIOB
#define LED1_RED_Pin GPIO_PIN_8
#define LED1_RED_GPIO_Port GPIOA
#define LED2_GREEN_Pin GPIO_PIN_9
#define LED2_GREEN_GPIO_Port GPIOA
#define BQ2_CS_Pin GPIO_PIN_15
#define BQ2_CS_GPIO_Port GPIOA
#define BQ_SCL_Pin GPIO_PIN_6
#define BQ_SCL_GPIO_Port GPIOB
#define BQ_SDA_Pin GPIO_PIN_7
#define BQ_SDA_GPIO_Port GPIOB
#define FDCAN_RX_Pin GPIO_PIN_8
#define FDCAN_RX_GPIO_Port GPIOB
#define FDCAN_TX_Pin GPIO_PIN_9
#define FDCAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
