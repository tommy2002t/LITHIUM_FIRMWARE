/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/* External variables --------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;


/* USER CODE BEGIN EV */

/* USER CODE END EV */


/******************************************************************************/
/* Cortex-M0+ Processor Interruption and Exception Handlers                    */
/******************************************************************************/


/**
  * @brief Non-maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */


  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  while (1)
  {
  }

  /* USER CODE END NonMaskableInt_IRQn 1 */
}


/**
  * @brief Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /*
   * Remove charging permission immediately if the CPU reaches
   * HardFault.
   *
   * We intentionally do NOT manipulate BATFET here because
   * doing so could kill avionics during flight.
   */
  HAL_GPIO_WritePin(
          BQ_CE_GPIO_Port,
          BQ_CE_Pin,
          GPIO_PIN_SET);


  /* USER CODE END HardFault_IRQn 0 */


  while (1)
  {
    /*
     * Do not refresh IWDG.
     *
     * If watchdog has been started, MCU will reset.
     */

    /* USER CODE BEGIN W1_HardFault_IRQn 0 */

    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}


/**
  * @brief System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */


  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}


/**
  * @brief Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */


  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}


/**
  * @brief System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */


  HAL_IncTick();


  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}


/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/


/**
  * @brief EXTI line 0 and line 1 interrupt.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */


  HAL_GPIO_EXTI_IRQHandler(
          BQ_INT_Pin);


  HAL_GPIO_EXTI_IRQHandler(
          BQ_PG_Pin);


  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}


/**
  * @brief EXTI line 4 to 15 interrupt.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */


  HAL_GPIO_EXTI_IRQHandler(
          BQ2_ALERT_Pin);


  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}


/**
  * @brief I2C1 combined interrupt.
  *
  * STM32G0B1 has one I2C1 IRQ vector.
  *
  * Both the HAL event handler and error handler are called
  * because asynchronous I2C transfers require normal event
  * processing as well as NACK / bus / arbitration /
  * overrun / timeout error handling.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */


  HAL_I2C_EV_IRQHandler(
          &hi2c1);


  HAL_I2C_ER_IRQHandler(
          &hi2c1);


  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}


/**
  * @brief SPI1 interrupt.
  *
  * HAL_SPI_IRQHandler() advances the asynchronous
  * HAL_SPI_TransmitReceive_IT() transaction.
  *
  * On completion HAL subsequently calls
  * HAL_SPI_TxRxCpltCallback(), where our Lithium application
  * raises only the BQ76942 transfer-complete flag.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */


  HAL_SPI_IRQHandler(
          &hspi1);


  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
