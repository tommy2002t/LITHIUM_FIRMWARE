/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g0xx_hal_msp.c
  * @brief        MSP Initialization and De-Initialization code.
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


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */


/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */


  __HAL_RCC_SYSCFG_CLK_ENABLE();

  __HAL_RCC_PWR_CLK_ENABLE();


  /*
   * Disable the internal Pull-Up in Dead Battery pins
   * of UCPD peripheral.
   */
  HAL_SYSCFG_StrobeDBattpinsConfig(
          SYSCFG_CFGR1_UCPD1_STROBE |
          SYSCFG_CFGR1_UCPD2_STROBE);


  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}


/**
  * @brief FDCAN MSP Initialization
  * @param hfdcan FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspInit(
        FDCAN_HandleTypeDef *hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  if (hfdcan->Instance == FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspInit 0 */

    /* USER CODE END FDCAN1_MspInit 0 */


    /* --------------------------------------------------------
     * FDCAN peripheral clock
     * -------------------------------------------------------- */

    PeriphClkInit.PeriphClockSelection =
            RCC_PERIPHCLK_FDCAN;


    PeriphClkInit.FdcanClockSelection =
            RCC_FDCANCLKSOURCE_PLL;


    if (HAL_RCCEx_PeriphCLKConfig(
            &PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }


    __HAL_RCC_FDCAN_CLK_ENABLE();


    /* --------------------------------------------------------
     * GPIO
     *
     * PB8 -> FDCAN1_RX
     * PB9 -> FDCAN1_TX
     * -------------------------------------------------------- */

    __HAL_RCC_GPIOB_CLK_ENABLE();


    GPIO_InitStruct.Pin =
            FDCAN_RX_Pin |
            FDCAN_TX_Pin;


    GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;


    GPIO_InitStruct.Pull =
            GPIO_NOPULL;


    GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_LOW;


    GPIO_InitStruct.Alternate =
            GPIO_AF3_FDCAN1;


    HAL_GPIO_Init(
            GPIOB,
            &GPIO_InitStruct);


    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */
  }
}


/**
  * @brief FDCAN MSP De-Initialization
  * @param hfdcan FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspDeInit(
        FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance == FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

    /* USER CODE END FDCAN1_MspDeInit 0 */


    __HAL_RCC_FDCAN_CLK_DISABLE();


    HAL_GPIO_DeInit(
            GPIOB,
            FDCAN_RX_Pin |
            FDCAN_TX_Pin);


    /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

    /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}


/**
  * @brief I2C MSP Initialization
  * @param hi2c I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(
        I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  if (hi2c->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspInit 0 */

    /* USER CODE END I2C1_MspInit 0 */


    /* --------------------------------------------------------
     * I2C clock source
     * -------------------------------------------------------- */

    PeriphClkInit.PeriphClockSelection =
            RCC_PERIPHCLK_I2C1;


    PeriphClkInit.I2c1ClockSelection =
            RCC_I2C1CLKSOURCE_PCLK1;


    if (HAL_RCCEx_PeriphCLKConfig(
            &PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }


    /* --------------------------------------------------------
     * GPIO
     *
     * PB6 -> I2C1_SCL
     * PB7 -> I2C1_SDA
     * -------------------------------------------------------- */

    __HAL_RCC_GPIOB_CLK_ENABLE();


    GPIO_InitStruct.Pin =
            BQ_SCL_Pin |
            BQ_SDA_Pin;


    GPIO_InitStruct.Mode =
            GPIO_MODE_AF_OD;


    GPIO_InitStruct.Pull =
            GPIO_NOPULL;


    GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_LOW;


    GPIO_InitStruct.Alternate =
            GPIO_AF6_I2C1;


    HAL_GPIO_Init(
            GPIOB,
            &GPIO_InitStruct);


    /* --------------------------------------------------------
     * Peripheral clock
     * -------------------------------------------------------- */

    __HAL_RCC_I2C1_CLK_ENABLE();


    /* --------------------------------------------------------
     * I2C1 interrupt
     *
     * Required by:
     *
     * HAL_I2C_Mem_Read_IT()
     * HAL_I2C_Mem_Write_IT()
     *
     * STM32G0B1 uses a combined I2C1 interrupt vector.
     * -------------------------------------------------------- */

    HAL_NVIC_SetPriority(
            I2C1_IRQn,
            1U,
            0U);


    HAL_NVIC_EnableIRQ(
            I2C1_IRQn);


    /* USER CODE BEGIN I2C1_MspInit 1 */

    /* USER CODE END I2C1_MspInit 1 */
  }
}


/**
  * @brief I2C MSP De-Initialization
  * @param hi2c I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(
        I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspDeInit 0 */

    /* USER CODE END I2C1_MspDeInit 0 */


    /*
     * Disable interrupt before peripheral clock.
     */
    HAL_NVIC_DisableIRQ(
            I2C1_IRQn);


    __HAL_RCC_I2C1_CLK_DISABLE();


    HAL_GPIO_DeInit(
            BQ_SCL_GPIO_Port,
            BQ_SCL_Pin);


    HAL_GPIO_DeInit(
            BQ_SDA_GPIO_Port,
            BQ_SDA_Pin);


    /* USER CODE BEGIN I2C1_MspDeInit 1 */

    /* USER CODE END I2C1_MspDeInit 1 */
  }
}


/**
  * @brief SPI MSP Initialization
  * @param hspi SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(
        SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};


  if (hspi->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */


    /* --------------------------------------------------------
     * Peripheral clock
     * -------------------------------------------------------- */

    __HAL_RCC_SPI1_CLK_ENABLE();


    /* --------------------------------------------------------
     * GPIO
     *
     * PA5 -> SPI1_SCK
     * PA6 -> SPI1_MISO
     * PA7 -> SPI1_MOSI
     *
     * Chip-select remains software-controlled separately.
     * -------------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();


    GPIO_InitStruct.Pin =
            BQ2_SCLK_Pin |
            BQ2_MISO_Pin |
            BQ2_MOSI_Pin;


    GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;


    GPIO_InitStruct.Pull =
            GPIO_NOPULL;


    GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_LOW;


    GPIO_InitStruct.Alternate =
            GPIO_AF0_SPI1;


    HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct);


    /* --------------------------------------------------------
     * SPI1 interrupt
     *
     * Required by:
     *
     * HAL_SPI_TransmitReceive_IT()
     * -------------------------------------------------------- */

    HAL_NVIC_SetPriority(
            SPI1_IRQn,
            1U,
            0U);


    HAL_NVIC_EnableIRQ(
            SPI1_IRQn);


    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
}


/**
  * @brief SPI MSP De-Initialization
  * @param hspi SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(
        SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */


    HAL_NVIC_DisableIRQ(
            SPI1_IRQn);


    __HAL_RCC_SPI1_CLK_DISABLE();


    HAL_GPIO_DeInit(
            GPIOA,
            BQ2_SCLK_Pin |
            BQ2_MISO_Pin |
            BQ2_MOSI_Pin);


    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
