/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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


/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lithium_app.h"
#include "lithium_config.h"
#include "lithium_led.h"
#include "lithium_can.h"

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * STM32G0 Independent Watchdog.
 *
 * LSI nominal:
 * approximately 32 kHz.
 *
 * Prescaler:
 * /32
 *
 * Reload:
 * 999
 *
 * Nominal timeout:
 *
 * (999 + 1) * 32 / 32000
 * ~= 1.0 s
 *
 * Exact timeout varies with LSI tolerance.
 */
#define LITHIUM_IWDG_PRESCALER_REGISTER_VALUE      3U
#define LITHIUM_IWDG_RELOAD_VALUE                  999U

#define LITHIUM_IWDG_START_KEY                     0xCCCCU
#define LITHIUM_IWDG_WRITE_ACCESS_KEY              0x5555U
#define LITHIUM_IWDG_REFRESH_KEY                   0xAAAAU

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;


/* USER CODE BEGIN PV */

static uint32_t last_system_task_ms = 0U;

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);


/* USER CODE BEGIN PFP */

static void Lithium_IWDG_Init(void);

static void Lithium_IWDG_Refresh(void);

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* ============================================================
 * STM32 INDEPENDENT WATCHDOG
 * ============================================================ */

static void Lithium_IWDG_Init(void)
{
    /*
     * IWDG uses the independent LSI oscillator.
     *
     * Enable LSI explicitly and wait until it is ready before
     * starting the watchdog.
     */
    RCC->CSR |=
            RCC_CSR_LSION;


    while ((RCC->CSR &
            RCC_CSR_LSIRDY) == 0U)
    {
        /*
         * Startup-only wait.
         *
         * Watchdog has not yet been started.
         */
    }


    /*
     * Start IWDG.
     */
    IWDG->KR =
            LITHIUM_IWDG_START_KEY;


    /*
     * Enable PR / RLR writes.
     */
    IWDG->KR =
            LITHIUM_IWDG_WRITE_ACCESS_KEY;


    /*
     * Prescaler /32.
     */
    IWDG->PR =
            LITHIUM_IWDG_PRESCALER_REGISTER_VALUE;


    /*
     * Approximately 1-second nominal timeout.
     */
    IWDG->RLR =
            LITHIUM_IWDG_RELOAD_VALUE;


    /*
     * Wait for prescaler/reload synchronization.
     */
    while (IWDG->SR != 0U)
    {
        /*
         * This occurs once during startup.
         */
    }


    /*
     * Load configured value immediately.
     */
    IWDG->KR =
            LITHIUM_IWDG_REFRESH_KEY;
}


static void Lithium_IWDG_Refresh(void)
{
    IWDG->KR =
            LITHIUM_IWDG_REFRESH_KEY;
}


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


  /* Reset of all peripherals,
     Initializes the Flash interface
     and the Systick. */

  HAL_Init();


  /* USER CODE BEGIN Init */

  /*
   * BQ_CE is active-low.
   *
   * Force it HIGH as early as possible so an MCU reset does
   * not intentionally leave charging enabled.
   */

  Lithium_EarlySafeGPIO_Init();

  /* USER CODE END Init */


  /* Configure the system clock */

  SystemClock_Config();


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */


  /* Initialize all configured peripherals */

  MX_GPIO_Init();

  MX_FDCAN1_Init();

  MX_I2C1_Init();

  MX_SPI1_Init();


  /* USER CODE BEGIN 2 */

  Lithium_LED_Init();


  Lithium_CAN_Init();


  Lithium_AppInit();


  /*
   * Start STM32 independent watchdog only after all fundamental
   * MCU peripherals and the application context are ready.
   *
   * From this point onward, a main-loop lockup will reset the
   * MCU if the watchdog is not refreshed for approximately 1 s.
   */
  Lithium_IWDG_Init();


  /*
   * Force first System FSM pass immediately.
   */
  last_system_task_ms =
          HAL_GetTick() -
          LITHIUM_SYSTEM_PERIOD_MS;

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */

    uint32_t now;


    now =
            HAL_GetTick();


    /*
     * FAST CONTINUOUS SERVICE
     *
     * - asynchronous I2C/SPI driver progress
     * - completed measurement processing
     * - interrupt flags
     * - BQ watchdog
     * - fast current safety
     */
    Lithium_ProcessEvents();


    /*
     * HIGH-LEVEL 100-ms SYSTEM TASK
     */
    if ((now -
         last_system_task_ms) >=
        LITHIUM_SYSTEM_PERIOD_MS)
    {
        last_system_task_ms =
                now;


        Lithium_SystemFSM_Run();
    }


    /*
     * LED and CAN remain alive during application faults.
     */
    Lithium_LED_Update();


    Lithium_CAN_Process();


    /*
     * Refresh ONLY here in normal thread context.
     *
     * Never refresh from:
     *
     * - ISR
     * - EXTI callback
     * - I2C callback
     * - SPI callback
     *
     * Therefore a frozen main loop eventually forces a reset.
     */
    Lithium_IWDG_Refresh();
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
  HAL_PWREx_ControlVoltageScaling(
          PWR_REGULATOR_VOLTAGE_SCALE1);


  /** Initializes the RCC Oscillators according
    * to the specified parameters in the
    * RCC_OscInitTypeDef structure.
    */
  RCC_OscInitStruct.OscillatorType =
          RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState =
          RCC_HSI_ON;

  RCC_OscInitStruct.HSIDiv =
          RCC_HSI_DIV1;

  RCC_OscInitStruct.HSICalibrationValue =
          RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState =
          RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource =
          RCC_PLLSOURCE_HSI;

  RCC_OscInitStruct.PLL.PLLM =
          RCC_PLLM_DIV1;

  RCC_OscInitStruct.PLL.PLLN =
          8;

  RCC_OscInitStruct.PLL.PLLP =
          RCC_PLLP_DIV2;

  RCC_OscInitStruct.PLL.PLLQ =
          RCC_PLLQ_DIV2;

  RCC_OscInitStruct.PLL.PLLR =
          RCC_PLLR_DIV2;


  if (HAL_RCC_OscConfig(
          &RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }


  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType =
          RCC_CLOCKTYPE_HCLK |
          RCC_CLOCKTYPE_SYSCLK |
          RCC_CLOCKTYPE_PCLK1;

  RCC_ClkInitStruct.SYSCLKSource =
          RCC_SYSCLKSOURCE_PLLCLK;

  RCC_ClkInitStruct.AHBCLKDivider =
          RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider =
          RCC_HCLK_DIV1;


  if (HAL_RCC_ClockConfig(
          &RCC_ClkInitStruct,
          FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */


  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */


  hfdcan1.Instance =
          FDCAN1;

  hfdcan1.Init.ClockDivider =
          FDCAN_CLOCK_DIV1;

  hfdcan1.Init.FrameFormat =
          FDCAN_FRAME_CLASSIC;

  hfdcan1.Init.Mode =
          FDCAN_MODE_NORMAL;

  hfdcan1.Init.AutoRetransmission =
          ENABLE;

  hfdcan1.Init.TransmitPause =
          DISABLE;

  hfdcan1.Init.ProtocolException =
          ENABLE;

  hfdcan1.Init.NominalPrescaler =
          8;

  hfdcan1.Init.NominalSyncJumpWidth =
          2;

  hfdcan1.Init.NominalTimeSeg1 =
          13;

  hfdcan1.Init.NominalTimeSeg2 =
          2;

  hfdcan1.Init.DataPrescaler =
          8;

  hfdcan1.Init.DataSyncJumpWidth =
          2;

  hfdcan1.Init.DataTimeSeg1 =
          13;

  hfdcan1.Init.DataTimeSeg2 =
          1;

  hfdcan1.Init.StdFiltersNbr =
          0;

  hfdcan1.Init.ExtFiltersNbr =
          0;

  hfdcan1.Init.TxFifoQueueMode =
          FDCAN_TX_FIFO_OPERATION;


  if (HAL_FDCAN_Init(
          &hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */


  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */


  hi2c1.Instance =
          I2C1;

  hi2c1.Init.Timing =
          0x10B17DB5;

  hi2c1.Init.OwnAddress1 =
          0;

  hi2c1.Init.AddressingMode =
          I2C_ADDRESSINGMODE_7BIT;

  hi2c1.Init.DualAddressMode =
          I2C_DUALADDRESS_DISABLE;

  hi2c1.Init.OwnAddress2 =
          0;

  hi2c1.Init.OwnAddress2Masks =
          I2C_OA2_NOMASK;

  hi2c1.Init.GeneralCallMode =
          I2C_GENERALCALL_DISABLE;

  hi2c1.Init.NoStretchMode =
          I2C_NOSTRETCH_DISABLE;


  if (HAL_I2C_Init(
          &hi2c1) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_I2CEx_ConfigAnalogFilter(
          &hi2c1,
          I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }


  if (HAL_I2CEx_ConfigDigitalFilter(
          &hi2c1,
          0) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}


/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */


  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */


  hspi1.Instance =
          SPI1;

  hspi1.Init.Mode =
          SPI_MODE_MASTER;

  hspi1.Init.Direction =
          SPI_DIRECTION_2LINES;

  hspi1.Init.DataSize =
          SPI_DATASIZE_8BIT;

  hspi1.Init.CLKPolarity =
          SPI_POLARITY_LOW;

  hspi1.Init.CLKPhase =
          SPI_PHASE_1EDGE;

  hspi1.Init.NSS =
          SPI_NSS_SOFT;

  hspi1.Init.BaudRatePrescaler =
          SPI_BAUDRATEPRESCALER_64;

  hspi1.Init.FirstBit =
          SPI_FIRSTBIT_MSB;

  hspi1.Init.TIMode =
          SPI_TIMODE_DISABLE;

  hspi1.Init.CRCCalculation =
          SPI_CRCCALCULATION_DISABLE;

  hspi1.Init.CRCPolynomial =
          7;

  hspi1.Init.CRCLength =
          SPI_CRC_LENGTH_DATASIZE;

  hspi1.Init.NSSPMode =
          SPI_NSS_PULSE_DISABLE;


  if (HAL_SPI_Init(
          &hspi1) != HAL_OK)
  {
    Error_Handler();
  }


  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
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


  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();


  HAL_GPIO_WritePin(
          GPIOA,
          BQ_CE_Pin |
          CAN_STB_Pin |
          BQ2_CS_Pin,
          GPIO_PIN_SET);


  HAL_GPIO_WritePin(
          GPIOA,
          LED1_RED_Pin |
          LED2_GREEN_Pin,
          GPIO_PIN_RESET);


  /* BQ interrupt pins */

  GPIO_InitStruct.Pin =
          BQ_INT_Pin |
          BQ2_ALERT_Pin;

  GPIO_InitStruct.Mode =
          GPIO_MODE_IT_FALLING;

  GPIO_InitStruct.Pull =
          GPIO_NOPULL;

  HAL_GPIO_Init(
          GPIOA,
          &GPIO_InitStruct);


  /* Power-good changes both directions */

  GPIO_InitStruct.Pin =
          BQ_PG_Pin;

  GPIO_InitStruct.Mode =
          GPIO_MODE_IT_RISING_FALLING;

  GPIO_InitStruct.Pull =
          GPIO_NOPULL;

  HAL_GPIO_Init(
          BQ_PG_GPIO_Port,
          &GPIO_InitStruct);


  /* Outputs */

  GPIO_InitStruct.Pin =
          BQ_CE_Pin |
          CAN_STB_Pin |
          LED1_RED_Pin |
          LED2_GREEN_Pin |
          BQ2_CS_Pin;

  GPIO_InitStruct.Mode =
          GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull =
          GPIO_NOPULL;

  GPIO_InitStruct.Speed =
          GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(
          GPIOA,
          &GPIO_InitStruct);


  /* BQ status pins */

  GPIO_InitStruct.Pin =
          BQ_STAT1_Pin |
          BQ_STAT2_Pin;

  GPIO_InitStruct.Mode =
          GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull =
          GPIO_NOPULL;

  HAL_GPIO_Init(
          GPIOB,
          &GPIO_InitStruct);


  /* EXTI */

  HAL_NVIC_SetPriority(
          EXTI0_1_IRQn,
          1,
          0);

  HAL_NVIC_EnableIRQ(
          EXTI0_1_IRQn);


  HAL_NVIC_SetPriority(
          EXTI4_15_IRQn,
          1,
          0);

  HAL_NVIC_EnableIRQ(
          EXTI4_15_IRQn);


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

  /*
   * Fatal MCU/HAL failure.
   *
   * Do NOT intentionally interrupt VSYS.
   *
   * Remove charging permission locally.
   */

  __HAL_RCC_GPIOA_CLK_ENABLE();


  HAL_GPIO_WritePin(
          BQ_CE_GPIO_Port,
          BQ_CE_Pin,
          GPIO_PIN_SET);


  HAL_GPIO_WritePin(
          CAN_STB_GPIO_Port,
          CAN_STB_Pin,
          GPIO_PIN_SET);


  HAL_GPIO_WritePin(
          BQ2_CS_GPIO_Port,
          BQ2_CS_Pin,
          GPIO_PIN_SET);


  HAL_GPIO_WritePin(
          LED2_GREEN_GPIO_Port,
          LED2_GREEN_Pin,
          GPIO_PIN_RESET);


  HAL_GPIO_WritePin(
          LED1_RED_GPIO_Port,
          LED1_RED_Pin,
          GPIO_PIN_SET);


  /*
   * Do not refresh IWDG here.
   *
   * If the watchdog is already active, the MCU will eventually
   * reset instead of remaining forever inside Error_Handler().
   */
  __disable_irq();


  while (1)
  {
  }


  /* USER CODE END Error_Handler_Debug */
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(
        uint8_t *file,
        uint32_t line)
{
  /* USER CODE BEGIN 6 */

  (void)file;

  (void)line;

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
