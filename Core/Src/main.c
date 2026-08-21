/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : BQ25750 Stage-1 strict read-only bring-up test
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"


/* -------------------------------------------------------------------------- */
/* Global peripheral handles                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Keep these globals because other source / IRQ files in the project
 * reference them, even though FDCAN and SPI are intentionally NOT
 * initialized in Stage 1.
 */

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;


/* -------------------------------------------------------------------------- */
/* BQ25750 Stage-1 configuration                                              */
/* -------------------------------------------------------------------------- */

#define BQ25750_TEST_I2C_ADDRESS_7BIT             0x6BU

#define BQ25750_TEST_I2C_ADDRESS_HAL              \
        (BQ25750_TEST_I2C_ADDRESS_7BIT << 1)


#define BQ25750_TEST_REG_CHARGER_CONTROL          0x17U

#define BQ25750_TEST_REG_PIN_CONTROL              0x18U

#define BQ25750_TEST_REG_POWER_PATH_CONTROL       0x19U

#define BQ25750_TEST_REG_CHARGER_STATUS_1         0x21U

#define BQ25750_TEST_REG_CHARGER_STATUS_2         0x22U

#define BQ25750_TEST_REG_CHARGER_STATUS_3         0x23U

#define BQ25750_TEST_REG_FAULT_STATUS             0x24U

#define BQ25750_TEST_REG_ADC_CONTROL              0x2BU

#define BQ25750_TEST_REG_ADC_CHANNEL_CONTROL      0x2CU

#define BQ25750_TEST_REG_PART_INFORMATION         0x3DU


#define BQ25750_TEST_PERIOD_MS                    500U

#define BQ25750_TEST_STARTUP_DELAY_MS             1000U

#define BQ25750_TEST_I2C_TIMEOUT_MS               20U

#define BQ25750_TEST_REQUIRED_CONSECUTIVE_PASSES  10U


/*
 * Strict POR/default values.
 */

#define BQ25750_TEST_RESET_CHARGER_CONTROL        0xC9U

#define BQ25750_TEST_RESET_PIN_CONTROL            0xC0U

#define BQ25750_TEST_RESET_POWER_PATH_CONTROL     0x20U

#define BQ25750_TEST_RESET_ADC_CONTROL            0x60U

#define BQ25750_TEST_RESET_ADC_CHANNEL_CONTROL    0x02U

#define BQ25750_TEST_PART_INFORMATION_EXPECTED    0x02U


/*
 * Strict expected values for THIS exact test condition:
 *
 * - Battery only on J3
 * - VIN / J2 disconnected
 * - CE held HIGH
 * - ADC left at POR state (disabled)
 * - Fresh BQ25750 POR
 *
 *
 * STATUS 1 = 0x00
 *
 * - ADC not running
 * - no IAC DPM
 * - no VAC DPM
 * - watchdog OK
 * - not charging
 *
 *
 * STATUS 2 = 0x00
 *
 * - VIN not power-good
 * - TS normal
 * - MPPT disabled
 *
 *
 * STATUS 3 = 0x01
 *
 * - BATFET ON
 * - ACFET OFF
 * - reverse mode OFF
 * - CV timer normal
 * - FSW_SYNC normal
 *
 *
 * FAULT STATUS = 0x02
 *
 * In battery-only mode with ADC disabled, TI documents
 * DRV_OKZ_STAT bit as always reading 1.
 *
 * Every other fault bit must be zero.
 */

#define BQ25750_TEST_EXPECTED_STATUS1             0x00U

#define BQ25750_TEST_EXPECTED_STATUS2             0x00U

#define BQ25750_TEST_EXPECTED_STATUS3             0x01U

#define BQ25750_TEST_EXPECTED_FAULT_STATUS        0x02U


/* -------------------------------------------------------------------------- */
/* Failure flags                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Any failure is latched until MCU reset / power-cycle.
 */

#define BQ25750_TEST_FAIL_I2C_ACK                 (1UL << 0)

#define BQ25750_TEST_FAIL_REGISTER_READ           (1UL << 1)

#define BQ25750_TEST_FAIL_POR_VALUES              (1UL << 2)

#define BQ25750_TEST_FAIL_PART_ID                 (1UL << 3)

#define BQ25750_TEST_FAIL_STATUS1                 (1UL << 4)

#define BQ25750_TEST_FAIL_STATUS2                 (1UL << 5)

#define BQ25750_TEST_FAIL_STATUS3                 (1UL << 6)

#define BQ25750_TEST_FAIL_FAULT_STATUS            (1UL << 7)

#define BQ25750_TEST_FAIL_SAFE_OUTPUTS            (1UL << 8)

#define BQ25750_TEST_FAIL_I2C_IDLE                (1UL << 9)

#define BQ25750_TEST_FAIL_HAL_I2C_ERROR           (1UL << 10)


/* -------------------------------------------------------------------------- */
/* Test variables                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Add these variables to STM32CubeIDE Live Expressions.
 */

static uint32_t last_bq_test_ms = 0U;


/*
 * MAIN PASS / FAIL VARIABLES
 */

volatile uint8_t bq_test_stage1_pass = 0U;

volatile uint8_t bq_test_failure_latched = 0U;

volatile uint32_t bq_test_failure_flags = 0U;


/*
 * Individual test results
 */

volatile uint8_t bq_test_address_ack = 0U;

volatile uint8_t bq_test_comm_ok = 0U;

volatile uint8_t bq_test_por_values_match = 0U;

volatile uint8_t bq_test_part_ok = 0U;

volatile uint8_t bq_test_status_ok = 0U;

volatile uint8_t bq_test_faults_ok = 0U;

volatile uint8_t bq_test_safe_outputs_ok = 0U;

volatile uint8_t bq_test_i2c_idle_ok = 0U;


/*
 * Counters
 */

volatile uint32_t bq_test_consecutive_passes = 0U;

volatile uint32_t bq_test_success_count = 0U;

volatile uint32_t bq_test_error_count = 0U;


/*
 * HAL / I2C diagnostics
 */

volatile uint32_t bq_test_last_hal_status = 0U;

volatile uint32_t bq_test_last_i2c_error = 0U;


/*
 * BQ25750 control registers
 */

volatile uint8_t bq_test_charger_control = 0U;

volatile uint8_t bq_test_pin_control = 0U;

volatile uint8_t bq_test_power_path_control = 0U;

volatile uint8_t bq_test_adc_control = 0U;

volatile uint8_t bq_test_adc_channel_control = 0U;

volatile uint8_t bq_test_part_information = 0U;


/*
 * BQ25750 status registers
 */

volatile uint8_t bq_test_status1 = 0U;

volatile uint8_t bq_test_status2 = 0U;

volatile uint8_t bq_test_status3 = 0U;

volatile uint8_t bq_test_fault_status = 0U;


/*
 * Safety / I2C physical pin state
 */

volatile uint8_t bq_test_ce_pin = 0U;

volatile uint8_t bq_test_can_stb_pin = 0U;

volatile uint8_t bq_test_bq2_cs_pin = 0U;

volatile uint8_t bq_test_scl_pin = 0U;

volatile uint8_t bq_test_sda_pin = 0U;


/*
 * Observation only.
 *
 * These are NOT used as Stage-1 PASS criteria.
 */

volatile uint8_t bq_test_pg_pin = 0U;

volatile uint8_t bq_test_stat1_pin = 0U;

volatile uint8_t bq_test_stat2_pin = 0U;

volatile uint8_t bq_test_int_pin = 0U;


/* -------------------------------------------------------------------------- */
/* Function prototypes                                                        */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void);


static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);


static void BQ25750_TestEarlySafeGPIO_Init(void);


static HAL_StatusTypeDef BQ25750_TestRead8(
        uint8_t reg,
        volatile uint8_t *destination);


static void BQ25750_TestSetPendingLEDs(void);

static void BQ25750_TestSetPassLEDs(void);


static void BQ25750_TestLatchFailure(
        uint32_t failure_flag);


static void BQ25750_TestRun(void);


/* -------------------------------------------------------------------------- */
/* Early safe GPIO                                                            */
/* -------------------------------------------------------------------------- */

static void BQ25750_TestEarlySafeGPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();


    /*
     * Preload SAFE levels BEFORE changing pin direction.
     *
     * BQ_CE HIGH:
     *     charging permission removed.
     *
     * CAN_STB HIGH:
     *     CAN transceiver standby.
     *
     * BQ2_CS HIGH:
     *     BQ76942 deselected.
     */

    HAL_GPIO_WritePin(
            GPIOA,
            BQ_CE_Pin |
            CAN_STB_Pin |
            BQ2_CS_Pin,
            GPIO_PIN_SET);


    /*
     * LEDs initially OFF.
     */

    HAL_GPIO_WritePin(
            GPIOA,
            LED1_RED_Pin |
            LED2_GREEN_Pin,
            GPIO_PIN_RESET);


    GPIO_InitStruct.Pin =
            BQ_CE_Pin |
            CAN_STB_Pin |
            BQ2_CS_Pin |
            LED1_RED_Pin |
            LED2_GREEN_Pin;


    GPIO_InitStruct.Mode =
            GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
            GPIO_NOPULL;


    GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct);
}


/* -------------------------------------------------------------------------- */
/* One-byte BQ25750 register read                                             */
/* -------------------------------------------------------------------------- */

static HAL_StatusTypeDef BQ25750_TestRead8(
        uint8_t reg,
        volatile uint8_t *destination)
{
    HAL_StatusTypeDef status;

    uint8_t value = 0U;


    if (destination == NULL)
    {
        return HAL_ERROR;
    }


    status =
            HAL_I2C_Mem_Read(
                    &hi2c1,
                    BQ25750_TEST_I2C_ADDRESS_HAL,
                    reg,
                    I2C_MEMADD_SIZE_8BIT,
                    &value,
                    1U,
                    BQ25750_TEST_I2C_TIMEOUT_MS);


    if (status == HAL_OK)
    {
        *destination =
                value;
    }


    return status;
}


/* -------------------------------------------------------------------------- */
/* LED logic                                                                  */
/* -------------------------------------------------------------------------- */

static void BQ25750_TestSetPendingLEDs(void)
{
    /*
     * Neither PASS nor FAIL yet.
     *
     * RED   OFF
     * GREEN OFF
     */

    HAL_GPIO_WritePin(
            LED1_RED_GPIO_Port,
            LED1_RED_Pin,
            GPIO_PIN_RESET);


    HAL_GPIO_WritePin(
            LED2_GREEN_GPIO_Port,
            LED2_GREEN_Pin,
            GPIO_PIN_RESET);
}


static void BQ25750_TestSetPassLEDs(void)
{
    /*
     * STRICT STAGE-1 PASS
     *
     * RED   OFF
     * GREEN ON
     */

    HAL_GPIO_WritePin(
            LED1_RED_GPIO_Port,
            LED1_RED_Pin,
            GPIO_PIN_RESET);


    HAL_GPIO_WritePin(
            LED2_GREEN_GPIO_Port,
            LED2_GREEN_Pin,
            GPIO_PIN_SET);
}


static void BQ25750_TestLatchFailure(
        uint32_t failure_flag)
{
    /*
     * Save the exact reason for the failure.
     */

    bq_test_failure_flags |=
            failure_flag;


    /*
     * Once failure occurs it stays failed until reset.
     */

    bq_test_failure_latched =
            1U;


    bq_test_stage1_pass =
            0U;


    bq_test_comm_ok =
            0U;


    bq_test_consecutive_passes =
            0U;


    bq_test_error_count++;


    /*
     * GREEN can never return after a failure.
     */

    HAL_GPIO_WritePin(
            LED2_GREEN_GPIO_Port,
            LED2_GREEN_Pin,
            GPIO_PIN_RESET);


    /*
     * RED permanently ON.
     */

    HAL_GPIO_WritePin(
            LED1_RED_GPIO_Port,
            LED1_RED_Pin,
            GPIO_PIN_SET);
}


/* -------------------------------------------------------------------------- */
/* Strict Stage-1 BQ25750 test                                                */
/* -------------------------------------------------------------------------- */

static void BQ25750_TestRun(void)
{
    HAL_StatusTypeDef status;


    /*
     * Never hide an intermittent failure.
     *
     * If anything failed previously,
     * leave RED latched forever until reset.
     */

    if (bq_test_failure_latched != 0U)
    {
        return;
    }


    /* ======================================================================
     * STEP 1
     *
     * Force all safety outputs.
     * ====================================================================== */


    /*
     * BQ_CE is active-low.
     *
     * HIGH = charging NOT allowed.
     */

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


    /*
     * Read actual pin levels back from GPIO input register.
     */

    bq_test_ce_pin =
            (HAL_GPIO_ReadPin(
                    BQ_CE_GPIO_Port,
                    BQ_CE_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    bq_test_can_stb_pin =
            (HAL_GPIO_ReadPin(
                    CAN_STB_GPIO_Port,
                    CAN_STB_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    bq_test_bq2_cs_pin =
            (HAL_GPIO_ReadPin(
                    BQ2_CS_GPIO_Port,
                    BQ2_CS_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    if ((bq_test_ce_pin != 1U) ||
        (bq_test_can_stb_pin != 1U) ||
        (bq_test_bq2_cs_pin != 1U))
    {
        bq_test_safe_outputs_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_SAFE_OUTPUTS);


        return;
    }


    bq_test_safe_outputs_ok =
            1U;


    /* ======================================================================
     * Observation-only physical BQ pins
     * ====================================================================== */

    bq_test_pg_pin =
            (HAL_GPIO_ReadPin(
                    BQ_PG_GPIO_Port,
                    BQ_PG_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    bq_test_stat1_pin =
            (HAL_GPIO_ReadPin(
                    BQ_STAT1_GPIO_Port,
                    BQ_STAT1_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    bq_test_stat2_pin =
            (HAL_GPIO_ReadPin(
                    BQ_STAT2_GPIO_Port,
                    BQ_STAT2_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    bq_test_int_pin =
            (HAL_GPIO_ReadPin(
                    BQ_INT_GPIO_Port,
                    BQ_INT_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    /* ======================================================================
     * STEP 2
     *
     * BQ25750 I2C address ACK.
     * ====================================================================== */

    status =
            HAL_I2C_IsDeviceReady(
                    &hi2c1,
                    BQ25750_TEST_I2C_ADDRESS_HAL,
                    3U,
                    BQ25750_TEST_I2C_TIMEOUT_MS);


    bq_test_last_hal_status =
            (uint32_t)status;


    bq_test_last_i2c_error =
            HAL_I2C_GetError(&hi2c1);


    if (status != HAL_OK)
    {
        bq_test_address_ack =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_I2C_ACK);


        return;
    }


    bq_test_address_ack =
            1U;


    /* ======================================================================
     * STEP 3
     *
     * READ all required BQ25750 registers.
     *
     * IMPORTANT:
     *
     * There are NO register writes anywhere in Stage 1.
     * ====================================================================== */


#define BQ25750_STAGE1_READ_OR_FAIL(register_address, destination)            \
    do                                                                        \
    {                                                                         \
        status =                                                              \
                BQ25750_TestRead8(                                            \
                        (register_address),                                   \
                        &(destination));                                      \
                                                                              \
        if (status != HAL_OK)                                                 \
        {                                                                     \
            bq_test_last_hal_status =                                         \
                    (uint32_t)status;                                         \
                                                                              \
            bq_test_last_i2c_error =                                          \
                    HAL_I2C_GetError(&hi2c1);                                \
                                                                              \
            BQ25750_TestLatchFailure(                                         \
                    BQ25750_TEST_FAIL_REGISTER_READ);                         \
                                                                              \
            return;                                                           \
        }                                                                     \
    } while (0)


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_CHARGER_CONTROL,
            bq_test_charger_control);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_PIN_CONTROL,
            bq_test_pin_control);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_POWER_PATH_CONTROL,
            bq_test_power_path_control);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_ADC_CONTROL,
            bq_test_adc_control);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_ADC_CHANNEL_CONTROL,
            bq_test_adc_channel_control);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_PART_INFORMATION,
            bq_test_part_information);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_CHARGER_STATUS_1,
            bq_test_status1);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_CHARGER_STATUS_2,
            bq_test_status2);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_CHARGER_STATUS_3,
            bq_test_status3);


    BQ25750_STAGE1_READ_OR_FAIL(
            BQ25750_TEST_REG_FAULT_STATUS,
            bq_test_fault_status);


#undef BQ25750_STAGE1_READ_OR_FAIL


    bq_test_last_hal_status =
            (uint32_t)HAL_OK;


    bq_test_last_i2c_error =
            HAL_I2C_GetError(&hi2c1);


    /* ======================================================================
     * STEP 4
     *
     * HAL I2C must report zero errors.
     * ====================================================================== */

    if (bq_test_last_i2c_error !=
        (uint32_t)HAL_I2C_ERROR_NONE)
    {
        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_HAL_I2C_ERROR);


        return;
    }


    /* ======================================================================
     * STEP 5
     *
     * I2C bus must return to idle:
     *
     * SCL = HIGH
     * SDA = HIGH
     * ====================================================================== */

    bq_test_scl_pin =
            (HAL_GPIO_ReadPin(
                    BQ_SCL_GPIO_Port,
                    BQ_SCL_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    bq_test_sda_pin =
            (HAL_GPIO_ReadPin(
                    BQ_SDA_GPIO_Port,
                    BQ_SDA_Pin) == GPIO_PIN_SET)
            ? 1U : 0U;


    if ((bq_test_scl_pin != 1U) ||
        (bq_test_sda_pin != 1U))
    {
        bq_test_i2c_idle_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_I2C_IDLE);


        return;
    }


    bq_test_i2c_idle_ok =
            1U;


    /* ======================================================================
     * STEP 6
     *
     * Strict POR/default register verification.
     * ====================================================================== */

    if ((bq_test_charger_control !=
         BQ25750_TEST_RESET_CHARGER_CONTROL) ||

        (bq_test_pin_control !=
         BQ25750_TEST_RESET_PIN_CONTROL) ||

        (bq_test_power_path_control !=
         BQ25750_TEST_RESET_POWER_PATH_CONTROL) ||

        (bq_test_adc_control !=
         BQ25750_TEST_RESET_ADC_CONTROL) ||

        (bq_test_adc_channel_control !=
         BQ25750_TEST_RESET_ADC_CHANNEL_CONTROL))
    {
        bq_test_por_values_match =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_POR_VALUES);


        return;
    }


    bq_test_por_values_match =
            1U;


    /* ======================================================================
     * STEP 7
     *
     * Verify actual BQ25750 silicon ID / revision byte.
     * ====================================================================== */

    if (bq_test_part_information !=
        BQ25750_TEST_PART_INFORMATION_EXPECTED)
    {
        bq_test_part_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_PART_ID);


        return;
    }


    bq_test_part_ok =
            1U;


    /* ======================================================================
     * STEP 8
     *
     * Strict battery-only status validation.
     * ====================================================================== */


    /*
     * STATUS 1
     */

    if (bq_test_status1 !=
        BQ25750_TEST_EXPECTED_STATUS1)
    {
        bq_test_status_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_STATUS1);


        return;
    }


    /*
     * STATUS 2
     */

    if (bq_test_status2 !=
        BQ25750_TEST_EXPECTED_STATUS2)
    {
        bq_test_status_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_STATUS2);


        return;
    }


    /*
     * STATUS 3
     *
     * Battery-only:
     *
     * BATFET must be ON.
     */

    if (bq_test_status3 !=
        BQ25750_TEST_EXPECTED_STATUS3)
    {
        bq_test_status_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_STATUS3);


        return;
    }


    bq_test_status_ok =
            1U;


    /* ======================================================================
     * STEP 9
     *
     * Fault register validation.
     *
     * In battery-only mode with ADC disabled:
     *
     * DRV_OKZ_STAT = 1 is expected.
     *
     * Therefore:
     *
     * Fault Status must equal exactly 0x02.
     *
     * No VAC_OV
     * No VAC_UV
     * No IBAT_OCP
     * No VBAT_OV
     * No thermal shutdown
     * No charge timer fault
     * ====================================================================== */

    if (bq_test_fault_status !=
        BQ25750_TEST_EXPECTED_FAULT_STATUS)
    {
        bq_test_faults_ok =
                0U;


        BQ25750_TestLatchFailure(
                BQ25750_TEST_FAIL_FAULT_STATUS);


        return;
    }


    bq_test_faults_ok =
            1U;


    /* ======================================================================
     * COMPLETE SUCCESSFUL TEST CYCLE
     * ====================================================================== */

    bq_test_comm_ok =
            1U;


    bq_test_success_count++;


    bq_test_consecutive_passes++;


    /*
     * GREEN is NOT allowed after only one successful communication.
     *
     * We require 10 complete consecutive successful cycles.
     */

    if (bq_test_consecutive_passes >=
        BQ25750_TEST_REQUIRED_CONSECUTIVE_PASSES)
    {
        bq_test_stage1_pass =
                1U;


        BQ25750_TestSetPassLEDs();
    }
    else
    {
        /*
         * Still proving stability.
         *
         * Both LEDs remain OFF.
         */

        bq_test_stage1_pass =
                0U;


        BQ25750_TestSetPendingLEDs();
    }
}


/* -------------------------------------------------------------------------- */
/* Main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
    /*
     * Reset peripherals, Flash interface and SysTick.
     */

    HAL_Init();


    /*
     * IMPORTANT:
     *
     * CE HIGH as early as physically possible.
     */

    BQ25750_TestEarlySafeGPIO_Init();


    /*
     * Configure MCU clock.
     */

    SystemClock_Config();


    /*
     * Stage 1 intentionally initializes ONLY:
     *
     * GPIO
     * I2C1
     *
     * No SPI.
     * No CAN.
     */

    MX_GPIO_Init();

    MX_I2C1_Init();


    /*
     * Re-assert safe states after normal GPIO initialization.
     */

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


    /*
     * No IWDG in Stage 1.
     *
     * This allows debugging / breakpoints without watchdog reset.
     *
     * No:
     *
     * Lithium_AppInit()
     * BQ25750_Init()
     * BQ76942_Init()
     * charger FSM
     * balancing FSM
     * FDCAN
     * SPI
     */


    /*
     * At startup:
     *
     * RED   OFF
     * GREEN OFF
     *
     * until the test has proven itself.
     */

    BQ25750_TestSetPendingLEDs();


    /*
     * Give BQ25750 enough time to complete battery-only POR
     * and BATFET startup.
     */

    HAL_Delay(
            BQ25750_TEST_STARTUP_DELAY_MS);


    /*
     * First strict test cycle.
     */

    BQ25750_TestRun();


    last_bq_test_ms =
            HAL_GetTick();


    /* ----------------------------------------------------------------------
     * Main loop
     * ---------------------------------------------------------------------- */

    while (1)
    {
        uint32_t now;


        /*
         * Never allow charging during Stage 1.
         */

        HAL_GPIO_WritePin(
                BQ_CE_GPIO_Port,
                BQ_CE_Pin,
                GPIO_PIN_SET);


        now =
                HAL_GetTick();


        /*
         * Repeat complete Stage-1 test every 500 ms.
         */

        if ((now - last_bq_test_ms) >=
            BQ25750_TEST_PERIOD_MS)
        {
            last_bq_test_ms =
                    now;


            BQ25750_TestRun();
        }
    }
}


/* -------------------------------------------------------------------------- */
/* System clock                                                               */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


    HAL_PWREx_ControlVoltageScaling(
            PWR_REGULATOR_VOLTAGE_SCALE1);


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


/* -------------------------------------------------------------------------- */
/* I2C1 initialization                                                        */
/* -------------------------------------------------------------------------- */

static void MX_I2C1_Init(void)
{
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
}


/* -------------------------------------------------------------------------- */
/* GPIO initialization                                                        */
/* -------------------------------------------------------------------------- */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();


    /*
     * Safe output levels BEFORE output-mode configuration.
     */

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


    /*
     * BQ diagnostic pins.
     *
     * Plain inputs.
     *
     * NO EXTI interrupts in Stage 1.
     */

    GPIO_InitStruct.Pin =
            BQ_INT_Pin |
            BQ_PG_Pin |
            BQ2_ALERT_Pin;


    GPIO_InitStruct.Mode =
            GPIO_MODE_INPUT;


    GPIO_InitStruct.Pull =
            GPIO_NOPULL;


    HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct);


    /*
     * Safety outputs + test LEDs.
     */

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


    /*
     * BQ25750 STAT pins.
     *
     * Observation only.
     */

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
}


/* -------------------------------------------------------------------------- */
/* Fatal error handler                                                        */
/* -------------------------------------------------------------------------- */

void Error_Handler(void)
{
    /*
     * Even if MCU initialization itself fails,
     * first force the system into its safe state.
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


    /*
     * Fatal MCU/HAL error:
     *
     * GREEN OFF
     * RED ON
     */

    HAL_GPIO_WritePin(
            LED2_GREEN_GPIO_Port,
            LED2_GREEN_Pin,
            GPIO_PIN_RESET);


    HAL_GPIO_WritePin(
            LED1_RED_GPIO_Port,
            LED1_RED_Pin,
            GPIO_PIN_SET);


    __disable_irq();


    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
        uint8_t *file,
        uint32_t line)
{
    (void)file;

    (void)line;
}

#endif /* USE_FULL_ASSERT */
