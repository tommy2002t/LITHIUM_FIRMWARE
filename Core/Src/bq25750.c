#include "bq25750.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "lithium_config.h"


extern I2C_HandleTypeDef hi2c1;


/* ============================================================
 * I2C
 * ============================================================ */

#define BQ25750_I2C_ADDRESS_7BIT                 0x6BU
#define BQ25750_I2C_ADDRESS_HAL                  \
        (BQ25750_I2C_ADDRESS_7BIT << 1)

#define BQ25750_CONTROL_I2C_TIMEOUT_MS           5U


/* ============================================================
 * REGISTER MAP
 * ============================================================ */

#define BQ25750_REG_CHARGE_VOLTAGE               0x00U
#define BQ25750_REG_CHARGE_CURRENT               0x02U

#define BQ25750_REG_INPUT_CURRENT_DPM            0x06U
#define BQ25750_REG_INPUT_VOLTAGE_DPM            0x08U

#define BQ25750_REG_PRECHARGE_CURRENT            0x10U
#define BQ25750_REG_TERMINATION_CURRENT          0x12U

#define BQ25750_REG_PRECHARGE_TERM_CONTROL       0x14U
#define BQ25750_REG_TIMER_CONTROL                0x15U
#define BQ25750_REG_THREE_STAGE_CONTROL          0x16U
#define BQ25750_REG_CHARGER_CONTROL              0x17U
#define BQ25750_REG_PIN_CONTROL                  0x18U
#define BQ25750_REG_POWER_PATH_CONTROL           0x19U

#define BQ25750_REG_TS_THRESHOLD_CONTROL         0x1BU
#define BQ25750_REG_TS_BEHAVIOR_CONTROL          0x1CU

#define BQ25750_REG_CHARGER_STATUS_1             0x21U
#define BQ25750_REG_CHARGER_STATUS_2             0x22U
#define BQ25750_REG_CHARGER_STATUS_3             0x23U
#define BQ25750_REG_FAULT_STATUS                 0x24U

#define BQ25750_REG_ADC_CONTROL                  0x2BU
#define BQ25750_REG_ADC_CHANNEL_CONTROL          0x2CU

#define BQ25750_REG_ADC_IAC                      0x2DU
#define BQ25750_REG_ADC_IBAT                     0x2FU
#define BQ25750_REG_ADC_VAC                      0x31U
#define BQ25750_REG_ADC_VBAT                     0x33U
#define BQ25750_REG_ADC_VSYS                     0x35U
#define BQ25750_REG_ADC_TS                       0x37U


/* ============================================================
 * CHARGER CONTROL
 * ============================================================ */

#define BQ25750_CHARGER_CONTROL_VRECHG_MASK      (3U << 6)
#define BQ25750_CHARGER_CONTROL_VRECHG_97_6      (3U << 6)

#define BQ25750_CHARGER_CONTROL_WD_RST           (1U << 5)
#define BQ25750_CHARGER_CONTROL_DIS_CE_PIN       (1U << 4)
#define BQ25750_CHARGER_CONTROL_EN_CHG_RESET     (1U << 3)
#define BQ25750_CHARGER_CONTROL_EN_HIZ           (1U << 2)
#define BQ25750_CHARGER_CONTROL_EN_CHG           (1U << 0)


/* ============================================================
 * PIN CONTROL
 * ============================================================ */

#define BQ25750_PIN_CONTROL_EN_ICHG_PIN           (1U << 7)
#define BQ25750_PIN_CONTROL_EN_ILIM_HIZ_PIN       (1U << 6)


/* ============================================================
 * POWER PATH
 * ============================================================ */

#define BQ25750_POWER_PATH_EN_PFM                 (1U << 5)
#define BQ25750_POWER_PATH_FORCE_BATFET_OFF       (1U << 4)


/* ============================================================
 * PRECHARGE / TERMINATION
 * ============================================================ */

#define BQ25750_PRETERM_EN_TERM                   (1U << 3)

#define BQ25750_PRETERM_VBAT_LOWV_MASK            (3U << 1)
#define BQ25750_PRETERM_VBAT_LOWV_71_4            (3U << 1)

#define BQ25750_PRETERM_EN_PRECHG                 (1U << 0)


/* ============================================================
 * TIMER CONTROL
 * ============================================================ */

#define BQ25750_TIMER_WATCHDOG_40S                (1U << 4)
#define BQ25750_TIMER_EN_CHG_TMR                  (1U << 3)
#define BQ25750_TIMER_CHG_TMR_8H                  (1U << 1)
#define BQ25750_TIMER_EN_TMR2X                    (1U << 0)


/* ============================================================
 * THREE-STAGE CONTROL
 * ============================================================ */

#define BQ25750_CV_TIMER_2H                       0x02U


/*
 * Keep the driver encoding explicitly tied to the system-level
 * configuration.  If either policy is changed later, compilation
 * must stop until the corresponding BQ25750 register encoding is
 * reviewed against the datasheet.
 */
#if (LITHIUM_CHARGE_SAFETY_TIMER_HOURS != 8U)
#error "BQ25750 timer encoding must be reviewed for this charge safety timer"
#endif

#if (LITHIUM_CV_TIMER_HOURS != 2U)
#error "BQ25750 CV timer encoding must be reviewed for this CV timer"
#endif


/* ============================================================
 * TS THRESHOLDS
 * ============================================================ */

/*
 * T5 (HOT) encoding is tied directly to lithium_config.h so the
 * independent BQ25750 TS backup cannot silently disagree with the
 * system-level configuration.
 *
 * T5[1:0]:
 *   00b -> 50 C
 *   01b -> 55 C
 *   10b -> 60 C
 *   11b -> 65 C
 */
#if (LITHIUM_BQ25750_TS_HOT_C == 50)
#define BQ25750_TS_T5_CONFIG                      (0U << 6)
#elif (LITHIUM_BQ25750_TS_HOT_C == 55)
#define BQ25750_TS_T5_CONFIG                      (1U << 6)
#elif (LITHIUM_BQ25750_TS_HOT_C == 60)
#define BQ25750_TS_T5_CONFIG                      (2U << 6)
#elif (LITHIUM_BQ25750_TS_HOT_C == 65)
#define BQ25750_TS_T5_CONFIG                      (3U << 6)
#else
#error "Unsupported BQ25750 TS HOT threshold"
#endif

#if (LITHIUM_BQ25750_TS_COLD_C != 0)
#error "BQ25750 TS cold-threshold encoding must be reviewed"
#endif

#define BQ25750_TS_T3_45C                         (1U << 4)
#define BQ25750_TS_T2_10C                         (1U << 2)
#define BQ25750_TS_T1_0C                          (2U << 0)

#define BQ25750_TS_THRESHOLD_VALUE                \
        (BQ25750_TS_T5_CONFIG |                  \
         BQ25750_TS_T3_45C |                     \
         BQ25750_TS_T2_10C |                     \
         BQ25750_TS_T1_0C)

#define BQ25750_TS_BEHAVIOR_EN_JEITA              (1U << 1)
#define BQ25750_TS_BEHAVIOR_EN_TS                 (1U << 0)


/* ============================================================
 * ADC
 * ============================================================ */

#define BQ25750_ADC_CONTROL_VALUE                 0x80U

#define BQ25750_ADC_CHANNEL_CONTROL_VALUE         0x02U


/* ============================================================
 * STATUS
 * ============================================================ */

#define BQ25750_STATUS1_WATCHDOG                  (1U << 3)
#define BQ25750_STATUS1_CHARGE_STATE_MASK         0x07U

#define BQ25750_STATUS2_POWER_GOOD                (1U << 7)

#define BQ25750_STATUS2_TS_STATE_MASK             (7U << 4)
#define BQ25750_STATUS2_TS_STATE_SHIFT            4U

#define BQ25750_STATUS3_CV_TIMER_EXPIRED          (1U << 3)
#define BQ25750_STATUS3_ACFET_ON                  (1U << 1)
#define BQ25750_STATUS3_BATFET_ON                 (1U << 0)


/* ============================================================
 * FAULT STATUS
 * ============================================================ */

#define BQ25750_FAULT_VAC_UV                      (1U << 7)
#define BQ25750_FAULT_VAC_OV                      (1U << 6)
#define BQ25750_FAULT_IBAT_OCP                    (1U << 5)
#define BQ25750_FAULT_VBAT_OV                     (1U << 4)
#define BQ25750_FAULT_THERMAL_SHUTDOWN            (1U << 3)
#define BQ25750_FAULT_CHARGE_TIMER                (1U << 2)


/* ============================================================
 * SCALING
 * ============================================================ */

#define BQ25750_CHARGE_CURRENT_MIN_MA             400U
#define BQ25750_CHARGE_CURRENT_MAX_MA             20000U
#define BQ25750_CHARGE_CURRENT_STEP_MA            50U

#define BQ25750_INPUT_CURRENT_MIN_MA              1000U
#define BQ25750_INPUT_CURRENT_MAX_MA              50000U
#define BQ25750_INPUT_CURRENT_STEP_MA             125U

#define BQ25750_INPUT_VOLTAGE_MIN_MV              4200U
#define BQ25750_INPUT_VOLTAGE_MAX_MV              65000U
#define BQ25750_INPUT_VOLTAGE_STEP_MV             20U

#define BQ25750_VFB_MIN_MV                        1504U
#define BQ25750_VFB_MAX_MV                        1566U
#define BQ25750_VFB_STEP_MV                       2U

#define BQ25750_PRECHARGE_CURRENT_STEP_MA         50U
#define BQ25750_TERMINATION_CURRENT_STEP_MA       50U

#define BQ25750_CURRENT_ADC_STEP_MA               2L
#define BQ25750_VOLTAGE_ADC_STEP_MV              2U

#define BQ25750_TS_ADC_FULL_SCALE                 1024.0f


/* ============================================================
 * 103AT-2 + BOARD TS NETWORK
 *
 * Real hardware after removal of the temporary 10-kohm
 * dummy resistor:
 *
 * REGN -- 5.23 kohm -- TS --+-- 30.1 kohm -- PGND
 *                            |
 *                            +-- 103AT-2 NTC -- PGND
 *
 * The BQ25750 TS ADC reports VTS as a fraction of REGN.
 * ============================================================ */

#define BQ25750_TS_PULLUP_OHM                     5230.0f
#define BQ25750_TS_FIXED_TO_GND_OHM               30100.0f

#define BQ25750_THERMISTOR_R25_OHM                10000.0f
#define BQ25750_THERMISTOR_BETA_K                 3435.0f
#define BQ25750_THERMISTOR_T25_K                  298.15f


/* ============================================================
 * READ STATE
 * ============================================================ */

typedef enum
{
    BQ25750_READ_IDLE = 0,

    BQ25750_READ_IAC,
    BQ25750_READ_IBAT,

    BQ25750_READ_VIN,
    BQ25750_READ_VBAT,
    BQ25750_READ_VSYS,

    BQ25750_READ_TS,

    BQ25750_READ_STATUS1,
    BQ25750_READ_STATUS2,
    BQ25750_READ_STATUS3,

    BQ25750_READ_FAULT,

    BQ25750_READ_COMPLETE,
    BQ25750_READ_ERROR

} bq25750_read_state_t;


/* ============================================================
 * DRIVER STATE
 * ============================================================ */

static bq25750_read_state_t read_state =
        BQ25750_READ_IDLE;

static bq25750_data_t working_data;
static bq25750_data_t completed_data;

static uint8_t async_rx_buffer[2];

static volatile bool i2c_transfer_complete = false;
static volatile bool i2c_transfer_error = false;

static bool data_ready = false;


/*
 * Cached copy of the EN_HIZ state last read or successfully written by
 * this driver.  BQ25750_Init() intentionally preserves an already-active
 * HIZ state so a communication recovery or MCU reset cannot silently
 * reconnect an input that was isolated for overvoltage.
 */
static bool input_high_impedance_active = false;


/*
 * Cached copy of FORCE_BATFET_OFF.  As with EN_HIZ, initialization
 * preserves an already-forced battery isolation until the application
 * completes its normal safety recovery path.
 */
static bool battery_fet_forced_off = false;

static uint32_t transaction_start_time_ms = 0U;


/* ============================================================
 * PRIVATE FUNCTIONS
 * ============================================================ */

static bq_status_t BQ25750_Read8Blocking(
        uint8_t reg,
        uint8_t *value);

static bq_status_t BQ25750_Write8Blocking(
        uint8_t reg,
        uint8_t value);

static bq_status_t BQ25750_Read16Blocking(
        uint8_t reg,
        uint16_t *value);

static bq_status_t BQ25750_Write16Blocking(
        uint8_t reg,
        uint16_t value);

static bq_status_t BQ25750_Modify8Blocking(
        uint8_t reg,
        uint8_t clear_mask,
        uint8_t set_mask);

static bool BQ25750_AsyncTimedOut(void);

static uint16_t BQ25750_ParseUnsigned16(void);

static int16_t BQ25750_ParseSigned16(void);

static bool BQ25750_ConvertTsCodeToTemperature(
        uint16_t ts_code,
        int16_t *temperature_dC);


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

bq_status_t BQ25750_Init(void)
{
    bq_status_t status;

    uint8_t register8;
    uint16_t verify16;


    status = BQ25750_Read8Blocking(
            BQ25750_REG_CHARGER_CONTROL,
            &register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    register8 &=
            (uint8_t)~BQ25750_CHARGER_CONTROL_EN_CHG;

    register8 &=
            (uint8_t)~BQ25750_CHARGER_CONTROL_DIS_CE_PIN;

    register8 &=
            (uint8_t)~BQ25750_CHARGER_CONTROL_EN_CHG_RESET;


    /*
     * Preserve EN_HIZ exactly as found in hardware.
     *
     * EN_HIZ is a safety-relevant power-path state.  Clearing it here
     * would make a driver re-initialization reconnect an input that the
     * application had deliberately isolated after an overvoltage event.
     * On a normal cold start the BQ25750 POR value is EN_HIZ = 0, so no
     * special startup action is required.
     */
    input_high_impedance_active =
            ((register8 &
              BQ25750_CHARGER_CONTROL_EN_HIZ) != 0U);


    register8 &=
            (uint8_t)~BQ25750_CHARGER_CONTROL_VRECHG_MASK;

    register8 |=
            BQ25750_CHARGER_CONTROL_VRECHG_97_6;


    status = BQ25750_Write8Blocking(
            BQ25750_REG_CHARGER_CONTROL,
            register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Read8Blocking(
            BQ25750_REG_PIN_CONTROL,
            &register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    register8 |=
            BQ25750_PIN_CONTROL_EN_ICHG_PIN;

    register8 |=
            BQ25750_PIN_CONTROL_EN_ILIM_HIZ_PIN;


    status = BQ25750_Write8Blocking(
            BQ25750_REG_PIN_CONTROL,
            register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ25750_SetChargeVoltageFeedbackMv(
                    LITHIUM_CHARGE_VFB_NORMAL_MV);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ25750_SetChargeCurrentMa(
                    LITHIUM_CHARGE_CURRENT_NORMAL_MA);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ25750_SetInputCurrentLimitMa(
                    LITHIUM_INPUT_CURRENT_LIMIT_MA);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status =
            BQ25750_SetInputVoltageDpmMv(
                    LITHIUM_INPUT_VAC_DPM_MV);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Write16Blocking(
            BQ25750_REG_PRECHARGE_CURRENT,
            (uint16_t)(
                    (LITHIUM_LOW_CELL_RECOVERY_CURRENT_MA /
                     BQ25750_PRECHARGE_CURRENT_STEP_MA)
                    << 2));

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Write16Blocking(
            BQ25750_REG_TERMINATION_CURRENT,
            (uint16_t)(
                    (LITHIUM_TERMINATION_CURRENT_REGISTER_MA /
                     BQ25750_TERMINATION_CURRENT_STEP_MA)
                    << 2));

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    register8 =
            BQ25750_PRETERM_EN_TERM |
            BQ25750_PRETERM_VBAT_LOWV_71_4 |
            BQ25750_PRETERM_EN_PRECHG;


    status = BQ25750_Write8Blocking(
            BQ25750_REG_PRECHARGE_TERM_CONTROL,
            register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    /*
     * 8-h fast-charge safety timer for the 5-A hardware revision.
     *
     * CHG_TMR[2:1] = 01b -> 8 h.
     * EN_TMR2X remains enabled so the timer runs at half rate
     * while the BQ25750 is in input-current / input-voltage DPM.
     */
    register8 =
            BQ25750_TIMER_WATCHDOG_40S |
            BQ25750_TIMER_EN_CHG_TMR |
            BQ25750_TIMER_CHG_TMR_8H |
            BQ25750_TIMER_EN_TMR2X;


    status = BQ25750_Write8Blocking(
            BQ25750_REG_TIMER_CONTROL,
            register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Write8Blocking(
            BQ25750_REG_THREE_STAGE_CONTROL,
            BQ25750_CV_TIMER_2H);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Read8Blocking(
            BQ25750_REG_POWER_PATH_CONTROL,
            &register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    register8 &=
            (uint8_t)~BQ25750_POWER_PATH_EN_PFM;


    /*
     * Preserve FORCE_BATFET_OFF exactly as found in hardware.
     * A communication recovery or MCU reset must not silently reconnect
     * a battery that was deliberately isolated on ground for safety.
     */
    battery_fet_forced_off =
            ((register8 &
              BQ25750_POWER_PATH_FORCE_BATFET_OFF) != 0U);


    status = BQ25750_Write8Blocking(
            BQ25750_REG_POWER_PATH_CONTROL,
            register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    /*
     * Independent pack-center 103AT-2 thresholds.
     *
     * The STM32 charge policy stops charging at 45 C.  The BQ25750
     * T5 threshold is configured separately as an independent 50-C
     * hardware charger-suspension backup.
     */
    status = BQ25750_Write8Blocking(
            BQ25750_REG_TS_THRESHOLD_CONTROL,
            BQ25750_TS_THRESHOLD_VALUE);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    /*
     * Primary adaptive derating is performed by STM32 using
     * TH1 / TH2 / TH3.
     *
     * Keep the BQ25750 TS absolute COLD/HOT protection active,
     * but explicitly disable its JEITA derating profile.
     */
    status = BQ25750_Read8Blocking(
            BQ25750_REG_TS_BEHAVIOR_CONTROL,
            &register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    register8 &=
            (uint8_t)~BQ25750_TS_BEHAVIOR_EN_JEITA;

    register8 |=
            BQ25750_TS_BEHAVIOR_EN_TS;


    status = BQ25750_Write8Blocking(
            BQ25750_REG_TS_BEHAVIOR_CONTROL,
            register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Write8Blocking(
            BQ25750_REG_ADC_CHANNEL_CONTROL,
            BQ25750_ADC_CHANNEL_CONTROL_VALUE);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Write8Blocking(
            BQ25750_REG_ADC_CONTROL,
            BQ25750_ADC_CONTROL_VALUE);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    status = BQ25750_Read16Blocking(
            BQ25750_REG_CHARGE_VOLTAGE,
            &verify16);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if ((verify16 & 0x001FU) !=
        (uint16_t)(
                (LITHIUM_CHARGE_VFB_NORMAL_MV -
                 BQ25750_VFB_MIN_MV) /
                BQ25750_VFB_STEP_MV))
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    status = BQ25750_Read16Blocking(
            BQ25750_REG_CHARGE_CURRENT,
            &verify16);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if (((verify16 >> 2) *
         BQ25750_CHARGE_CURRENT_STEP_MA) !=
        LITHIUM_CHARGE_CURRENT_NORMAL_MA)
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    /*
     * Verify the 8.0-A total input-current DPM setting.
     *
     * This limit applies to total adapter current used by VSYS
     * plus the converter input current required for battery charge.
     * With the board's 2-mohm RAC_SNS, IAC_DPM uses
     * 125-mA steps.
     */
    status = BQ25750_Read16Blocking(
            BQ25750_REG_INPUT_CURRENT_DPM,
            &verify16);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if (((verify16 >> 2) *
         BQ25750_INPUT_CURRENT_STEP_MA) !=
        LITHIUM_INPUT_CURRENT_LIMIT_MA)
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    /*
     * Verify the 1.0-A precharge / low-cell hardware limit.
     */
    status = BQ25750_Read16Blocking(
            BQ25750_REG_PRECHARGE_CURRENT,
            &verify16);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if (((verify16 >> 2) *
         BQ25750_PRECHARGE_CURRENT_STEP_MA) !=
        LITHIUM_LOW_CELL_RECOVERY_CURRENT_MA)
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    /*
     * Verify the requested 300-mA termination register.
     * With RICHG = 10 kohm the ICHG-pin termination ceiling is
     * 500 mA, therefore the 300-mA register value becomes the
     * effective termination target.
     */
    status = BQ25750_Read16Blocking(
            BQ25750_REG_TERMINATION_CURRENT,
            &verify16);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if (((verify16 >> 2) *
         BQ25750_TERMINATION_CURRENT_STEP_MA) !=
        LITHIUM_TERMINATION_CURRENT_REGISTER_MA)
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    /*
     * Verify the timer byte exactly as programmed.
     */
    status = BQ25750_Read8Blocking(
            BQ25750_REG_TIMER_CONTROL,
            &register8);

    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    if ((register8 & 0x3FU) !=
        (BQ25750_TIMER_WATCHDOG_40S |
         BQ25750_TIMER_EN_CHG_TMR |
         BQ25750_TIMER_CHG_TMR_8H |
         BQ25750_TIMER_EN_TMR2X))
    {
        return BQ_STATUS_VERIFY_ERROR;
    }


    read_state =
            BQ25750_READ_IDLE;

    data_ready =
            false;

    i2c_transfer_complete =
            false;

    i2c_transfer_error =
            false;

    transaction_start_time_ms =
            0U;


    return BQ_STATUS_OK;
}


/* ============================================================
 * START DATA READ
 * ============================================================ */

bq_status_t BQ25750_StartReadData(void)
{
    if (read_state !=
        BQ25750_READ_IDLE)
    {
        return BQ_STATUS_BUSY;
    }


    memset(
            &working_data,
            0,
            sizeof(working_data));


    data_ready =
            false;

    i2c_transfer_complete =
            false;

    i2c_transfer_error =
            false;

    transaction_start_time_ms =
            0U;


    read_state =
            BQ25750_READ_IAC;


    return BQ_STATUS_OK;
}


/* ============================================================
 * NON-BLOCKING SERVICE
 * ============================================================ */

void BQ25750_Service(void)
{
    uint16_t raw16;
    int16_t signed16;
    uint8_t value8;

    HAL_StatusTypeDef hal_status;


    switch (read_state)
    {
        case BQ25750_READ_IDLE:

            break;


        case BQ25750_READ_IAC:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_ADC_IAC,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                2U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            signed16 =
                    BQ25750_ParseSigned16();


            working_data.input_current_ma =
                    (int32_t)signed16 *
                    BQ25750_CURRENT_ADC_STEP_MA;


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_IBAT;

            break;


        case BQ25750_READ_IBAT:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_ADC_IBAT,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                2U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            signed16 =
                    BQ25750_ParseSigned16();


            working_data.battery_current_ma =
                    (int32_t)signed16 *
                    BQ25750_CURRENT_ADC_STEP_MA;


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_VIN;

            break;


        case BQ25750_READ_VIN:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_ADC_VAC,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                2U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            raw16 =
                    BQ25750_ParseUnsigned16();


            working_data.vin_mv =
                    (uint16_t)(
                            raw16 *
                            BQ25750_VOLTAGE_ADC_STEP_MV);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_VBAT;

            break;


        case BQ25750_READ_VBAT:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_ADC_VBAT,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                2U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            raw16 =
                    BQ25750_ParseUnsigned16();


            working_data.vbat_mv =
                    (uint16_t)(
                            raw16 *
                            BQ25750_VOLTAGE_ADC_STEP_MV);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_VSYS;

            break;


        case BQ25750_READ_VSYS:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_ADC_VSYS,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                2U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            raw16 =
                    BQ25750_ParseUnsigned16();


            working_data.vsys_mv =
                    (uint16_t)(
                            raw16 *
                            BQ25750_VOLTAGE_ADC_STEP_MV);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_TS;

            break;


        case BQ25750_READ_TS:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_ADC_TS,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                2U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            raw16 =
                    BQ25750_ParseUnsigned16();


            working_data.pack_center_temperature_valid =
                    BQ25750_ConvertTsCodeToTemperature(
                            raw16,
                            &working_data.pack_center_temperature_dC);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_STATUS1;

            break;


        case BQ25750_READ_STATUS1:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_CHARGER_STATUS_1,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                1U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            value8 =
                    async_rx_buffer[0];


            working_data.watchdog_expired =
                    ((value8 &
                      BQ25750_STATUS1_WATCHDOG) != 0U);


            working_data.charge_state =
                    value8 &
                    BQ25750_STATUS1_CHARGE_STATE_MASK;


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_STATUS2;

            break;


        case BQ25750_READ_STATUS2:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_CHARGER_STATUS_2,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                1U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            value8 =
                    async_rx_buffer[0];


            working_data.power_good =
                    ((value8 &
                      BQ25750_STATUS2_POWER_GOOD) != 0U);


            working_data.ts_state =
                    (uint8_t)(
                            (value8 &
                             BQ25750_STATUS2_TS_STATE_MASK) >>
                            BQ25750_STATUS2_TS_STATE_SHIFT);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_STATUS3;

            break;


        case BQ25750_READ_STATUS3:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_CHARGER_STATUS_3,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                1U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            value8 =
                    async_rx_buffer[0];


            working_data.cv_timer_expired =
                    ((value8 &
                      BQ25750_STATUS3_CV_TIMER_EXPIRED) != 0U);


            working_data.acfet_on =
                    ((value8 &
                      BQ25750_STATUS3_ACFET_ON) != 0U);


            working_data.batfet_on =
                    ((value8 &
                      BQ25750_STATUS3_BATFET_ON) != 0U);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_FAULT;

            break;


        case BQ25750_READ_FAULT:

            if (!i2c_transfer_complete &&
                !i2c_transfer_error &&
                (transaction_start_time_ms == 0U))
            {
                hal_status =
                        HAL_I2C_Mem_Read_IT(
                                &hi2c1,
                                BQ25750_I2C_ADDRESS_HAL,
                                BQ25750_REG_FAULT_STATUS,
                                I2C_MEMADD_SIZE_8BIT,
                                async_rx_buffer,
                                1U);

                if (hal_status != HAL_OK)
                {
                    read_state =
                            BQ25750_READ_ERROR;

                    break;
                }

                transaction_start_time_ms =
                        HAL_GetTick();

                break;
            }


            if (i2c_transfer_error ||
                BQ25750_AsyncTimedOut())
            {
                read_state =
                        BQ25750_READ_ERROR;

                break;
            }


            if (!i2c_transfer_complete)
            {
                break;
            }


            value8 =
                    async_rx_buffer[0];


            working_data.fault_status =
                    value8;


            working_data.input_undervoltage =
                    ((value8 &
                      BQ25750_FAULT_VAC_UV) != 0U);


            working_data.input_overvoltage =
                    ((value8 &
                      BQ25750_FAULT_VAC_OV) != 0U);


            working_data.battery_overcurrent =
                    ((value8 &
                      BQ25750_FAULT_IBAT_OCP) != 0U);


            working_data.battery_overvoltage =
                    ((value8 &
                      BQ25750_FAULT_VBAT_OV) != 0U);


            working_data.thermal_shutdown =
                    ((value8 &
                      BQ25750_FAULT_THERMAL_SHUTDOWN) != 0U);


            working_data.charge_timer_expired =
                    ((value8 &
                      BQ25750_FAULT_CHARGE_TIMER) != 0U);


            i2c_transfer_complete =
                    false;

            transaction_start_time_ms =
                    0U;

            read_state =
                    BQ25750_READ_COMPLETE;

            break;


        case BQ25750_READ_COMPLETE:

            completed_data =
                    working_data;


            data_ready =
                    true;


            read_state =
                    BQ25750_READ_IDLE;

            break;


        case BQ25750_READ_ERROR:

        default:

            i2c_transfer_complete =
                    false;

            i2c_transfer_error =
                    false;

            transaction_start_time_ms =
                    0U;

            data_ready =
                    false;

            read_state =
                    BQ25750_READ_IDLE;

            break;
    }
}


/* ============================================================
 * DATA ACCESS
 * ============================================================ */

bool BQ25750_DataReady(void)
{
    return data_ready;
}


bq_status_t BQ25750_GetData(
        bq25750_data_t *data)
{
    if (data == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    if (!data_ready)
    {
        return BQ_STATUS_BUSY;
    }


    *data =
            completed_data;


    data_ready =
            false;


    return BQ_STATUS_OK;
}


bool BQ25750_IsBusy(void)
{
    return
            (read_state !=
             BQ25750_READ_IDLE);
}


/* ============================================================
 * HAL CALLBACK FLAGS
 * ============================================================ */

void BQ25750_NotifyI2CTransferComplete(void)
{
    i2c_transfer_complete =
            true;
}


void BQ25750_NotifyI2CTransferError(void)
{
    i2c_transfer_error =
            true;
}


/* ============================================================
 * CHARGE ENABLE
 * ============================================================ */

bq_status_t BQ25750_SetSoftwareChargeEnable(
        bool enable)
{
    if (enable)
    {
        return
                BQ25750_Modify8Blocking(
                        BQ25750_REG_CHARGER_CONTROL,
                        0U,
                        BQ25750_CHARGER_CONTROL_EN_CHG);
    }


    return
            BQ25750_Modify8Blocking(
                    BQ25750_REG_CHARGER_CONTROL,
                    BQ25750_CHARGER_CONTROL_EN_CHG,
                    0U);
}


/* ============================================================
 * CHARGE CURRENT
 * ============================================================ */

bq_status_t BQ25750_SetChargeCurrentMa(
        uint16_t current_ma)
{
    uint16_t encoded;


    if ((current_ma <
         BQ25750_CHARGE_CURRENT_MIN_MA) ||
        (current_ma >
         BQ25750_CHARGE_CURRENT_MAX_MA))
    {
        return BQ_STATUS_INVALID_DATA;
    }


    current_ma =
            (uint16_t)(
                    (current_ma /
                     BQ25750_CHARGE_CURRENT_STEP_MA) *
                    BQ25750_CHARGE_CURRENT_STEP_MA);


    encoded =
            (uint16_t)(
                    (current_ma /
                     BQ25750_CHARGE_CURRENT_STEP_MA)
                    << 2);


    return
            BQ25750_Write16Blocking(
                    BQ25750_REG_CHARGE_CURRENT,
                    encoded);
}


/* ============================================================
 * CHARGE VOLTAGE
 * ============================================================ */

bq_status_t BQ25750_SetChargeVoltageFeedbackMv(
        uint16_t vfb_mv)
{
    uint16_t encoded;


    if ((vfb_mv <
         BQ25750_VFB_MIN_MV) ||
        (vfb_mv >
         BQ25750_VFB_MAX_MV))
    {
        return BQ_STATUS_INVALID_DATA;
    }


    vfb_mv =
            (uint16_t)(
                    BQ25750_VFB_MIN_MV +
                    (((vfb_mv -
                       BQ25750_VFB_MIN_MV) /
                      BQ25750_VFB_STEP_MV) *
                     BQ25750_VFB_STEP_MV));


    encoded =
            (uint16_t)(
                    (vfb_mv -
                     BQ25750_VFB_MIN_MV) /
                    BQ25750_VFB_STEP_MV);


    return
            BQ25750_Write16Blocking(
                    BQ25750_REG_CHARGE_VOLTAGE,
                    encoded);
}


/* ============================================================
 * INPUT CURRENT DPM
 * ============================================================ */

bq_status_t BQ25750_SetInputCurrentLimitMa(
        uint16_t current_ma)
{
    uint16_t encoded;


    if ((current_ma <
         BQ25750_INPUT_CURRENT_MIN_MA) ||
        (current_ma >
         BQ25750_INPUT_CURRENT_MAX_MA))
    {
        return BQ_STATUS_INVALID_DATA;
    }


    current_ma =
            (uint16_t)(
                    (current_ma /
                     BQ25750_INPUT_CURRENT_STEP_MA) *
                    BQ25750_INPUT_CURRENT_STEP_MA);


    encoded =
            (uint16_t)(
                    (current_ma /
                     BQ25750_INPUT_CURRENT_STEP_MA)
                    << 2);


    return
            BQ25750_Write16Blocking(
                    BQ25750_REG_INPUT_CURRENT_DPM,
                    encoded);
}


/* ============================================================
 * INPUT VOLTAGE DPM
 * ============================================================ */

bq_status_t BQ25750_SetInputVoltageDpmMv(
        uint16_t voltage_mv)
{
    uint16_t encoded;


    if ((voltage_mv <
         BQ25750_INPUT_VOLTAGE_MIN_MV) ||
        (voltage_mv >
         BQ25750_INPUT_VOLTAGE_MAX_MV))
    {
        return BQ_STATUS_INVALID_DATA;
    }


    voltage_mv =
            (uint16_t)(
                    (voltage_mv /
                     BQ25750_INPUT_VOLTAGE_STEP_MV) *
                    BQ25750_INPUT_VOLTAGE_STEP_MV);


    encoded =
            (uint16_t)(
                    (voltage_mv /
                     BQ25750_INPUT_VOLTAGE_STEP_MV)
                    << 2);


    return
            BQ25750_Write16Blocking(
                    BQ25750_REG_INPUT_VOLTAGE_DPM,
                    encoded);
}


/* ============================================================
 * INPUT HIZ
 * ============================================================ */

bq_status_t BQ25750_SetInputHighImpedance(
        bool enable)
{
    bq_status_t status;


    if (enable)
    {
        status =
                BQ25750_Modify8Blocking(
                        BQ25750_REG_CHARGER_CONTROL,
                        0U,
                        BQ25750_CHARGER_CONTROL_EN_HIZ);
    }
    else
    {
        status =
                BQ25750_Modify8Blocking(
                        BQ25750_REG_CHARGER_CONTROL,
                        BQ25750_CHARGER_CONTROL_EN_HIZ,
                        0U);
    }


    if (status == BQ_STATUS_OK)
    {
        input_high_impedance_active =
                enable;
    }


    return status;
}


bool BQ25750_IsInputHighImpedanceActive(void)
{
    return input_high_impedance_active;
}


/* ============================================================
 * BATTERY FET
 * ============================================================ */

bq_status_t BQ25750_ForceBatteryFetOff(
        bool force_off)
{
    bq_status_t status;


    if (force_off)
    {
        status =
                BQ25750_Modify8Blocking(
                        BQ25750_REG_POWER_PATH_CONTROL,
                        0U,
                        BQ25750_POWER_PATH_FORCE_BATFET_OFF);
    }
    else
    {
        status =
                BQ25750_Modify8Blocking(
                        BQ25750_REG_POWER_PATH_CONTROL,
                        BQ25750_POWER_PATH_FORCE_BATFET_OFF,
                        0U);
    }


    if (status == BQ_STATUS_OK)
    {
        battery_fet_forced_off =
                force_off;
    }


    return status;
}


bool BQ25750_IsBatteryFetForcedOff(void)
{
    return battery_fet_forced_off;
}


/* ============================================================
 * WATCHDOG
 * ============================================================ */

bq_status_t BQ25750_ServiceWatchdog(void)
{
    return
            BQ25750_Modify8Blocking(
                    BQ25750_REG_CHARGER_CONTROL,
                    0U,
                    BQ25750_CHARGER_CONTROL_WD_RST);
}


/* ============================================================
 * BLOCKING REGISTER ACCESS
 * ============================================================ */

static bq_status_t BQ25750_Read8Blocking(
        uint8_t reg,
        uint8_t *value)
{
    HAL_StatusTypeDef status;


    if (value == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    status =
            HAL_I2C_Mem_Read(
                    &hi2c1,
                    BQ25750_I2C_ADDRESS_HAL,
                    reg,
                    I2C_MEMADD_SIZE_8BIT,
                    value,
                    1U,
                    BQ25750_CONTROL_I2C_TIMEOUT_MS);


    if (status == HAL_OK)
    {
        return BQ_STATUS_OK;
    }


    if (status == HAL_TIMEOUT)
    {
        return BQ_STATUS_TIMEOUT;
    }


    return BQ_STATUS_COMM_ERROR;
}


static bq_status_t BQ25750_Write8Blocking(
        uint8_t reg,
        uint8_t value)
{
    HAL_StatusTypeDef status;


    status =
            HAL_I2C_Mem_Write(
                    &hi2c1,
                    BQ25750_I2C_ADDRESS_HAL,
                    reg,
                    I2C_MEMADD_SIZE_8BIT,
                    &value,
                    1U,
                    BQ25750_CONTROL_I2C_TIMEOUT_MS);


    if (status == HAL_OK)
    {
        return BQ_STATUS_OK;
    }


    if (status == HAL_TIMEOUT)
    {
        return BQ_STATUS_TIMEOUT;
    }


    return BQ_STATUS_COMM_ERROR;
}


static bq_status_t BQ25750_Read16Blocking(
        uint8_t reg,
        uint16_t *value)
{
    HAL_StatusTypeDef status;

    uint8_t buffer[2];


    if (value == NULL)
    {
        return BQ_STATUS_INVALID_DATA;
    }


    status =
            HAL_I2C_Mem_Read(
                    &hi2c1,
                    BQ25750_I2C_ADDRESS_HAL,
                    reg,
                    I2C_MEMADD_SIZE_8BIT,
                    buffer,
                    2U,
                    BQ25750_CONTROL_I2C_TIMEOUT_MS);


    if (status == HAL_OK)
    {
        *value =
                (uint16_t)buffer[0] |
                ((uint16_t)buffer[1] << 8);


        return BQ_STATUS_OK;
    }


    if (status == HAL_TIMEOUT)
    {
        return BQ_STATUS_TIMEOUT;
    }


    return BQ_STATUS_COMM_ERROR;
}


static bq_status_t BQ25750_Write16Blocking(
        uint8_t reg,
        uint16_t value)
{
    HAL_StatusTypeDef status;

    uint8_t buffer[2];


    buffer[0] =
            (uint8_t)(value & 0xFFU);

    buffer[1] =
            (uint8_t)(value >> 8);


    status =
            HAL_I2C_Mem_Write(
                    &hi2c1,
                    BQ25750_I2C_ADDRESS_HAL,
                    reg,
                    I2C_MEMADD_SIZE_8BIT,
                    buffer,
                    2U,
                    BQ25750_CONTROL_I2C_TIMEOUT_MS);


    if (status == HAL_OK)
    {
        return BQ_STATUS_OK;
    }


    if (status == HAL_TIMEOUT)
    {
        return BQ_STATUS_TIMEOUT;
    }


    return BQ_STATUS_COMM_ERROR;
}


static bq_status_t BQ25750_Modify8Blocking(
        uint8_t reg,
        uint8_t clear_mask,
        uint8_t set_mask)
{
    bq_status_t status;

    uint8_t value;


    status =
            BQ25750_Read8Blocking(
                    reg,
                    &value);


    if (status != BQ_STATUS_OK)
    {
        return status;
    }


    value &=
            (uint8_t)~clear_mask;


    value |=
            set_mask;


    return
            BQ25750_Write8Blocking(
                    reg,
                    value);
}


/* ============================================================
 * ASYNC HELPERS
 * ============================================================ */

static bool BQ25750_AsyncTimedOut(void)
{
    if (transaction_start_time_ms == 0U)
    {
        return false;
    }


    return
            ((HAL_GetTick() -
              transaction_start_time_ms) >
             LITHIUM_BQ_TRANSACTION_TIMEOUT_MS);
}


static uint16_t BQ25750_ParseUnsigned16(void)
{
    return
            (uint16_t)async_rx_buffer[0] |
            ((uint16_t)async_rx_buffer[1] << 8);
}


static int16_t BQ25750_ParseSigned16(void)
{
    return
            (int16_t)(
                    (uint16_t)async_rx_buffer[0] |
                    ((uint16_t)async_rx_buffer[1] << 8));
}


/* ============================================================
 * 103AT-2 CONVERSION
 * ============================================================ */

static bool BQ25750_ConvertTsCodeToTemperature(
        uint16_t ts_code,
        int16_t *temperature_dC)
{
    float ratio;
    float equivalent_low_resistance_ohm;
    float resistance_ohm;

    float inverse_temperature;
    float temperature_k;
    float temperature_c;


    if (temperature_dC == NULL)
    {
        return false;
    }


    if ((ts_code < 10U) ||
        (ts_code > 1014U))
    {
        return false;
    }


    ratio =
            ((float)ts_code /
             BQ25750_TS_ADC_FULL_SCALE);


    if ((ratio <= 0.0f) ||
        (ratio >= 1.0f))
    {
        return false;
    }


    /*
     * First recover the total resistance from TS to PGND.
     * This is the parallel combination of the fixed 30.1-kohm
     * resistor and the real 103AT-2 thermistor.
     */
    equivalent_low_resistance_ohm =
            BQ25750_TS_PULLUP_OHM *
            ratio /
            (1.0f - ratio);


    /*
     * With the NTC open, the low-side resistance approaches
     * the fixed 30.1-kohm resistor. Values at or above that
     * limit cannot represent a finite NTC resistance.
     */
    if ((equivalent_low_resistance_ohm <= 0.0f) ||
        (equivalent_low_resistance_ohm >=
         BQ25750_TS_FIXED_TO_GND_OHM))
    {
        return false;
    }


    resistance_ohm =
            (BQ25750_TS_FIXED_TO_GND_OHM *
             equivalent_low_resistance_ohm) /
            (BQ25750_TS_FIXED_TO_GND_OHM -
             equivalent_low_resistance_ohm);


    if ((resistance_ohm < 300.0f) ||
        (resistance_ohm > 500000.0f))
    {
        return false;
    }


    inverse_temperature =
            (1.0f /
             BQ25750_THERMISTOR_T25_K) +
            (logf(
                    resistance_ohm /
                    BQ25750_THERMISTOR_R25_OHM) /
             BQ25750_THERMISTOR_BETA_K);


    if (inverse_temperature <= 0.0f)
    {
        return false;
    }


    temperature_k =
            1.0f /
            inverse_temperature;


    temperature_c =
            temperature_k -
            273.15f;


    if ((temperature_c < -50.0f) ||
        (temperature_c > 110.0f))
    {
        return false;
    }


    *temperature_dC =
            (int16_t)(
                    temperature_c *
                    10.0f);


    return true;
}
