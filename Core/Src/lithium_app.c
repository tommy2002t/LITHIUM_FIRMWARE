#include "lithium_app.h"

#include <string.h>

#include "main.h"

#include "lithium_config.h"

#include "charger_fsm.h"
#include "balancing_fsm.h"

#include "lithium_led.h"

#include "bq25750.h"
#include "bq76942.h"


/* ============================================================
 * GLOBAL CONTEXT
 * ============================================================ */

lithium_context_t g_lithium;

lithium_event_flags_t g_lithium_events;


/* ============================================================
 * LAST COMPLETE BQ SNAPSHOTS
 * ============================================================ */

static bq25750_data_t last_bq25750_data;

static bq76942_data_t last_bq76942_data;


/* ============================================================
 * MEASUREMENT TIMING
 * ============================================================ */

static uint32_t last_measurement_request_ms = 0U;

static uint32_t last_bq25750_measurement_ms = 0U;

static uint32_t last_bq76942_measurement_ms = 0U;

static uint32_t last_watchdog_service_ms = 0U;

static uint32_t last_fast_power_safety_ms = 0U;


/* ============================================================
 * COMMUNICATION FAILURE COUNTERS
 * ============================================================ */

static uint8_t bq25750_comm_fail_count = 0U;

static uint8_t bq76942_comm_fail_count = 0U;


/* ============================================================
 * SELF TEST
 * ============================================================ */

static uint8_t self_test_good_count = 0U;


/* ============================================================
 * VOLTAGE MONITORING COUNTERS
 * ============================================================ */

static uint8_t severe_ov_count = 0U;

static uint8_t severe_uv_count = 0U;

static uint8_t pack_voltage_mismatch_count = 0U;

static uint8_t pack_voltage_match_count = 0U;


/* ============================================================
 * INPUT HIZ RECOVERY
 * ============================================================ */

static uint8_t input_hiz_recovery_count = 0U;

static uint32_t input_hiz_recovery_sample_ms = 0U;


/* ============================================================
 * TEMPERATURE FILTERING
 * ============================================================ */

/*
 * accepted_cell_temperature_dC[]
 *
 * Last temperature sample accepted as physically plausible.
 *
 * A one-sample spike does not immediately replace these values.
 */
static int16_t accepted_cell_temperature_dC[3];

static int16_t accepted_pack_temperature_dC = 0;


/*
 * Indicates whether an initial accepted measurement exists.
 */
static bool accepted_cell_temperature_valid[3];

static bool accepted_pack_temperature_valid = false;


/*
 * Consecutive fast-step samples.
 */
static uint8_t cell_temperature_step_count[3];

static uint8_t pack_temperature_step_count = 0U;


/*
 * Persistent disagreement counters.
 */
static uint8_t cell_temperature_disagreement_count[3];

static uint8_t pack_temperature_disagreement_count = 0U;


/*
 * Thermal threshold confirmation.
 *
 * Two agreeing sensors:
 * fast confirmation.
 *
 * One isolated sensor:
 * slower confirmation.
 */
static uint8_t hot_cross_confirm_count = 0U;

static uint8_t cold_cross_confirm_count = 0U;

static uint8_t hot_single_sensor_count = 0U;

static uint8_t cold_single_sensor_count = 0U;


/*
 * Recovery confirmation.
 */
static uint8_t hot_recovery_count = 0U;

static uint8_t cold_recovery_count = 0U;


/* ============================================================
 * OVERCURRENT
 * ============================================================ */

static uint8_t overcurrent_count = 0U;


/* ============================================================
 * PRIVATE FUNCTIONS
 * ============================================================ */

static void Lithium_StartPeriodicMeasurements(
        uint32_t now);

static void Lithium_CollectCompletedMeasurements(
        uint32_t now);

static void Lithium_UpdateMeasurementFreshness(
        uint32_t now);

static void Lithium_UpdateTemperatureValidation(void);

static void Lithium_UpdateThermalFaults(void);

static void Lithium_UpdateVoltageFaults(void);

static void Lithium_UpdateInputFaults(void);

static void Lithium_UpdateChargerFaults(void);

static void Lithium_UpdatePackVoltageCrossCheck(void);

static void Lithium_UpdatePowerSource(void);

static void Lithium_UpdateStatusIndication(void);

static void Lithium_ApplyPowerPathSafety(void);

static void Lithium_RunFastPowerSafety(
        uint32_t now);

static void Lithium_ServiceBQWatchdog(
        uint32_t now);

static void Lithium_RunBQRecovery(
        uint32_t now);

static bool Lithium_SelfTestSetIsGood(void);

static uint16_t Lithium_GetMinCellMv(void);

static uint16_t Lithium_GetMaxCellMv(void);

static int16_t Lithium_GetLowestTrustedTemperaturedC(void);

static int16_t Lithium_GetHighestTrustedTemperaturedC(void);

static uint32_t Lithium_SelectPrimaryFault(
        uint32_t faults);

static int16_t Lithium_AbsTemperatureDifference(
        int16_t a,
        int16_t b);

static bool Lithium_TemperaturePlausible(
        int16_t temperature_dC);

static uint8_t Lithium_CountHotTemperatureSources(void);

static uint8_t Lithium_CountColdTemperatureSources(void);

static uint8_t Lithium_CountTemperatureSourcesBelow(
        int16_t threshold_dC);

static uint8_t Lithium_CountTemperatureSourcesAbove(
        int16_t threshold_dC);


/* ============================================================
 * EARLY SAFE GPIO
 * ============================================================ */

void Lithium_EarlySafeGPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();


    /*
     * Preload safe output states BEFORE switching the pins
     * to output mode.
     *
     * BQ_CE HIGH:
     * charging hardware permission removed.
     *
     * CAN_STB HIGH:
     * CAN transceiver standby.
     *
     * BQ2_CS HIGH:
     * BQ76942 deselected.
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


/* ============================================================
 * APPLICATION INIT
 * ============================================================ */

void Lithium_AppInit(void)
{
    uint32_t now;


    now =
            HAL_GetTick();


    memset(
            &g_lithium,
            0,
            sizeof(g_lithium));


    memset(
            &g_lithium_events,
            0,
            sizeof(g_lithium_events));


    memset(
            &last_bq25750_data,
            0,
            sizeof(last_bq25750_data));


    memset(
            &last_bq76942_data,
            0,
            sizeof(last_bq76942_data));


    memset(
            accepted_cell_temperature_dC,
            0,
            sizeof(accepted_cell_temperature_dC));


    memset(
            accepted_cell_temperature_valid,
            0,
            sizeof(accepted_cell_temperature_valid));


    memset(
            cell_temperature_step_count,
            0,
            sizeof(cell_temperature_step_count));


    memset(
            cell_temperature_disagreement_count,
            0,
            sizeof(cell_temperature_disagreement_count));


    g_lithium.system_state =
            SYS_BOOT;


    g_lithium.charger_state =
            CHARGE_OFF;


    g_lithium.balancing_state =
            BALANCE_OFF;


    g_lithium.power_source =
            POWER_UNKNOWN;


    g_lithium.operating_mode =
            OPERATING_MODE_UNKNOWN;


    g_lithium.bq25750_status =
            BQ_STATUS_NOT_INITIALIZED;


    g_lithium.bq76942_status =
            BQ_STATUS_NOT_INITIALIZED;


    g_lithium.state_enter_time_ms =
            now;


    g_lithium.last_init_attempt_ms =
            now;


    g_lithium.last_recovery_attempt_ms =
            now;


    g_lithium.last_measurement_time_ms =
            now;


    g_lithium.remaining_charge_mah =
            LITHIUM_PACK_MINIMUM_CAPACITY_MAH;


    g_lithium.soc_centi_percent =
            10000U;


    last_measurement_request_ms =
            now;


    last_bq25750_measurement_ms =
            now;


    last_bq76942_measurement_ms =
            now;


    last_watchdog_service_ms =
            now;


    last_fast_power_safety_ms =
            now;


    bq25750_comm_fail_count =
            0U;


    bq76942_comm_fail_count =
            0U;


    self_test_good_count =
            0U;


    severe_ov_count =
            0U;


    severe_uv_count =
            0U;


    pack_voltage_mismatch_count =
            0U;


    pack_voltage_match_count =
            0U;


    input_hiz_recovery_count =
            0U;


    input_hiz_recovery_sample_ms =
            now;


    hot_cross_confirm_count =
            0U;


    cold_cross_confirm_count =
            0U;


    hot_single_sensor_count =
            0U;


    cold_single_sensor_count =
            0U;


    hot_recovery_count =
            0U;


    cold_recovery_count =
            0U;


    overcurrent_count =
            0U;


    ChargerFSM_Init(
            &g_lithium);


    BalancingFSM_Init(
            &g_lithium);


    Lithium_LED_SetPattern(
            LED_PATTERN_BOOT);
}


/* ============================================================
 * FAST MAIN LOOP
 * ============================================================ */

void Lithium_ProcessEvents(void)
{
    uint32_t now;


    now =
            HAL_GetTick();


    /*
     * Drivers are serviced every main-loop pass.
     *
     * No waiting loop exists here.
     */
    if (g_lithium.bq25750_initialized)
    {
        BQ25750_Service();
    }


    if (g_lithium.bq76942_initialized)
    {
        BQ76942_Service();
    }


    /*
     * Hardware interrupt flags only request fresh data.
     *
     * They never perform communication inside the ISR.
     */
    if (g_lithium_events.bq25750_interrupt)
    {
        g_lithium_events.bq25750_interrupt =
                false;


        last_measurement_request_ms =
                now - LITHIUM_SYSTEM_PERIOD_MS;
    }


    if (g_lithium_events.bq76942_alert)
    {
        g_lithium_events.bq76942_alert =
                false;


        last_measurement_request_ms =
                now - LITHIUM_SYSTEM_PERIOD_MS;
    }


    if (g_lithium_events.power_good_changed)
    {
        g_lithium_events.power_good_changed =
                false;


        last_measurement_request_ms =
                now - LITHIUM_SYSTEM_PERIOD_MS;
    }


    Lithium_StartPeriodicMeasurements(
            now);


    Lithium_CollectCompletedMeasurements(
            now);


    Lithium_UpdateMeasurementFreshness(
            now);


    Lithium_ServiceBQWatchdog(
            now);


    Lithium_RunFastPowerSafety(
            now);
}


/* ============================================================
 * START PERIODIC MEASUREMENTS
 * ============================================================ */

static void Lithium_StartPeriodicMeasurements(
        uint32_t now)
{
    if ((now -
         last_measurement_request_ms) <
        LITHIUM_SYSTEM_PERIOD_MS)
    {
        return;
    }


    last_measurement_request_ms =
            now;


    if (g_lithium.bq25750_initialized &&
        (!BQ25750_IsBusy()))
    {
        if (BQ25750_StartReadData() !=
            BQ_STATUS_OK)
        {
            if (bq25750_comm_fail_count <
                255U)
            {
                bq25750_comm_fail_count++;
            }
        }
    }


    if (g_lithium.bq76942_initialized &&
        (!BQ76942_IsBusy()))
    {
        if (BQ76942_StartReadData() !=
            BQ_STATUS_OK)
        {
            if (bq76942_comm_fail_count <
                255U)
            {
                bq76942_comm_fail_count++;
            }
        }
    }
}


/* ============================================================
 * COLLECT COMPLETED DATA
 * ============================================================ */

static void Lithium_CollectCompletedMeasurements(
        uint32_t now)
{
    bool updated = false;


    /* --------------------------------------------------------
     * BQ25750
     * -------------------------------------------------------- */

    if (BQ25750_DataReady())
    {
        if (BQ25750_GetData(
                &last_bq25750_data) ==
            BQ_STATUS_OK)
        {
            bq25750_comm_fail_count =
                    0U;


            g_lithium.bq25750_status =
                    BQ_STATUS_OK;


            g_lithium.measurements.vin_mv =
                    last_bq25750_data.vin_mv;


            g_lithium.measurements.vsys_mv =
                    last_bq25750_data.vsys_mv;


            g_lithium.measurements.charger_vbat_mv =
                    last_bq25750_data.vbat_mv;


            g_lithium.measurements.battery_current_ma =
                    last_bq25750_data.battery_current_ma;


            g_lithium.measurements.input_current_ma =
                    last_bq25750_data.input_current_ma;


            g_lithium.measurements.charger_power_good =
                    last_bq25750_data.power_good;


            g_lithium.measurements.charger_watchdog_expired =
                    last_bq25750_data.watchdog_expired;


            g_lithium.measurements.charger_charge_state =
                    last_bq25750_data.charge_state;


            g_lithium.measurements.charger_fault_status =
                    last_bq25750_data.fault_status;


            g_lithium.measurements.pack_center_temperature_dC =
                    last_bq25750_data.pack_center_temperature_dC;


            g_lithium.measurements.pack_center_temperature_valid =
                    last_bq25750_data.pack_center_temperature_valid;


            g_lithium.measurements.bq25750_valid =
                    true;


            last_bq25750_measurement_ms =
                    now;


            updated =
                    true;


            Lithium_ClearFault(
                    LITHIUM_FAULT_BQ25750_COMM);
        }
    }


    /* --------------------------------------------------------
     * BQ76942
     * -------------------------------------------------------- */

    if (BQ76942_DataReady())
    {
        if (BQ76942_GetData(
                &last_bq76942_data) ==
            BQ_STATUS_OK)
        {
            bq76942_comm_fail_count =
                    0U;


            g_lithium.bq76942_status =
                    BQ_STATUS_OK;


            g_lithium.measurements.cell_mv[0] =
                    last_bq76942_data.cell_mv[0];


            g_lithium.measurements.cell_mv[1] =
                    last_bq76942_data.cell_mv[1];


            g_lithium.measurements.cell_mv[2] =
                    last_bq76942_data.cell_mv[2];


            g_lithium.measurements.pack_mv =
                    last_bq76942_data.pack_mv;


            g_lithium.measurements.temperature_dC[0] =
                    last_bq76942_data.temperature_dC[0];


            g_lithium.measurements.temperature_dC[1] =
                    last_bq76942_data.temperature_dC[1];


            g_lithium.measurements.temperature_dC[2] =
                    last_bq76942_data.temperature_dC[2];


            g_lithium.measurements.bq76942_cuv_active =
                    last_bq76942_data.cell_undervoltage;


            g_lithium.measurements.bq76942_cov_active =
                    last_bq76942_data.cell_overvoltage;


            g_lithium.measurements.bq76942_valid =
                    true;


            last_bq76942_measurement_ms =
                    now;


            updated =
                    true;


            Lithium_ClearFault(
                    LITHIUM_FAULT_BQ76942_COMM);
        }
    }


    if (!updated)
    {
        return;
    }


    g_lithium.last_measurement_time_ms =
            now;


    Lithium_UpdateTemperatureValidation();


    Lithium_UpdatePowerSource();


    Lithium_UpdateVoltageFaults();


    Lithium_UpdateThermalFaults();


    Lithium_UpdateInputFaults();


    Lithium_UpdatePackVoltageCrossCheck();


    Lithium_ApplyPowerPathSafety();


    Lithium_UpdateStatusIndication();
}


/* ============================================================
 * MEASUREMENT FRESHNESS
 * ============================================================ */

static void Lithium_UpdateMeasurementFreshness(
        uint32_t now)
{
    if (g_lithium.bq25750_initialized)
    {
        if ((now -
             last_bq25750_measurement_ms) >
            LITHIUM_DATA_STALE_FAULT_MS)
        {
            g_lithium.measurements.bq25750_valid =
                    false;


            if (bq25750_comm_fail_count <
                255U)
            {
                bq25750_comm_fail_count++;
            }


            if (bq25750_comm_fail_count >=
                LITHIUM_COMM_FAILURE_CONFIRM_COUNT)
            {
                Lithium_SetFault(
                        LITHIUM_FAULT_BQ25750_COMM,
                        FAULT_SEVERITY_RECOVERABLE);
            }
        }
    }


    if (g_lithium.bq76942_initialized)
    {
        if ((now -
             last_bq76942_measurement_ms) >
            LITHIUM_DATA_STALE_FAULT_MS)
        {
            g_lithium.measurements.bq76942_valid =
                    false;


            if (bq76942_comm_fail_count <
                255U)
            {
                bq76942_comm_fail_count++;
            }


            if (bq76942_comm_fail_count >=
                LITHIUM_COMM_FAILURE_CONFIRM_COUNT)
            {
                Lithium_SetFault(
                        LITHIUM_FAULT_BQ76942_COMM,
                        FAULT_SEVERITY_RECOVERABLE);
            }
        }
    }


    g_lithium.measurements.valid =
            g_lithium.measurements.bq25750_valid &&
            g_lithium.measurements.bq76942_valid;
}


/* ============================================================
 * TEMPERATURE VALIDATION
 * ============================================================ */

static void Lithium_UpdateTemperatureValidation(void)
{
    uint8_t i;
    uint8_t j;

    int16_t raw_temperature;
    int16_t reference_sum;
    int16_t reference_temperature;

    uint8_t reference_count;


    /* --------------------------------------------------------
     * CELL-GROUP THERMISTORS
     * -------------------------------------------------------- */

    for (i = 0U;
         i < 3U;
         i++)
    {
        raw_temperature =
                g_lithium.measurements.temperature_dC[i];


        if (!Lithium_TemperaturePlausible(
                raw_temperature))
        {
            g_lithium.measurements.temperature_status[i] =
                    TEMP_SENSOR_STATUS_FAULT;


            continue;
        }


        /*
         * First usable sample.
         */
        if (!accepted_cell_temperature_valid[i])
        {
            accepted_cell_temperature_dC[i] =
                    raw_temperature;


            accepted_cell_temperature_valid[i] =
                    true;


            cell_temperature_step_count[i] =
                    0U;


            g_lithium.measurements.temperature_status[i] =
                    TEMP_SENSOR_STATUS_VALID;


            continue;
        }


        /*
         * Fast one-sample step.
         *
         * A bulk 18650 cell cannot realistically move 5 C in
         * 100 ms, therefore this is initially treated as noise.
         */
        if (Lithium_AbsTemperatureDifference(
                raw_temperature,
                accepted_cell_temperature_dC[i]) >
            LITHIUM_TEMP_MAX_SINGLE_STEP_dC)
        {
            if (cell_temperature_step_count[i] <
                255U)
            {
                cell_temperature_step_count[i]++;
            }


            if (cell_temperature_step_count[i] <
                LITHIUM_TEMP_NOISE_FILTER_COUNT)
            {
                g_lithium.measurements.temperature_dC[i] =
                        accepted_cell_temperature_dC[i];


                g_lithium.measurements.temperature_status[i] =
                        TEMP_SENSOR_STATUS_SUSPECT;


                continue;
            }
        }
        else
        {
            cell_temperature_step_count[i] =
                    0U;
        }


        /*
         * Persistent step is accepted.
         *
         * This prevents the filter from permanently hiding a
         * genuine fast temperature rise.
         */
        accepted_cell_temperature_dC[i] =
                raw_temperature;


        g_lithium.measurements.temperature_dC[i] =
                accepted_cell_temperature_dC[i];


        g_lithium.measurements.temperature_status[i] =
                TEMP_SENSOR_STATUS_VALID;
    }


    /* --------------------------------------------------------
     * PACK-CENTER BACKUP THERMISTOR
     * -------------------------------------------------------- */

    raw_temperature =
            g_lithium.measurements.pack_center_temperature_dC;


    if ((!g_lithium.measurements.pack_center_temperature_valid) ||
        (!Lithium_TemperaturePlausible(
                raw_temperature)))
    {
        g_lithium.measurements.pack_center_temperature_status =
                TEMP_SENSOR_STATUS_FAULT;
    }
    else
    {
        if (!accepted_pack_temperature_valid)
        {
            accepted_pack_temperature_dC =
                    raw_temperature;


            accepted_pack_temperature_valid =
                    true;


            pack_temperature_step_count =
                    0U;


            g_lithium.measurements.pack_center_temperature_status =
                    TEMP_SENSOR_STATUS_VALID;
        }
        else
        {
            if (Lithium_AbsTemperatureDifference(
                    raw_temperature,
                    accepted_pack_temperature_dC) >
                LITHIUM_TEMP_MAX_SINGLE_STEP_dC)
            {
                if (pack_temperature_step_count <
                    255U)
                {
                    pack_temperature_step_count++;
                }


                if (pack_temperature_step_count <
                    LITHIUM_TEMP_NOISE_FILTER_COUNT)
                {
                    g_lithium.measurements.pack_center_temperature_dC =
                            accepted_pack_temperature_dC;


                    g_lithium.measurements.pack_center_temperature_status =
                            TEMP_SENSOR_STATUS_SUSPECT;
                }
                else
                {
                    accepted_pack_temperature_dC =
                            raw_temperature;


                    g_lithium.measurements.pack_center_temperature_dC =
                            accepted_pack_temperature_dC;


                    g_lithium.measurements.pack_center_temperature_status =
                            TEMP_SENSOR_STATUS_VALID;
                }
            }
            else
            {
                pack_temperature_step_count =
                        0U;


                accepted_pack_temperature_dC =
                        raw_temperature;


                g_lithium.measurements.pack_center_temperature_dC =
                        accepted_pack_temperature_dC;


                g_lithium.measurements.pack_center_temperature_status =
                        TEMP_SENSOR_STATUS_VALID;
            }
        }
    }


    /* --------------------------------------------------------
     * CROSS-CHECK EACH CELL THERMISTOR
     * -------------------------------------------------------- */

    for (i = 0U;
         i < 3U;
         i++)
    {
        if (g_lithium.measurements.temperature_status[i] ==
            TEMP_SENSOR_STATUS_FAULT)
        {
            continue;
        }


        reference_sum =
                0;


        reference_count =
                0U;


        for (j = 0U;
             j < 3U;
             j++)
        {
            if (j == i)
            {
                continue;
            }


            if (g_lithium.measurements.temperature_status[j] !=
                TEMP_SENSOR_STATUS_FAULT)
            {
                reference_sum +=
                        g_lithium.measurements.temperature_dC[j];


                reference_count++;
            }
        }


        if (g_lithium.measurements.pack_center_temperature_status !=
            TEMP_SENSOR_STATUS_FAULT)
        {
            reference_sum +=
                    g_lithium.measurements.pack_center_temperature_dC;


            reference_count++;
        }


        if (reference_count == 0U)
        {
            continue;
        }


        reference_temperature =
                (int16_t)(
                        reference_sum /
                        (int16_t)reference_count);


        if (Lithium_AbsTemperatureDifference(
                g_lithium.measurements.temperature_dC[i],
                reference_temperature) >
            LITHIUM_TEMP_SENSOR_DISAGREEMENT_dC)
        {
            if (cell_temperature_disagreement_count[i] <
                255U)
            {
                cell_temperature_disagreement_count[i]++;
            }


            if (cell_temperature_disagreement_count[i] >=
                LITHIUM_TEMP_SENSOR_FAULT_CONFIRM_COUNT)
            {
                g_lithium.measurements.temperature_status[i] =
                        TEMP_SENSOR_STATUS_FAULT;
            }
            else
            {
                g_lithium.measurements.temperature_status[i] =
                        TEMP_SENSOR_STATUS_SUSPECT;
            }
        }
        else
        {
            cell_temperature_disagreement_count[i] =
                    0U;


            if (g_lithium.measurements.temperature_status[i] !=
                TEMP_SENSOR_STATUS_SUSPECT)
            {
                g_lithium.measurements.temperature_status[i] =
                        TEMP_SENSOR_STATUS_VALID;
            }
        }
    }


    /* --------------------------------------------------------
     * CROSS-CHECK PACK-CENTER SENSOR
     * -------------------------------------------------------- */

    if (g_lithium.measurements.pack_center_temperature_status !=
        TEMP_SENSOR_STATUS_FAULT)
    {
        reference_sum =
                0;


        reference_count =
                0U;


        for (i = 0U;
             i < 3U;
             i++)
        {
            if (g_lithium.measurements.temperature_status[i] !=
                TEMP_SENSOR_STATUS_FAULT)
            {
                reference_sum +=
                        g_lithium.measurements.temperature_dC[i];


                reference_count++;
            }
        }


        if (reference_count > 0U)
        {
            reference_temperature =
                    (int16_t)(
                            reference_sum /
                            (int16_t)reference_count);


            if (Lithium_AbsTemperatureDifference(
                    g_lithium.measurements.pack_center_temperature_dC,
                    reference_temperature) >
                LITHIUM_TEMP_SENSOR_DISAGREEMENT_dC)
            {
                if (pack_temperature_disagreement_count <
                    255U)
                {
                    pack_temperature_disagreement_count++;
                }


                if (pack_temperature_disagreement_count >=
                    LITHIUM_TEMP_SENSOR_FAULT_CONFIRM_COUNT)
                {
                    g_lithium.measurements.pack_center_temperature_status =
                            TEMP_SENSOR_STATUS_FAULT;
                }
                else
                {
                    g_lithium.measurements.pack_center_temperature_status =
                            TEMP_SENSOR_STATUS_SUSPECT;
                }
            }
            else
            {
                pack_temperature_disagreement_count =
                        0U;


                if (g_lithium.measurements.pack_center_temperature_status !=
                    TEMP_SENSOR_STATUS_SUSPECT)
                {
                    g_lithium.measurements.pack_center_temperature_status =
                            TEMP_SENSOR_STATUS_VALID;
                }
            }
        }
    }


    /* --------------------------------------------------------
     * SENSOR FAULT AGGREGATION
     *
     * One failed cell-group thermistor is tolerated in a
     * degraded mode only when the other two cell-group sensors
     * AND the independent pack-center sensor remain healthy.
     *
     * Two failed cell-group sensors, or one failed group sensor
     * together with a failed pack-center backup, blocks charging.
     * -------------------------------------------------------- */

    {
        uint8_t failed_cell_temperature_sensors = 0U;
        uint8_t valid_cell_temperature_sensors = 0U;
        uint8_t total_valid_temperature_sources;
        bool pack_center_valid;


        for (i = 0U;
             i < 3U;
             i++)
        {
            if (g_lithium.measurements.temperature_status[i] ==
                TEMP_SENSOR_STATUS_FAULT)
            {
                failed_cell_temperature_sensors++;
            }
            else if (g_lithium.measurements.temperature_status[i] ==
                     TEMP_SENSOR_STATUS_VALID)
            {
                valid_cell_temperature_sensors++;
            }
        }


        pack_center_valid =
                (g_lithium.measurements.pack_center_temperature_status ==
                 TEMP_SENSOR_STATUS_VALID);


        total_valid_temperature_sources =
                valid_cell_temperature_sensors +
                (pack_center_valid ? 1U : 0U);


        /*
         * Charging and balancing both require at least three
         * explicitly VALID temperature sources.  SUSPECT is not
         * counted as trusted redundancy.
         *
         * Exactly one failed local thermistor remains a WARNING
         * only when the other two local sensors and the independent
         * pack-center sensor are all explicitly VALID.
         */
        if ((failed_cell_temperature_sensors == 1U) &&
            (valid_cell_temperature_sensors == 2U) &&
            pack_center_valid)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_TEMP_SENSOR,
                    FAULT_SEVERITY_WARNING);
        }
        else if (total_valid_temperature_sources < 3U)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_TEMP_SENSOR,
                    FAULT_SEVERITY_RECOVERABLE);
        }
        else
        {
            Lithium_ClearFault(
                    LITHIUM_FAULT_TEMP_SENSOR);
        }
    }


    if (g_lithium.measurements.pack_center_temperature_status ==
        TEMP_SENSOR_STATUS_FAULT)
    {
        /*
         * Losing only the backup sensor is a warning while all
         * three local group sensors remain available.
         *
         * If a local sensor is also lost, TEMP_SENSOR above is
         * escalated to RECOVERABLE and charging is blocked.
         */
        Lithium_SetFault(
                LITHIUM_FAULT_PACK_TEMP_BACKUP,
                FAULT_SEVERITY_WARNING);
    }
    else
    {
        Lithium_ClearFault(
                LITHIUM_FAULT_PACK_TEMP_BACKUP);
    }
}


/* ============================================================
 * THERMAL FAULT VOTING
 * ============================================================ */

static void Lithium_UpdateThermalFaults(void)
{
    uint8_t hot_sources;
    uint8_t cold_sources;

    uint8_t warm_recovery_sources;
    uint8_t cold_recovery_sources;


    hot_sources =
            Lithium_CountHotTemperatureSources();


    cold_sources =
            Lithium_CountColdTemperatureSources();


    /* --------------------------------------------------------
     * HOT
     * -------------------------------------------------------- */

    if (hot_sources >= 2U)
    {
        hot_single_sensor_count =
                0U;


        if (hot_cross_confirm_count <
            255U)
        {
            hot_cross_confirm_count++;
        }


        if (hot_cross_confirm_count >=
            LITHIUM_CROSS_CONFIRMED_TEMP_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_TEMP_HIGH,
                    FAULT_SEVERITY_RECOVERABLE);
        }
    }
    else if (hot_sources == 1U)
    {
        hot_cross_confirm_count =
                0U;


        if (hot_single_sensor_count <
            255U)
        {
            hot_single_sensor_count++;
        }


        /*
         * One sensor alone must remain high for ~1 s before
         * charging is stopped as a genuine thermal event.
         *
         * This is intentionally separate from a sensor-fault
         * disagreement.
         */
        if (hot_single_sensor_count >=
            LITHIUM_SINGLE_TEMP_EVENT_CONFIRM_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_TEMP_HIGH,
                    FAULT_SEVERITY_RECOVERABLE);
        }
    }
    else
    {
        hot_cross_confirm_count =
                0U;


        hot_single_sensor_count =
                0U;
    }


    /* --------------------------------------------------------
     * CRITICAL HOT ESCALATION
     *
     * 55 degC is the recoverable charge-stop boundary.
     *
     * If that already-confirmed thermal event continues to
     * rise to 60 degC, the same TEMP_HIGH fault is escalated
     * to LATCHED.  Reusing the same fault bit preserves the
     * existing fault taxonomy while severity distinguishes
     * "hot - recoverable" from "critical hot - latched".
     *
     * The escalation is intentionally allowed only after the
     * normal hot voting / persistence logic has confirmed the
     * event, so one noisy 60-degC sample cannot latch the pack.
     * -------------------------------------------------------- */

    if ((((g_lithium.recoverable_fault_flags |
           g_lithium.latched_fault_flags) &
          LITHIUM_FAULT_TEMP_HIGH) != 0U) &&
        (Lithium_GetHighestTrustedTemperaturedC() >=
         LITHIUM_TEMP_SEVERE_HIGH_dC))
    {
        Lithium_SetFault(
                LITHIUM_FAULT_TEMP_HIGH,
                FAULT_SEVERITY_LATCHED);
    }


    /* --------------------------------------------------------
     * COLD
     * -------------------------------------------------------- */

    if (cold_sources >= 2U)
    {
        cold_single_sensor_count =
                0U;


        if (cold_cross_confirm_count <
            255U)
        {
            cold_cross_confirm_count++;
        }


        if (cold_cross_confirm_count >=
            LITHIUM_CROSS_CONFIRMED_TEMP_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_TEMP_LOW,
                    FAULT_SEVERITY_RECOVERABLE);
        }
    }
    else if (cold_sources == 1U)
    {
        cold_cross_confirm_count =
                0U;


        if (cold_single_sensor_count <
            255U)
        {
            cold_single_sensor_count++;
        }


        if (cold_single_sensor_count >=
            LITHIUM_SINGLE_TEMP_EVENT_CONFIRM_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_TEMP_LOW,
                    FAULT_SEVERITY_RECOVERABLE);
        }
    }
    else
    {
        cold_cross_confirm_count =
                0U;


        cold_single_sensor_count =
                0U;
    }


    /* --------------------------------------------------------
     * HOT RECOVERY
     * -------------------------------------------------------- */

    warm_recovery_sources =
            Lithium_CountTemperatureSourcesBelow(
                    LITHIUM_TEMP_HOT_RECOVER_dC);


    if ((g_lithium.recoverable_fault_flags &
         LITHIUM_FAULT_TEMP_HIGH) != 0U)
    {
        if (warm_recovery_sources >= 3U)
        {
            if (hot_recovery_count <
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                hot_recovery_count++;
            }


            if (hot_recovery_count >=
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                Lithium_ClearFault(
                        LITHIUM_FAULT_TEMP_HIGH);


                hot_recovery_count =
                        0U;
            }
        }
        else
        {
            hot_recovery_count =
                    0U;
        }
    }


    /* --------------------------------------------------------
     * COLD RECOVERY
     * -------------------------------------------------------- */

    cold_recovery_sources =
            Lithium_CountTemperatureSourcesAbove(
                    LITHIUM_TEMP_COLD_RECOVER_dC);


    if ((g_lithium.recoverable_fault_flags &
         LITHIUM_FAULT_TEMP_LOW) != 0U)
    {
        if (cold_recovery_sources >= 3U)
        {
            if (cold_recovery_count <
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                cold_recovery_count++;
            }


            if (cold_recovery_count >=
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                Lithium_ClearFault(
                        LITHIUM_FAULT_TEMP_LOW);


                cold_recovery_count =
                        0U;
            }
        }
        else
        {
            cold_recovery_count =
                    0U;
        }
    }
}


/* ============================================================
 * VOLTAGE FAULTS
 * ============================================================ */

static void Lithium_UpdateVoltageFaults(void)
{
    uint16_t min_cell_mv;

    uint16_t max_cell_mv;


    if (!g_lithium.measurements.bq76942_valid)
    {
        return;
    }


    min_cell_mv =
            Lithium_GetMinCellMv();


    max_cell_mv =
            Lithium_GetMaxCellMv();


    /* --------------------------------------------------------
     * CELL OVERVOLTAGE
     *
     * Layer 1: software soft-high guard at 4.170 V.
     *          -> WARNING, charging inhibited by Charger FSM.
     *
     * Layer 2: independent BQ76942 COV comparator/status at
     *          approximately 4.20 V / 500 ms.
     *          -> WARNING plus an independent charger inhibit.
     *             Balancing may remain active if otherwise safe.
     *
     * Recovery from either non-latched OV path requires BOTH:
     * - BQ76942 COV status cleared, and
     * - max cell <= 4.150 V.
     * -------------------------------------------------------- */

    if (g_lithium.measurements.bq76942_cov_active)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_CELL_OVERVOLTAGE,
                FAULT_SEVERITY_WARNING);
    }
    else if (max_cell_mv >=
             LITHIUM_CELL_CHARGE_INHIBIT_MV)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_CELL_OVERVOLTAGE,
                FAULT_SEVERITY_WARNING);
    }
    else if (max_cell_mv <=
             LITHIUM_CELL_CHARGE_RECOVER_MV)
    {
        Lithium_ClearFault(
                LITHIUM_FAULT_CELL_OVERVOLTAGE);
    }


    /* --------------------------------------------------------
     * SEVERE OV
     * -------------------------------------------------------- */

    if (max_cell_mv >=
        LITHIUM_CELL_OV_LATCH_MV)
    {
        if (severe_ov_count <
            255U)
        {
            severe_ov_count++;
        }


        if (severe_ov_count >=
            LITHIUM_MONITOR_CONFIRM_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_CELL_OVERVOLTAGE,
                    FAULT_SEVERITY_LATCHED);
        }
    }
    else
    {
        severe_ov_count =
                0U;
    }


    /* --------------------------------------------------------
     * CELL UNDERVOLTAGE / BQ76942 CUV
     *
     * The independent BQ76942 CUV comparator is deliberately
     * treated as a WARNING, not a blocking recoverable fault.
     * This preserves the designed 2.50...3.00-V low-cell
     * recovery path while still reporting that the hardware
     * UV comparator has corroborated the low-cell condition.
     * -------------------------------------------------------- */

    if ((min_cell_mv <
         LITHIUM_CELL_UV_WARNING_MV) ||
        g_lithium.measurements.bq76942_cuv_active)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_CELL_UNDERVOLTAGE,
                FAULT_SEVERITY_WARNING);
    }
    else if ((min_cell_mv >=
              LITHIUM_CELL_UV_WARNING_RECOVER_MV) &&
             (!g_lithium.measurements.bq76942_cuv_active))
    {
        Lithium_ClearFault(
                LITHIUM_FAULT_CELL_UNDERVOLTAGE);
    }


    /* --------------------------------------------------------
     * DEEP UV
     * -------------------------------------------------------- */

    if (min_cell_mv <
        LITHIUM_CELL_UV_LATCH_MV)
    {
        if (severe_uv_count <
            255U)
        {
            severe_uv_count++;
        }


        if (severe_uv_count >=
            LITHIUM_MONITOR_CONFIRM_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE,
                    FAULT_SEVERITY_LATCHED);
        }
    }
    else
    {
        severe_uv_count =
                0U;
    }
}


/* ============================================================
 * INPUT FAULTS
 * ============================================================ */

static void Lithium_UpdateInputFaults(void)
{
    bool input_low_warning;


    if (!g_lithium.measurements.bq25750_valid)
    {
        return;
    }


    /* --------------------------------------------------------
     * Preserve the exact live BQ25750 REG0x24 protection cause
     * for CAN diagnostics.
     *
     * These bits do NOT replace the existing global fault
     * classes.  They only prevent different hardware events
     * from being collapsed into the same generic text.
     * -------------------------------------------------------- */

    if (last_bq25750_data.input_undervoltage)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_BQ_VAC_UV;
    }
    else
    {
        g_lithium.charger_diagnostic_flags &=
                (uint16_t)~LITHIUM_CHARGER_DIAG_BQ_VAC_UV;
    }


    if (last_bq25750_data.input_overvoltage)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_BQ_VAC_OV;
    }
    else
    {
        g_lithium.charger_diagnostic_flags &=
                (uint16_t)~LITHIUM_CHARGER_DIAG_BQ_VAC_OV;
    }


    if (last_bq25750_data.battery_overcurrent)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_BQ_IBAT_OCP;


        /*
         * BQ25750 BAT_OCP is a charger-side protection event.
         * Keep the existing recoverable charger policy; do not
         * reinterpret it as the separate 19.5-A discharge-side
         * software overcurrent latch.
         */
        Lithium_SetFault(
                LITHIUM_FAULT_CHARGER,
                FAULT_SEVERITY_RECOVERABLE);
    }
    else
    {
        g_lithium.charger_diagnostic_flags &=
                (uint16_t)~LITHIUM_CHARGER_DIAG_BQ_IBAT_OCP;
    }


    if (last_bq25750_data.battery_overvoltage)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_BQ_VBAT_OV;


        /*
         * The BQ25750 pack-feedback BAT_OVP is an independent
         * charger backup.  Keep the established non-latched OV
         * behavior: charging is inhibited while safe balancing
         * may continue.  The cell-level 4.250-V software path
         * remains the condition that latches CELL_OVERVOLTAGE.
         */
        Lithium_SetFault(
                LITHIUM_FAULT_CELL_OVERVOLTAGE,
                FAULT_SEVERITY_WARNING);
    }
    else
    {
        g_lithium.charger_diagnostic_flags &=
                (uint16_t)~LITHIUM_CHARGER_DIAG_BQ_VBAT_OV;
    }


    if (last_bq25750_data.thermal_shutdown)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_BQ_TSHUT;
    }
    else
    {
        g_lithium.charger_diagnostic_flags &=
                (uint16_t)~LITHIUM_CHARGER_DIAG_BQ_TSHUT;
    }


    /* --------------------------------------------------------
     * HIGH INPUT
     *
     * Existing software policy is preserved:
     *
     * >= 30 V  -> recoverable + HIZ request
     * >= 27 V  -> warning
     *
     * In addition, an asserted BQ25750 VAC_OV protection status
     * is treated as a recoverable input-OV event because the
     * hardware charger has already entered its OVP protection.
     * -------------------------------------------------------- */

    if ((g_lithium.measurements.vin_mv >=
         LITHIUM_INPUT_HIZ_REQUEST_MV) ||
        last_bq25750_data.input_overvoltage)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_INPUT_OVERVOLTAGE,
                FAULT_SEVERITY_RECOVERABLE);
    }
    else if (g_lithium.input_isolated)
    {
        /*
         * Keep the recoverable fault active while the device
         * remains in HIZ.  Lithium_ApplyPowerPathSafety() owns
         * the controlled HIZ release sequence.
         */
    }
    else if (g_lithium.measurements.vin_mv >=
             LITHIUM_INPUT_HIGH_WARNING_MV)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_INPUT_OVERVOLTAGE,
                FAULT_SEVERITY_WARNING);
    }
    else
    {
        Lithium_ClearFault(
                LITHIUM_FAULT_INPUT_OVERVOLTAGE);
    }


    /* --------------------------------------------------------
     * LOW INPUT
     *
     * Report either the existing software <21-V warning or the
     * BQ25750 VAC_UV protection status.  Hardware VAC_DPM/UVP
     * remains responsible for reducing/stopping charge demand.
     * -------------------------------------------------------- */

    input_low_warning =
            ((g_lithium.input_present &&
              (g_lithium.measurements.vin_mv <
               LITHIUM_INPUT_LOW_WARNING_MV)) ||
             last_bq25750_data.input_undervoltage);


    if (input_low_warning)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_INPUT_POWER,
                FAULT_SEVERITY_WARNING);
    }
    else
    {
        Lithium_ClearFault(
                LITHIUM_FAULT_INPUT_POWER);
    }


    /* --------------------------------------------------------
     * CHARGE / CV TIMER BACKUPS
     * -------------------------------------------------------- */

    if (last_bq25750_data.charge_timer_expired)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_FAST_CHARGE_TIMEOUT;


        Lithium_SetFault(
                LITHIUM_FAULT_CHARGE_TIMEOUT,
                FAULT_SEVERITY_LATCHED);
    }


    if (last_bq25750_data.cv_timer_expired)
    {
        g_lithium.charger_diagnostic_flags |=
                LITHIUM_CHARGER_DIAG_CV_TIMEOUT;


        Lithium_SetFault(
                LITHIUM_FAULT_CHARGE_TIMEOUT,
                FAULT_SEVERITY_LATCHED);
    }


    /* --------------------------------------------------------
     * BQ25750 JUNCTION THERMAL SHUTDOWN
     * -------------------------------------------------------- */

    if (last_bq25750_data.thermal_shutdown)
    {
        Lithium_SetFault(
                LITHIUM_FAULT_TEMP_HIGH,
                FAULT_SEVERITY_RECOVERABLE);
    }
}


/* ============================================================
 * CHARGER FSM FAULT MAPPING
 *
 * Charger FSM records the exact local reason only.
 * This function, owned by the application layer, decides the
 * global fault severity and therefore preserves the architecture
 * rule that subordinate FSMs do not own system_state.
 * ============================================================ */

static void Lithium_UpdateChargerFaults(void)
{
    uint16_t diagnostics;


    diagnostics =
            g_lithium.charger_diagnostic_flags;


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_LOW_CELL_TIMEOUT) != 0U)
    {
        /*
         * A 30-minute low-cell recovery attempt has failed to
         * return the pack to the normal operating region.
         * Automatic repeated recovery attempts are forbidden.
         */
        Lithium_SetFault(
                LITHIUM_FAULT_CHARGE_TIMEOUT,
                FAULT_SEVERITY_LATCHED);
    }


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_RECOVERABLE_MASK) != 0U)
    {
        /*
         * BQ watchdog/device/configuration failures are treated
         * as controlled recoverable charger faults.  Recovery is
         * performed by reinitializing/qualifying the BQ25750 and
         * then passing through SELF_TEST before charging resumes.
         */
        Lithium_SetFault(
                LITHIUM_FAULT_CHARGER,
                FAULT_SEVERITY_RECOVERABLE);
    }
}


/* ============================================================
 * PACK-VOLTAGE CROSS CHECK
 * ============================================================ */

static void Lithium_UpdatePackVoltageCrossCheck(void)
{
    uint16_t difference_mv;


    if ((!g_lithium.measurements.bq25750_valid) ||
        (!g_lithium.measurements.bq76942_valid))
    {
        return;
    }


    if (g_lithium.measurements.pack_mv >=
        g_lithium.measurements.charger_vbat_mv)
    {
        difference_mv =
                (uint16_t)(
                        g_lithium.measurements.pack_mv -
                        g_lithium.measurements.charger_vbat_mv);
    }
    else
    {
        difference_mv =
                (uint16_t)(
                        g_lithium.measurements.charger_vbat_mv -
                        g_lithium.measurements.pack_mv);
    }


    if (difference_mv >
        LITHIUM_PACK_VOLTAGE_TOLERANCE_MV)
    {
        pack_voltage_match_count =
                0U;


        if (pack_voltage_mismatch_count <
            255U)
        {
            pack_voltage_mismatch_count++;
        }


        if (pack_voltage_mismatch_count >=
            LITHIUM_PACK_XCHECK_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_PACK_VOLTAGE_MISMATCH,
                    FAULT_SEVERITY_RECOVERABLE);
        }
    }
    else
    {
        pack_voltage_mismatch_count =
                0U;


        if (pack_voltage_match_count <
            LITHIUM_MONITOR_CONFIRM_COUNT)
        {
            pack_voltage_match_count++;
        }


        if (pack_voltage_match_count >=
            LITHIUM_MONITOR_CONFIRM_COUNT)
        {
            Lithium_ClearFault(
                    LITHIUM_FAULT_PACK_VOLTAGE_MISMATCH);
        }
    }
}


/* ============================================================
 * POWER SOURCE / OPERATING MODE
 * ============================================================ */

static void Lithium_UpdatePowerSource(void)
{
    bool pg_pin_low;


    pg_pin_low =
            (HAL_GPIO_ReadPin(
                    BQ_PG_GPIO_Port,
                    BQ_PG_Pin) ==
             GPIO_PIN_RESET);


    /*
     * BQ25750 PG is an autonomous, active-low indication that
     * the input source is valid and the device is not in HIZ.
     * It does not depend on a successful I2C transaction.
     *
     * The ADC VIN value and internal PG_STAT remain diagnostic
     * cross-checks, but they are not prerequisites for deciding
     * which source is actually available to the direct power
     * path.
     */
    if (pg_pin_low)
    {
        g_lithium.input_present =
                true;


        g_lithium.power_source =
                POWER_INPUT;


        g_lithium.operating_mode =
                OPERATING_MODE_GROUND;
    }
    else
    {
        g_lithium.input_present =
                false;


        g_lithium.power_source =
                POWER_BATTERY;


        g_lithium.operating_mode =
                OPERATING_MODE_FLIGHT;
    }
}


/* ============================================================
 * FAST POWER SAFETY
 * ============================================================ */

static void Lithium_RunFastPowerSafety(
        uint32_t now)
{
    int32_t battery_current_ma;


    if ((now -
         last_fast_power_safety_ms) <
        LITHIUM_FAST_POWER_SAFETY_PERIOD_MS)
    {
        return;
    }


    last_fast_power_safety_ms =
            now;


    if (!g_lithium.measurements.bq25750_valid)
    {
        return;
    }


    battery_current_ma =
            g_lithium.measurements.battery_current_ma;


    if (battery_current_ma < 0)
    {
        battery_current_ma =
                -battery_current_ma;
    }


    if (battery_current_ma >=
        LITHIUM_SW_OVERCURRENT_ADC_LIMIT_MA)
    {
        if (overcurrent_count <
            255U)
        {
            overcurrent_count++;
        }


        if (overcurrent_count >=
            LITHIUM_SW_OVERCURRENT_CONFIRM_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_BATTERY_OVERCURRENT,
                    FAULT_SEVERITY_LATCHED);


            /*
             * Ground:
             *
             * Umbilical is available, so isolate battery
             * immediately if BQ25750 communication still works.
             *
             * Flight:
             *
             * Do NOT intentionally kill all avionics at the
             * ~20-A ADC saturation boundary. The external
             * BIFROST SF-1206HHA800R-2 fuse is 8-A rated, but
             * 8...10 A is not treated as a firmware fault.
             * The fuse remains the primary physical protection
             * for sustained high-current / short-circuit events;
             * this software threshold is only an extreme-current
             * diagnostic near the usable IBAT ADC range.
             */

            if ((g_lithium.operating_mode ==
                 OPERATING_MODE_GROUND) &&
                g_lithium.input_present)
            {
                if (BQ25750_ForceBatteryFetOff(
                        true) ==
                    BQ_STATUS_OK)
                {
                    g_lithium.battery_isolated =
                            true;
                }
            }


            /*
             * Charging is always removed immediately.
             */
            HAL_GPIO_WritePin(
                    BQ_CE_GPIO_Port,
                    BQ_CE_Pin,
                    GPIO_PIN_SET);
        }
    }
    else
    {
        overcurrent_count =
                0U;
    }
}


/* ============================================================
 * BQ WATCHDOG
 * ============================================================ */

static void Lithium_ServiceBQWatchdog(
        uint32_t now)
{
    if (!g_lithium.bq25750_initialized)
    {
        return;
    }


    if ((now -
         last_watchdog_service_ms) <
        LITHIUM_BQ25750_WATCHDOG_SERVICE_MS)
    {
        return;
    }


    last_watchdog_service_ms =
            now;


    g_lithium.bq25750_status =
            BQ25750_ServiceWatchdog();


    if (g_lithium.bq25750_status !=
        BQ_STATUS_OK)
    {
        if (bq25750_comm_fail_count <
            255U)
        {
            bq25750_comm_fail_count++;
        }


        if (bq25750_comm_fail_count >=
            LITHIUM_COMM_FAILURE_CONFIRM_COUNT)
        {
            Lithium_SetFault(
                    LITHIUM_FAULT_BQ25750_COMM,
                    FAULT_SEVERITY_RECOVERABLE);
        }
    }
}


/* ============================================================
 * POWER-PATH SAFETY POLICY
 * ============================================================ */

static void Lithium_ApplyPowerPathSafety(void)
{
    uint32_t severe_battery_faults;

    bool severe_temperature_high;
    bool ground_battery_isolation_required;


    severe_battery_faults =
            g_lithium.latched_fault_flags &
            (LITHIUM_FAULT_CELL_OVERVOLTAGE |
             LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE |
             LITHIUM_FAULT_BATTERY_OVERCURRENT);


    /*
     * The 60-C ground-isolation policy is applied only after
     * TEMP_HIGH has already passed the normal thermal voting /
     * persistence logic. This prevents one unconfirmed sample
     * from disconnecting the battery.
     */
    severe_temperature_high =
            ((((g_lithium.recoverable_fault_flags |
                g_lithium.latched_fault_flags) &
               LITHIUM_FAULT_TEMP_HIGH) != 0U) &&
             (Lithium_GetHighestTrustedTemperaturedC() >=
              LITHIUM_TEMP_SEVERE_HIGH_dC));


    ground_battery_isolation_required =
            ((g_lithium.operating_mode ==
              OPERATING_MODE_GROUND) &&
             g_lithium.input_present &&
             g_lithium.bq25750_initialized &&
             ((severe_battery_faults != 0U) ||
              severe_temperature_high ||
              ((g_lithium.recoverable_fault_flags &
                LITHIUM_FAULT_BQ76942_COMM) != 0U)));


    /*
     * Any serious condition removes charging permission.
     */
    if ((g_lithium.recoverable_fault_flags != 0U) ||
        (g_lithium.latched_fault_flags != 0U))
    {
        HAL_GPIO_WritePin(
                BQ_CE_GPIO_Port,
                BQ_CE_Pin,
                GPIO_PIN_SET);
    }


    /* --------------------------------------------------------
     * HIGH INPUT VOLTAGE / HIZ
     * -------------------------------------------------------- */

    if (g_lithium.measurements.bq25750_valid &&
        (g_lithium.measurements.vin_mv >=
         LITHIUM_INPUT_HIZ_REQUEST_MV))
    {
        input_hiz_recovery_count =
                0U;


        input_hiz_recovery_sample_ms =
                last_bq25750_measurement_ms;


        if (BQ25750_SetInputHighImpedance(
                true) ==
            BQ_STATUS_OK)
        {
            g_lithium.input_isolated =
                    true;
        }
    }
    else if (g_lithium.input_isolated)
    {
        /*
         * Automatic HIZ recovery is allowed only after three
         * fresh BQ25750 samples confirm VIN below the 27-V
         * warning boundary. Repeated calls using the same cached
         * sample do not advance this counter.
         */
        if (g_lithium.measurements.bq25750_valid &&
            (g_lithium.measurements.vin_mv <
             LITHIUM_INPUT_HIGH_WARNING_MV))
        {
            if (last_bq25750_measurement_ms !=
                input_hiz_recovery_sample_ms)
            {
                input_hiz_recovery_sample_ms =
                        last_bq25750_measurement_ms;


                if (input_hiz_recovery_count <
                    255U)
                {
                    input_hiz_recovery_count++;
                }
            }


            if (input_hiz_recovery_count >=
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                if (BQ25750_SetInputHighImpedance(
                        false) ==
                    BQ_STATUS_OK)
                {
                    g_lithium.input_isolated =
                            false;


                    input_hiz_recovery_count =
                            0U;


                    input_hiz_recovery_sample_ms =
                            last_bq25750_measurement_ms;
                }
            }
        }
        else
        {
            input_hiz_recovery_count =
                    0U;


            input_hiz_recovery_sample_ms =
                    last_bq25750_measurement_ms;
        }
    }
    else
    {
        input_hiz_recovery_count =
                0U;


        input_hiz_recovery_sample_ms =
                last_bq25750_measurement_ms;
    }


    /* --------------------------------------------------------
     * GROUND SEVERE BATTERY FAULT
     *
     * Umbilical keeps VSYS alive.
     * -------------------------------------------------------- */

    if (ground_battery_isolation_required)
    {
        if (BQ25750_ForceBatteryFetOff(
                true) ==
            BQ_STATUS_OK)
        {
            g_lithium.battery_isolated =
                    true;
        }
    }
    else if ((g_lithium.operating_mode ==
              OPERATING_MODE_GROUND) &&
             g_lithium.input_present &&
             g_lithium.battery_isolated &&
             g_lithium.bq25750_initialized &&
             (g_lithium.system_state ==
              SYS_READY) &&
             (severe_battery_faults == 0U) &&
             ((g_lithium.recoverable_fault_flags &
               LITHIUM_FAULT_BQ76942_COMM) == 0U) &&
             g_lithium.measurements.bq25750_valid &&
             g_lithium.measurements.bq76942_valid)
    {
        /*
         * Recoverable ground-only isolation is released only
         * after the normal recovery path has completed and the
         * complete system has passed SELF_TEST back to READY.
         * Latched severe faults can never satisfy this condition.
         */
        if (BQ25750_ForceBatteryFetOff(
                false) ==
            BQ_STATUS_OK)
        {
            g_lithium.battery_isolated =
                    false;
        }
    }


    /* --------------------------------------------------------
     * FLIGHT
     *
     * Battery is needed for avionics.
     *
     * Release any previous ground-only forced isolation.
     * -------------------------------------------------------- */

    if ((g_lithium.operating_mode ==
         OPERATING_MODE_FLIGHT) &&
        g_lithium.battery_isolated &&
        g_lithium.bq25750_initialized)
    {
        if (BQ25750_ForceBatteryFetOff(
                false) ==
            BQ_STATUS_OK)
        {
            g_lithium.battery_isolated =
                    false;
        }
    }
}


/* ============================================================
 * SYSTEM FSM
 * ============================================================ */

void Lithium_SystemFSM_Run(void)
{
    uint32_t now;

    bool init_attempts_exhausted;


    now =
            HAL_GetTick();


    switch (g_lithium.system_state)
    {
        /* ----------------------------------------------------
         * BOOT
         * ---------------------------------------------------- */

        case SYS_BOOT:

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


            ChargerFSM_ForceOff(
                    &g_lithium);


            BalancingFSM_ForceOff(
                    &g_lithium);


            g_lithium.last_init_attempt_ms =
                    now -
                    LITHIUM_INIT_RETRY_DELAY_MS;


            g_lithium.system_state =
                    SYS_INIT;


            g_lithium.state_enter_time_ms =
                    now;


            Lithium_LED_SetPattern(
                    LED_PATTERN_INIT);


            break;


        /* ----------------------------------------------------
         * INIT
         * ---------------------------------------------------- */

        case SYS_INIT:

            ChargerFSM_ForceOff(
                    &g_lithium);


            BalancingFSM_ForceOff(
                    &g_lithium);


            if (g_lithium.bq25750_initialized &&
                g_lithium.bq76942_initialized)
            {
                self_test_good_count =
                        0U;


                g_lithium.system_state =
                        SYS_SELF_TEST;


                g_lithium.state_enter_time_ms =
                        now;


                Lithium_LED_SetPattern(
                        LED_PATTERN_SELF_TEST);


                break;
            }


            if ((now -
                 g_lithium.last_init_attempt_ms) <
                LITHIUM_INIT_RETRY_DELAY_MS)
            {
                break;
            }


            g_lithium.last_init_attempt_ms =
                    now;


            if ((!g_lithium.bq25750_initialized) &&
                (g_lithium.bq25750_init_attempts <
                 LITHIUM_INIT_MAX_ATTEMPTS))
            {
                g_lithium.bq25750_init_attempts++;


                g_lithium.bq25750_status =
                        BQ25750_Init();


                if (g_lithium.bq25750_status ==
                    BQ_STATUS_OK)
                {
                    g_lithium.bq25750_initialized =
                            true;


                    g_lithium.input_isolated =
                            BQ25750_IsInputHighImpedanceActive();


                    g_lithium.battery_isolated =
                            BQ25750_IsBatteryFetForcedOff();


                    bq25750_comm_fail_count =
                            0U;


                    last_bq25750_measurement_ms =
                            now;


                    last_watchdog_service_ms =
                            now;


                    Lithium_ClearFault(
                            LITHIUM_FAULT_BQ25750_INIT |
                            LITHIUM_FAULT_BQ25750_COMM);
                }
                else
                {
                    Lithium_SetFault(
                            LITHIUM_FAULT_BQ25750_INIT,
                            FAULT_SEVERITY_WARNING);
                }
            }


            if ((!g_lithium.bq76942_initialized) &&
                (g_lithium.bq76942_init_attempts <
                 LITHIUM_INIT_MAX_ATTEMPTS))
            {
                g_lithium.bq76942_init_attempts++;


                g_lithium.bq76942_status =
                        BQ76942_Init();


                if (g_lithium.bq76942_status ==
                    BQ_STATUS_OK)
                {
                    g_lithium.bq76942_initialized =
                            true;


                    bq76942_comm_fail_count =
                            0U;


                    last_bq76942_measurement_ms =
                            now;


                    Lithium_ClearFault(
                            LITHIUM_FAULT_BQ76942_INIT |
                            LITHIUM_FAULT_BQ76942_COMM);
                }
                else
                {
                    Lithium_SetFault(
                            LITHIUM_FAULT_BQ76942_INIT,
                            FAULT_SEVERITY_WARNING);
                }
            }


            init_attempts_exhausted =
                    true;


            if ((!g_lithium.bq25750_initialized) &&
                (g_lithium.bq25750_init_attempts <
                 LITHIUM_INIT_MAX_ATTEMPTS))
            {
                init_attempts_exhausted =
                        false;
            }


            if ((!g_lithium.bq76942_initialized) &&
                (g_lithium.bq76942_init_attempts <
                 LITHIUM_INIT_MAX_ATTEMPTS))
            {
                init_attempts_exhausted =
                        false;
            }


            if (init_attempts_exhausted &&
                ((!g_lithium.bq25750_initialized) ||
                 (!g_lithium.bq76942_initialized)))
            {
                Lithium_SetFault(
                        ((!g_lithium.bq25750_initialized) ?
                         LITHIUM_FAULT_BQ25750_INIT :
                         0U) |
                        ((!g_lithium.bq76942_initialized) ?
                         LITHIUM_FAULT_BQ76942_INIT :
                         0U),
                        FAULT_SEVERITY_RECOVERABLE);


                g_lithium.system_state =
                        SYS_FAULT_RECOVERABLE;


                g_lithium.state_enter_time_ms =
                        now;


                g_lithium.recovery_attempts =
                        0U;


                g_lithium.last_recovery_attempt_ms =
                        now;
            }


            break;


        /* ----------------------------------------------------
         * SELF TEST
         * ---------------------------------------------------- */

        case SYS_SELF_TEST:

            ChargerFSM_ForceOff(
                    &g_lithium);


            BalancingFSM_ForceOff(
                    &g_lithium);


            if ((now -
                 g_lithium.state_enter_time_ms) <
                LITHIUM_SELF_TEST_SETTLING_TIME_MS)
            {
                break;
            }


            if (Lithium_SelfTestSetIsGood())
            {
                if (self_test_good_count <
                    255U)
                {
                    self_test_good_count++;
                }


                if (self_test_good_count >=
                    LITHIUM_SELF_TEST_VALID_SET_COUNT)
                {
                    g_lithium.system_state =
                            SYS_READY;


                    g_lithium.state_enter_time_ms =
                            now;


                    Lithium_LED_SetPattern(
                            LED_PATTERN_READY);
                }
            }
            else
            {
                self_test_good_count =
                        0U;


                if (g_lithium.latched_fault_flags !=
                    LITHIUM_FAULT_NONE)
                {
                    g_lithium.system_state =
                            SYS_FAULT_LATCHED;


                    g_lithium.state_enter_time_ms =
                            now;
                }
                else if (g_lithium.recoverable_fault_flags !=
                         LITHIUM_FAULT_NONE)
                {
                    g_lithium.system_state =
                            SYS_FAULT_RECOVERABLE;


                    g_lithium.state_enter_time_ms =
                            now;


                    g_lithium.recovery_attempts =
                            0U;


                    g_lithium.last_recovery_attempt_ms =
                            now;
                }
            }


            break;


        /* ----------------------------------------------------
         * READY
         * ---------------------------------------------------- */

        case SYS_READY:

            Lithium_ApplyPowerPathSafety();


            ChargerFSM_Run(
                    &g_lithium);


            Lithium_UpdateChargerFaults();


            BalancingFSM_Run(
                    &g_lithium);


            if (g_lithium.latched_fault_flags !=
                LITHIUM_FAULT_NONE)
            {
                g_lithium.system_state =
                        SYS_FAULT_LATCHED;


                g_lithium.state_enter_time_ms =
                        now;


                break;
            }


            if (g_lithium.recoverable_fault_flags !=
                LITHIUM_FAULT_NONE)
            {
                g_lithium.system_state =
                        SYS_FAULT_RECOVERABLE;


                g_lithium.state_enter_time_ms =
                        now;


                g_lithium.recovery_attempts =
                        0U;


                g_lithium.last_recovery_attempt_ms =
                        now;
            }


            break;


        /* ----------------------------------------------------
         * RECOVERABLE FAULT
         * ---------------------------------------------------- */

        case SYS_FAULT_RECOVERABLE:

            ChargerFSM_ForceOff(
                    &g_lithium);


            BalancingFSM_ForceOff(
                    &g_lithium);


            Lithium_ApplyPowerPathSafety();


            if (g_lithium.latched_fault_flags !=
                LITHIUM_FAULT_NONE)
            {
                g_lithium.system_state =
                        SYS_FAULT_LATCHED;


                g_lithium.state_enter_time_ms =
                        now;


                break;
            }


            Lithium_RunBQRecovery(
                    now);


            if (g_lithium.latched_fault_flags !=
                LITHIUM_FAULT_NONE)
            {
                g_lithium.system_state =
                        SYS_FAULT_LATCHED;


                g_lithium.state_enter_time_ms =
                        now;


                break;
            }


            if (g_lithium.recoverable_fault_flags ==
                LITHIUM_FAULT_NONE)
            {
                self_test_good_count =
                        0U;


                g_lithium.system_state =
                        SYS_SELF_TEST;


                g_lithium.state_enter_time_ms =
                        now;
            }


            break;


        /* ----------------------------------------------------
         * LATCHED
         * ---------------------------------------------------- */

        case SYS_FAULT_LATCHED:

            ChargerFSM_ForceOff(
                    &g_lithium);


            BalancingFSM_ForceOff(
                    &g_lithium);


            Lithium_ApplyPowerPathSafety();


            /*
             * V1:
             *
             * Latched faults remain until MCU reset /
             * controlled power cycle.
             */
            break;


        default:

            ChargerFSM_ForceOff(
                    &g_lithium);


            BalancingFSM_ForceOff(
                    &g_lithium);


            Lithium_SetFault(
                    LITHIUM_FAULT_INTERNAL_STATE,
                    FAULT_SEVERITY_LATCHED);


            break;
    }


    Lithium_UpdateStatusIndication();
}


/* ============================================================
 * SELF TEST
 * ============================================================ */

static bool Lithium_SelfTestSetIsGood(void)
{
    if ((!g_lithium.bq25750_initialized) ||
        (!g_lithium.bq76942_initialized))
    {
        return false;
    }


    if ((!g_lithium.measurements.bq25750_valid) ||
        (!g_lithium.measurements.bq76942_valid))
    {
        return false;
    }


    if ((Lithium_GetMinCellMv() <
         LITHIUM_CELL_UV_LATCH_MV) ||
        (Lithium_GetMaxCellMv() >=
         LITHIUM_CELL_OV_LATCH_MV))
    {
        return false;
    }


    if ((g_lithium.latched_fault_flags !=
         LITHIUM_FAULT_NONE) ||
        (g_lithium.recoverable_fault_flags !=
         LITHIUM_FAULT_NONE))
    {
        return false;
    }


    return true;
}


/* ============================================================
 * BQ RECOVERY
 * ============================================================ */

static void Lithium_RunBQRecovery(
        uint32_t now)
{
    uint32_t recoverable_bq_faults;


    recoverable_bq_faults =
            g_lithium.recoverable_fault_flags &
            (LITHIUM_FAULT_BQ25750_MASK |
             LITHIUM_FAULT_BQ76942_MASK |
             LITHIUM_FAULT_CHARGER);


    /*
     * Recovery attempts in this function belong only to
     * BQ25750/BQ76942 communication-initialization faults and
     * recoverable charger-controller faults. Other recoverable
     * conditions clear through their own monitoring logic and
     * must not consume the BQ retry budget.
     */
    if (recoverable_bq_faults == 0U)
    {
        return;
    }


    if ((now -
         g_lithium.last_recovery_attempt_ms) <
        LITHIUM_RECOVERY_RETRY_PERIOD_MS)
    {
        return;
    }


    if (g_lithium.recovery_attempts >=
        LITHIUM_RECOVERY_MAX_ATTEMPTS)
    {
        Lithium_SetFault(
                recoverable_bq_faults,
                FAULT_SEVERITY_LATCHED);


        return;
    }


    g_lithium.last_recovery_attempt_ms =
            now;


    g_lithium.recovery_attempts++;


    if ((recoverable_bq_faults &
         (LITHIUM_FAULT_BQ25750_MASK |
          LITHIUM_FAULT_CHARGER)) != 0U)
    {
        g_lithium.bq25750_initialized =
                false;


        g_lithium.bq25750_status =
                BQ25750_Init();


        if (g_lithium.bq25750_status ==
            BQ_STATUS_OK)
        {
            g_lithium.bq25750_initialized =
                    true;


            g_lithium.input_isolated =
                    BQ25750_IsInputHighImpedanceActive();


            g_lithium.battery_isolated =
                    BQ25750_IsBatteryFetForcedOff();


            bq25750_comm_fail_count =
                    0U;


            last_bq25750_measurement_ms =
                    now;


            Lithium_ClearFault(
                    LITHIUM_FAULT_BQ25750_MASK |
                    LITHIUM_FAULT_CHARGER);


            g_lithium.charger_diagnostic_flags &=
                    (uint16_t)~LITHIUM_CHARGER_DIAG_RECOVERABLE_MASK;
        }
    }


    if ((recoverable_bq_faults &
         LITHIUM_FAULT_BQ76942_MASK) != 0U)
    {
        g_lithium.bq76942_initialized =
                false;


        g_lithium.bq76942_status =
                BQ76942_Init();


        if (g_lithium.bq76942_status ==
            BQ_STATUS_OK)
        {
            g_lithium.bq76942_initialized =
                    true;


            bq76942_comm_fail_count =
                    0U;


            last_bq76942_measurement_ms =
                    now;


            Lithium_ClearFault(
                    LITHIUM_FAULT_BQ76942_MASK);
        }
    }


    recoverable_bq_faults =
            g_lithium.recoverable_fault_flags &
            (LITHIUM_FAULT_BQ25750_MASK |
             LITHIUM_FAULT_BQ76942_MASK |
             LITHIUM_FAULT_CHARGER);


    if ((recoverable_bq_faults != 0U) &&
        (g_lithium.recovery_attempts >=
         LITHIUM_RECOVERY_MAX_ATTEMPTS))
    {
        Lithium_SetFault(
                recoverable_bq_faults,
                FAULT_SEVERITY_LATCHED);
    }
}


/* ============================================================
 * FAULT MANAGEMENT
 * ============================================================ */

void Lithium_SetFault(
        uint32_t fault,
        fault_severity_t severity)
{
    uint32_t now;


    if (fault == LITHIUM_FAULT_NONE)
    {
        return;
    }


    now =
            HAL_GetTick();


    g_lithium.fault_flags |=
            fault;


    switch (severity)
    {
        case FAULT_SEVERITY_WARNING:

            /*
             * Never downgrade a fault that is already active at
             * a higher severity.  This matters for conditions
             * such as TEMP_HIGH, which is recoverable at 55 C
             * but latched if the confirmed event reaches 60 C.
             */
            if (((g_lithium.recoverable_fault_flags |
                  g_lithium.latched_fault_flags) &
                 fault) == 0U)
            {
                g_lithium.warning_fault_flags |=
                        fault;
            }


            break;


        case FAULT_SEVERITY_RECOVERABLE:

            /*
             * A latched fault is permanent until reset /
             * controlled power cycle.  Do not re-add the same
             * condition to the recoverable set on later samples.
             */
            if ((g_lithium.latched_fault_flags &
                 fault) == 0U)
            {
                g_lithium.warning_fault_flags &=
                        ~fault;


                g_lithium.recoverable_fault_flags |=
                        fault;
            }


            break;


        case FAULT_SEVERITY_LATCHED:

            g_lithium.warning_fault_flags &=
                    ~fault;


            g_lithium.recoverable_fault_flags &=
                    ~fault;


            g_lithium.latched_fault_flags |=
                    fault;


            if (g_lithium.system_state !=
                SYS_FAULT_LATCHED)
            {
                g_lithium.system_state =
                        SYS_FAULT_LATCHED;


                g_lithium.state_enter_time_ms =
                        now;
            }


            break;


        case FAULT_SEVERITY_NONE:

        default:

            break;
    }


    /*
     * Select the primary fault from the highest-severity active
     * mask first.  This guarantees that primary_fault and
     * primary_fault_severity always describe the same condition.
     */
    if (g_lithium.latched_fault_flags != 0U)
    {
        g_lithium.primary_fault =
                Lithium_SelectPrimaryFault(
                        g_lithium.latched_fault_flags);


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_LATCHED;
    }
    else if (g_lithium.recoverable_fault_flags != 0U)
    {
        g_lithium.primary_fault =
                Lithium_SelectPrimaryFault(
                        g_lithium.recoverable_fault_flags);


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_RECOVERABLE;
    }
    else if (g_lithium.warning_fault_flags != 0U)
    {
        g_lithium.primary_fault =
                Lithium_SelectPrimaryFault(
                        g_lithium.warning_fault_flags);


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_WARNING;
    }
    else
    {
        g_lithium.primary_fault =
                LITHIUM_FAULT_NONE;


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_NONE;
    }
}


void Lithium_ClearFault(
        uint32_t fault)
{
    uint32_t clearable_faults;


    /*
     * Never silently clear a latched fault.
     */
    clearable_faults =
            fault &
            ~g_lithium.latched_fault_flags;


    g_lithium.fault_flags &=
            ~clearable_faults;


    g_lithium.warning_fault_flags &=
            ~clearable_faults;


    g_lithium.recoverable_fault_flags &=
            ~clearable_faults;


    /*
     * Select the primary fault from the highest-severity active
     * mask first.  This guarantees that primary_fault and
     * primary_fault_severity always describe the same condition.
     */
    if (g_lithium.latched_fault_flags != 0U)
    {
        g_lithium.primary_fault =
                Lithium_SelectPrimaryFault(
                        g_lithium.latched_fault_flags);


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_LATCHED;
    }
    else if (g_lithium.recoverable_fault_flags != 0U)
    {
        g_lithium.primary_fault =
                Lithium_SelectPrimaryFault(
                        g_lithium.recoverable_fault_flags);


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_RECOVERABLE;
    }
    else if (g_lithium.warning_fault_flags != 0U)
    {
        g_lithium.primary_fault =
                Lithium_SelectPrimaryFault(
                        g_lithium.warning_fault_flags);


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_WARNING;
    }
    else
    {
        g_lithium.primary_fault =
                LITHIUM_FAULT_NONE;


        g_lithium.primary_fault_severity =
                FAULT_SEVERITY_NONE;
    }
}


/* ============================================================
 * PRIMARY FAULT
 * ============================================================ */

static uint32_t Lithium_SelectPrimaryFault(
        uint32_t faults)
{
    if ((faults &
         LITHIUM_FAULT_BATTERY_OVERCURRENT) != 0U)
    {
        return
                LITHIUM_FAULT_BATTERY_OVERCURRENT;
    }


    if ((faults &
         LITHIUM_FAULT_CELL_OVERVOLTAGE) != 0U)
    {
        return
                LITHIUM_FAULT_CELL_OVERVOLTAGE;
    }


    if ((faults &
         LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE) != 0U)
    {
        return
                LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE;
    }


    if ((faults &
         LITHIUM_FAULT_TEMP_HIGH) != 0U)
    {
        return
                LITHIUM_FAULT_TEMP_HIGH;
    }


    if ((faults &
         LITHIUM_FAULT_BQ25750_COMM) != 0U)
    {
        return
                LITHIUM_FAULT_BQ25750_COMM;
    }


    if ((faults &
         LITHIUM_FAULT_BQ25750_INIT) != 0U)
    {
        return
                LITHIUM_FAULT_BQ25750_INIT;
    }


    if ((faults &
         LITHIUM_FAULT_BQ76942_COMM) != 0U)
    {
        return
                LITHIUM_FAULT_BQ76942_COMM;
    }


    if ((faults &
         LITHIUM_FAULT_BQ76942_INIT) != 0U)
    {
        return
                LITHIUM_FAULT_BQ76942_INIT;
    }


    if ((faults &
         LITHIUM_FAULT_CHARGE_TIMEOUT) != 0U)
    {
        return
                LITHIUM_FAULT_CHARGE_TIMEOUT;
    }


    if ((faults &
         LITHIUM_FAULT_TEMP_SENSOR) != 0U)
    {
        return
                LITHIUM_FAULT_TEMP_SENSOR;
    }


    if ((faults &
         LITHIUM_FAULT_INPUT_OVERVOLTAGE) != 0U)
    {
        return
                LITHIUM_FAULT_INPUT_OVERVOLTAGE;
    }


    return
            faults &
            (0U - faults);
}


/* ============================================================
 * STATUS INDICATION
 * ============================================================ */

static void Lithium_UpdateStatusIndication(void)
{
    if (g_lithium.system_state ==
        SYS_FAULT_LATCHED)
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_FAULT_LATCHED);


        return;
    }


    if (g_lithium.system_state ==
        SYS_FAULT_RECOVERABLE)
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_FAULT_RECOVERABLE);


        return;
    }


    if ((!g_lithium.bq25750_initialized) &&
        (!g_lithium.bq76942_initialized))
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_BOTH_BQ_ERROR);


        return;
    }


    if (!g_lithium.bq25750_initialized)
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_BQ25750_ERROR);


        return;
    }


    if (!g_lithium.bq76942_initialized)
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_BQ76942_ERROR);


        return;
    }


    /*
     * Generic WARNING indication.
     *
     * Exact warning identity is intentionally not encoded in
     * the LED language.  CAN diagnostics will report the exact
     * active warning(s).  The local LED only communicates that
     * the system is still operating but requires attention.
     *
     * WARNING overrides the normal READY / CHARGING indication
     * so a degraded but still-operational condition cannot remain
     * invisible to the operator.
     */
    if (g_lithium.warning_fault_flags !=
        LITHIUM_FAULT_NONE)
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_WARNING);


        return;
    }


    if ((g_lithium.charger_state ==
         CHARGE_ACTIVE) ||
        (g_lithium.charger_state ==
         CHARGE_LOW_CELL_RECOVERY))
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_CHARGING);


        return;
    }


    if (g_lithium.system_state ==
        SYS_READY)
    {
        Lithium_LED_SetPattern(
                LED_PATTERN_READY);
    }
}


/* ============================================================
 * CELL HELPERS
 * ============================================================ */

static uint16_t Lithium_GetMinCellMv(void)
{
    uint16_t value;


    value =
            g_lithium.measurements.cell_mv[0];


    if (g_lithium.measurements.cell_mv[1] <
        value)
    {
        value =
                g_lithium.measurements.cell_mv[1];
    }


    if (g_lithium.measurements.cell_mv[2] <
        value)
    {
        value =
                g_lithium.measurements.cell_mv[2];
    }


    return value;
}


static uint16_t Lithium_GetMaxCellMv(void)
{
    uint16_t value;


    value =
            g_lithium.measurements.cell_mv[0];


    if (g_lithium.measurements.cell_mv[1] >
        value)
    {
        value =
                g_lithium.measurements.cell_mv[1];
    }


    if (g_lithium.measurements.cell_mv[2] >
        value)
    {
        value =
                g_lithium.measurements.cell_mv[2];
    }


    return value;
}


/* ============================================================
 * TRUSTED TEMPERATURE HELPERS
 * ============================================================ */

static int16_t Lithium_GetLowestTrustedTemperaturedC(void)
{
    int16_t value =
            1000;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if (g_lithium.measurements.temperature_status[i] ==
            TEMP_SENSOR_STATUS_VALID)
        {
            if (g_lithium.measurements.temperature_dC[i] <
                value)
            {
                value =
                        g_lithium.measurements.temperature_dC[i];
            }
        }
    }


    if (g_lithium.measurements.pack_center_temperature_status ==
        TEMP_SENSOR_STATUS_VALID)
    {
        if (g_lithium.measurements.pack_center_temperature_dC <
            value)
        {
            value =
                    g_lithium.measurements.pack_center_temperature_dC;
        }
    }


    return value;
}


static int16_t Lithium_GetHighestTrustedTemperaturedC(void)
{
    int16_t value =
            -1000;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if (g_lithium.measurements.temperature_status[i] ==
            TEMP_SENSOR_STATUS_VALID)
        {
            if (g_lithium.measurements.temperature_dC[i] >
                value)
            {
                value =
                        g_lithium.measurements.temperature_dC[i];
            }
        }
    }


    if (g_lithium.measurements.pack_center_temperature_status ==
        TEMP_SENSOR_STATUS_VALID)
    {
        if (g_lithium.measurements.pack_center_temperature_dC >
            value)
        {
            value =
                    g_lithium.measurements.pack_center_temperature_dC;
        }
    }


    return value;
}


/* ============================================================
 * TEMPERATURE COUNT HELPERS
 * ============================================================ */

static uint8_t Lithium_CountHotTemperatureSources(void)
{
    return
            Lithium_CountTemperatureSourcesAbove(
                    LITHIUM_CHARGE_TEMP_MAX_dC);
}


static uint8_t Lithium_CountColdTemperatureSources(void)
{
    uint8_t count =
            0U;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if ((g_lithium.measurements.temperature_status[i] !=
             TEMP_SENSOR_STATUS_FAULT) &&
            (g_lithium.measurements.temperature_dC[i] <
             LITHIUM_CHARGE_TEMP_MIN_dC))
        {
            count++;
        }
    }


    if ((g_lithium.measurements.pack_center_temperature_status !=
         TEMP_SENSOR_STATUS_FAULT) &&
        (g_lithium.measurements.pack_center_temperature_dC <
         LITHIUM_CHARGE_TEMP_MIN_dC))
    {
        count++;
    }


    return count;
}


static uint8_t Lithium_CountTemperatureSourcesBelow(
        int16_t threshold_dC)
{
    uint8_t count =
            0U;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if ((g_lithium.measurements.temperature_status[i] !=
             TEMP_SENSOR_STATUS_FAULT) &&
            (g_lithium.measurements.temperature_dC[i] <=
             threshold_dC))
        {
            count++;
        }
    }


    if ((g_lithium.measurements.pack_center_temperature_status !=
         TEMP_SENSOR_STATUS_FAULT) &&
        (g_lithium.measurements.pack_center_temperature_dC <=
         threshold_dC))
    {
        count++;
    }


    return count;
}


static uint8_t Lithium_CountTemperatureSourcesAbove(
        int16_t threshold_dC)
{
    uint8_t count =
            0U;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if ((g_lithium.measurements.temperature_status[i] !=
             TEMP_SENSOR_STATUS_FAULT) &&
            (g_lithium.measurements.temperature_dC[i] >=
             threshold_dC))
        {
            count++;
        }
    }


    if ((g_lithium.measurements.pack_center_temperature_status !=
         TEMP_SENSOR_STATUS_FAULT) &&
        (g_lithium.measurements.pack_center_temperature_dC >=
         threshold_dC))
    {
        count++;
    }


    return count;
}


/* ============================================================
 * TEMPERATURE UTILITY
 * ============================================================ */

static int16_t Lithium_AbsTemperatureDifference(
        int16_t a,
        int16_t b)
{
    int16_t difference;


    difference =
            (int16_t)(
                    a - b);


    if (difference < 0)
    {
        difference =
                (int16_t)(-difference);
    }


    return difference;
}


static bool Lithium_TemperaturePlausible(
        int16_t temperature_dC)
{
    return
            ((temperature_dC >=
              LITHIUM_TEMP_SENSOR_PLAUSIBLE_MIN_dC) &&
             (temperature_dC <=
              LITHIUM_TEMP_SENSOR_PLAUSIBLE_MAX_dC));
}


/* ============================================================
 * HAL EXTI CALLBACKS
 *
 * ISR rule:
 *
 * FLAGS ONLY.
 * ============================================================ */

void HAL_GPIO_EXTI_Falling_Callback(
        uint16_t GPIO_Pin)
{
    if (GPIO_Pin ==
        BQ_INT_Pin)
    {
        g_lithium_events.bq25750_interrupt =
                true;
    }


    if (GPIO_Pin ==
        BQ2_ALERT_Pin)
    {
        g_lithium_events.bq76942_alert =
                true;
    }


    if (GPIO_Pin ==
        BQ_PG_Pin)
    {
        g_lithium_events.power_good_changed =
                true;
    }
}


void HAL_GPIO_EXTI_Rising_Callback(
        uint16_t GPIO_Pin)
{
    if (GPIO_Pin ==
        BQ_PG_Pin)
    {
        g_lithium_events.power_good_changed =
                true;
    }
}


/* ============================================================
 * I2C CALLBACKS
 * ============================================================ */

void HAL_I2C_MemRxCpltCallback(
        I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return;
    }


    if (hi2c->Instance ==
        I2C1)
    {
        BQ25750_NotifyI2CTransferComplete();
    }
}


void HAL_I2C_ErrorCallback(
        I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return;
    }


    if (hi2c->Instance ==
        I2C1)
    {
        BQ25750_NotifyI2CTransferError();
    }
}


/* ============================================================
 * SPI CALLBACKS
 * ============================================================ */

void HAL_SPI_TxRxCpltCallback(
        SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL)
    {
        return;
    }


    if (hspi->Instance ==
        SPI1)
    {
        BQ76942_NotifySpiTransferComplete();
    }
}


void HAL_SPI_ErrorCallback(
        SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL)
    {
        return;
    }


    if (hspi->Instance ==
        SPI1)
    {
        BQ76942_NotifySpiTransferError();
    }
}
