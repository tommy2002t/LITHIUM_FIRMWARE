#include "lithium_can.h"

#include "main.h"
#include "lithium_app.h"
#include "lithium_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>


extern FDCAN_HandleTypeDef hfdcan1;


#if (LITHIUM_CAN_NODE_ID > 2U)
#error "LITHIUM_CAN_NODE_ID must be 0, 1 or 2 for the current CAN allocation."
#endif

#if ((LITHIUM_CAN_BASE_ID + \
      (LITHIUM_CAN_NODE_ID * LITHIUM_CAN_NODE_ID_STRIDE) + \
      LITHIUM_CAN_OFFSET_DIAGNOSTIC_TEXT) > 0x7FFU)
#error "LITHIUM CAN identifier allocation exceeds the 11-bit standard-ID range."
#endif


/* ============================================================
 * INTERNAL CAN STATE
 * ============================================================ */

static bool can_started = false;

static uint8_t can_tx_error_count = 0U;

static uint32_t last_can_restart_ms = 0U;

static uint32_t last_status_tx_ms = 0U;
static uint32_t last_cells_tx_ms = 0U;
static uint32_t last_temperature_tx_ms = 0U;
static uint32_t last_power_tx_ms = 0U;
static uint32_t last_current_tx_ms = 0U;
static uint32_t last_fault_summary_tx_ms = 0U;
static uint32_t last_fault_masks_a_tx_ms = 0U;
static uint32_t last_fault_masks_b_tx_ms = 0U;


/* ============================================================
 * DIAGNOSTIC CHANGE TRACKING
 * ============================================================ */

static uint32_t last_fault_flags = 0U;
static uint32_t last_warning_flags = 0U;
static uint32_t last_recoverable_flags = 0U;
static uint32_t last_latched_flags = 0U;
static uint16_t last_charger_diagnostic_flags = 0U;

static uint32_t pending_active_faults = 0U;
static uint32_t pending_cleared_faults = 0U;

static uint8_t diagnostic_sequence = 0U;


typedef enum
{
    CAN_DIAG_IDLE = 0,
    CAN_DIAG_SEND_HEADER,
    CAN_DIAG_SEND_TEXT

} can_diag_tx_state_t;


static can_diag_tx_state_t diagnostic_tx_state =
        CAN_DIAG_IDLE;

static uint32_t diagnostic_fault = 0U;

static lithium_can_diag_event_state_t diagnostic_event_state =
        LITHIUM_CAN_DIAG_EVENT_CLEARED;

static fault_severity_t diagnostic_severity =
        FAULT_SEVERITY_NONE;

static const char *diagnostic_text = NULL;

static uint8_t diagnostic_text_length = 0U;
static uint8_t diagnostic_segment_count = 0U;
static uint8_t diagnostic_segment_index = 0U;


/* ============================================================
 * INTERNAL HELPERS
 * ============================================================ */

static uint32_t Lithium_CAN_GetIdentifier(
        uint32_t offset);

static bool Lithium_CAN_StartBus(void);

static bool Lithium_CAN_SendFrame(
        uint32_t identifier,
        const uint8_t data[8]);

static void Lithium_CAN_WriteU16LE(
        uint8_t *destination,
        uint16_t value);

static void Lithium_CAN_WriteS16LE(
        uint8_t *destination,
        int16_t value);

static void Lithium_CAN_WriteU32LE(
        uint8_t *destination,
        uint32_t value);

static int16_t Lithium_CAN_SaturateS16(
        int32_t value);

static uint8_t Lithium_CAN_GetFaultBitIndex(
        uint32_t fault);

static uint32_t Lithium_CAN_GetLowestSetBit(
        uint32_t value);

static fault_severity_t Lithium_CAN_GetFaultSeverity(
        uint32_t fault);

static uint8_t Lithium_CAN_PackTemperatureStates(void);

static uint8_t Lithium_CAN_GetBQStatusFlags(void);

static bool Lithium_CAN_SendStatusFrame(void);
static bool Lithium_CAN_SendCellVoltageFrame(void);
static bool Lithium_CAN_SendTemperatureFrame(void);
static bool Lithium_CAN_SendPowerVoltageFrame(void);
static bool Lithium_CAN_SendCurrentSocFrame(void);
static bool Lithium_CAN_SendFaultSummaryFrame(void);
static bool Lithium_CAN_SendFaultMasksAFrame(void);
static bool Lithium_CAN_SendFaultMasksBFrame(void);

static bool Lithium_CAN_SendOnePeriodicFrame(
        uint32_t now);

static void Lithium_CAN_UpdateDiagnosticChanges(void);

static bool Lithium_CAN_ProcessDiagnosticTx(void);

static void Lithium_CAN_BeginNextDiagnostic(void);

static const char *Lithium_CAN_GetFaultText(
        uint32_t fault,
        fault_severity_t severity);

static const char *Lithium_CAN_GetLocalThermistorWarningText(void);

static const char *Lithium_CAN_GetPackCenterThermistorText(void);

static const char *Lithium_CAN_GetChargerFaultText(void);

static const char *Lithium_CAN_GetChargeTimeoutText(void);


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void Lithium_CAN_Init(void)
{
    can_started =
            false;


    can_tx_error_count =
            0U;


    last_can_restart_ms =
            HAL_GetTick();


    last_status_tx_ms =
            0U;


    last_cells_tx_ms =
            0U;


    last_temperature_tx_ms =
            0U;


    last_power_tx_ms =
            0U;


    last_current_tx_ms =
            0U;


    last_fault_summary_tx_ms =
            0U;


    last_fault_masks_a_tx_ms =
            0U;


    last_fault_masks_b_tx_ms =
            0U;


    last_fault_flags =
            0U;


    last_warning_flags =
            0U;


    last_recoverable_flags =
            0U;


    last_latched_flags =
            0U;


    last_charger_diagnostic_flags =
            0U;


    pending_active_faults =
            0U;


    pending_cleared_faults =
            0U;


    diagnostic_sequence =
            0U;


    diagnostic_tx_state =
            CAN_DIAG_IDLE;


    diagnostic_fault =
            0U;


    diagnostic_text =
            NULL;


    /*
     * Keep the TCAN3414 electrically quiet until the FDCAN
     * peripheral is configured and successfully started.
     */
    HAL_GPIO_WritePin(
            CAN_STB_GPIO_Port,
            CAN_STB_Pin,
            GPIO_PIN_SET);


#if LITHIUM_CAN_ENABLE

    (void)Lithium_CAN_StartBus();

#endif
}


/* ============================================================
 * MAIN NON-BLOCKING CAN SERVICE
 * ============================================================ */

void Lithium_CAN_Process(void)
{
#if LITHIUM_CAN_ENABLE

    uint32_t now;


    now =
            HAL_GetTick();


    if (!can_started)
    {
        if ((now -
             last_can_restart_ms) >=
            LITHIUM_CAN_RESTART_PERIOD_MS)
        {
            last_can_restart_ms =
                    now;


            (void)Lithium_CAN_StartBus();
        }


        if (!can_started)
        {
            /*
             * CAN is telemetry-only.  Losing the bus must be
             * visible locally, but must never alter charging,
             * balancing or either power path.
             */
            Lithium_SetFault(
                    LITHIUM_FAULT_CAN,
                    FAULT_SEVERITY_WARNING);


            /*
             * Track the state change even while the bus is down.
             * This prevents the diagnostic bookkeeping from
             * becoming stale across a later successful restart.
             */
            Lithium_CAN_UpdateDiagnosticChanges();


            return;
        }
    }


    if ((g_lithium.fault_flags &
         LITHIUM_FAULT_CAN) != 0U)
    {
        Lithium_ClearFault(
                LITHIUM_FAULT_CAN);
    }


    Lithium_CAN_UpdateDiagnosticChanges();


    /*
     * Diagnostic events have priority so a newly-detected fault
     * is described immediately. Only one CAN frame is attempted
     * per call from this path, so the main loop remains fully
     * non-blocking.
     */
    if (Lithium_CAN_ProcessDiagnosticTx())
    {
        return;
    }


    (void)Lithium_CAN_SendOnePeriodicFrame(
            now);

#else

    /*
     * CAN disabled at compile time.
     *
     * No CAN condition is ever allowed to alter charging,
     * balancing or the battery power path.
     */

#endif
}


/* ============================================================
 * IDENTIFIER ALLOCATION
 * ============================================================ */

static uint32_t Lithium_CAN_GetIdentifier(
        uint32_t offset)
{
    return
            LITHIUM_CAN_BASE_ID +
            (LITHIUM_CAN_NODE_ID *
             LITHIUM_CAN_NODE_ID_STRIDE) +
            offset;
}


/* ============================================================
 * BUS START / RESTART
 * ============================================================ */

static bool Lithium_CAN_StartBus(void)
{
#if LITHIUM_CAN_ENABLE

    /*
     * The current LITHIUM protocol is transmit-only.
     *
     * Reject all received frames and remote frames until an
     * explicitly reviewed command protocol is added. This keeps
     * CAN from acquiring hidden control authority.
     */
    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        HAL_GPIO_WritePin(
                CAN_STB_GPIO_Port,
                CAN_STB_Pin,
                GPIO_PIN_SET);


        can_started =
                false;


        return false;
    }


    if (HAL_FDCAN_Start(
            &hfdcan1) != HAL_OK)
    {
        HAL_GPIO_WritePin(
                CAN_STB_GPIO_Port,
                CAN_STB_Pin,
                GPIO_PIN_SET);


        can_started =
                false;


        return false;
    }


    HAL_GPIO_WritePin(
            CAN_STB_GPIO_Port,
            CAN_STB_Pin,
            GPIO_PIN_RESET);


    can_started =
            true;


    can_tx_error_count =
            0U;


    return true;

#else

    return false;

#endif
}


/* ============================================================
 * LOW-LEVEL TRANSMIT
 * ============================================================ */

static bool Lithium_CAN_SendFrame(
        uint32_t identifier,
        const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx_header;


    if ((!can_started) ||
        (data == NULL))
    {
        return false;
    }


    if (HAL_FDCAN_GetTxFifoFreeLevel(
            &hfdcan1) == 0U)
    {
        return false;
    }


    memset(
            &tx_header,
            0,
            sizeof(tx_header));


    tx_header.Identifier =
            identifier;


    tx_header.IdType =
            FDCAN_STANDARD_ID;


    tx_header.TxFrameType =
            FDCAN_DATA_FRAME;


    tx_header.DataLength =
            FDCAN_DLC_BYTES_8;


    tx_header.ErrorStateIndicator =
            FDCAN_ESI_ACTIVE;


    tx_header.BitRateSwitch =
            FDCAN_BRS_OFF;


    tx_header.FDFormat =
            FDCAN_CLASSIC_CAN;


    tx_header.TxEventFifoControl =
            FDCAN_NO_TX_EVENTS;


    tx_header.MessageMarker =
            0U;


    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &tx_header,
            (uint8_t *)data) == HAL_OK)
    {
        can_tx_error_count =
                0U;


        return true;
    }


    if (can_tx_error_count <
        255U)
    {
        can_tx_error_count++;
    }


    /*
     * CAN is telemetry-only. If the peripheral repeatedly fails,
     * isolate the transceiver and retry locally later. Never map
     * this to a battery-power or charging fault.
     */
    if (can_tx_error_count >=
        8U)
    {
        (void)HAL_FDCAN_Stop(
                &hfdcan1);


        HAL_GPIO_WritePin(
                CAN_STB_GPIO_Port,
                CAN_STB_Pin,
                GPIO_PIN_SET);


        can_started =
                false;


        last_can_restart_ms =
                HAL_GetTick();
    }


    return false;
}


/* ============================================================
 * BYTE PACKING
 * ============================================================ */

static void Lithium_CAN_WriteU16LE(
        uint8_t *destination,
        uint16_t value)
{
    destination[0] =
            (uint8_t)(value & 0xFFU);


    destination[1] =
            (uint8_t)((value >> 8) & 0xFFU);
}


static void Lithium_CAN_WriteS16LE(
        uint8_t *destination,
        int16_t value)
{
    Lithium_CAN_WriteU16LE(
            destination,
            (uint16_t)value);
}


static void Lithium_CAN_WriteU32LE(
        uint8_t *destination,
        uint32_t value)
{
    destination[0] =
            (uint8_t)(value & 0xFFU);


    destination[1] =
            (uint8_t)((value >> 8) & 0xFFU);


    destination[2] =
            (uint8_t)((value >> 16) & 0xFFU);


    destination[3] =
            (uint8_t)((value >> 24) & 0xFFU);
}


static int16_t Lithium_CAN_SaturateS16(
        int32_t value)
{
    if (value >
        32767L)
    {
        return
                32767;
    }


    if (value <
        -32768L)
    {
        return
                -32768;
    }


    return
            (int16_t)value;
}


/* ============================================================
 * FAULT / STATUS HELPERS
 * ============================================================ */

static uint8_t Lithium_CAN_GetFaultBitIndex(
        uint32_t fault)
{
    uint8_t index;


    if (fault == 0U)
    {
        return
                0xFFU;
    }


    for (index = 0U;
         index < 32U;
         index++)
    {
        if ((fault &
             (1UL << index)) != 0U)
        {
            return
                    index;
        }
    }


    return
            0xFFU;
}


static uint32_t Lithium_CAN_GetLowestSetBit(
        uint32_t value)
{
    return
            value &
            (0U - value);
}


static fault_severity_t Lithium_CAN_GetFaultSeverity(
        uint32_t fault)
{
    if ((g_lithium.latched_fault_flags &
         fault) != 0U)
    {
        return
                FAULT_SEVERITY_LATCHED;
    }


    if ((g_lithium.recoverable_fault_flags &
         fault) != 0U)
    {
        return
                FAULT_SEVERITY_RECOVERABLE;
    }


    if ((g_lithium.warning_fault_flags &
         fault) != 0U)
    {
        return
                FAULT_SEVERITY_WARNING;
    }


    return
            FAULT_SEVERITY_NONE;
}


static uint8_t Lithium_CAN_PackTemperatureStates(void)
{
    uint8_t packed;


    packed =
            0U;


    packed |=
            ((uint8_t)g_lithium.measurements.temperature_status[0] &
             0x03U) << 0;


    packed |=
            ((uint8_t)g_lithium.measurements.temperature_status[1] &
             0x03U) << 2;


    packed |=
            ((uint8_t)g_lithium.measurements.temperature_status[2] &
             0x03U) << 4;


    packed |=
            ((uint8_t)g_lithium.measurements.pack_center_temperature_status &
             0x03U) << 6;


    return packed;
}


static uint8_t Lithium_CAN_GetBQStatusFlags(void)
{
    uint8_t flags;


    flags =
            0U;


    if (g_lithium.measurements.bq76942_cuv_active)
    {
        flags |=
                LITHIUM_CAN_BQ_STATUS_CUV_ACTIVE;
    }


    if (g_lithium.measurements.bq76942_cov_active)
    {
        flags |=
                LITHIUM_CAN_BQ_STATUS_COV_ACTIVE;
    }


    if (g_lithium.measurements.bq25750_valid)
    {
        flags |=
                LITHIUM_CAN_BQ_STATUS_BQ25750_VALID;
    }


    if (g_lithium.measurements.bq76942_valid)
    {
        flags |=
                LITHIUM_CAN_BQ_STATUS_BQ76942_VALID;
    }


    if (g_lithium.measurements.charger_watchdog_expired)
    {
        flags |=
                LITHIUM_CAN_BQ_STATUS_WATCHDOG_EXPIRED;
    }


    return flags;
}


/* ============================================================
 * PERIODIC TELEMETRY FRAMES
 * ============================================================ */

static bool Lithium_CAN_SendStatusFrame(void)
{
    uint8_t data[8] = {0U};
    uint8_t status_flags = 0U;


    if (g_lithium.input_present)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_INPUT_PRESENT;
    }


    if (g_lithium.battery_isolated)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_BATTERY_ISOLATED;
    }


    if (g_lithium.input_isolated)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_INPUT_ISOLATED;
    }


    if (g_lithium.bq25750_initialized)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_BQ25750_INITIALIZED;
    }


    if (g_lithium.bq76942_initialized)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_BQ76942_INITIALIZED;
    }


    if (g_lithium.measurements.valid)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_MEASUREMENTS_VALID;
    }


    if (g_lithium.measurements.charger_power_good)
    {
        status_flags |=
                LITHIUM_CAN_STATUS_CHARGER_POWER_GOOD;
    }


    data[0] =
            LITHIUM_CAN_PROTOCOL_VERSION;


    data[1] =
            (uint8_t)g_lithium.system_state;


    data[2] =
            (uint8_t)g_lithium.charger_state;


    data[3] =
            (uint8_t)g_lithium.balancing_state;


    data[4] =
            (uint8_t)g_lithium.power_source;


    data[5] =
            (uint8_t)g_lithium.operating_mode;


    data[6] =
            (uint8_t)g_lithium.primary_fault_severity;


    data[7] =
            status_flags;


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_STATUS),
                    data);
}


static bool Lithium_CAN_SendCellVoltageFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteU16LE(
            &data[0],
            g_lithium.measurements.cell_mv[0]);


    Lithium_CAN_WriteU16LE(
            &data[2],
            g_lithium.measurements.cell_mv[1]);


    Lithium_CAN_WriteU16LE(
            &data[4],
            g_lithium.measurements.cell_mv[2]);


    Lithium_CAN_WriteU16LE(
            &data[6],
            g_lithium.measurements.pack_mv);


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_CELL_VOLTAGES),
                    data);
}


static bool Lithium_CAN_SendTemperatureFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteS16LE(
            &data[0],
            g_lithium.measurements.temperature_dC[0]);


    Lithium_CAN_WriteS16LE(
            &data[2],
            g_lithium.measurements.temperature_dC[1]);


    Lithium_CAN_WriteS16LE(
            &data[4],
            g_lithium.measurements.temperature_dC[2]);


    Lithium_CAN_WriteS16LE(
            &data[6],
            g_lithium.measurements.pack_center_temperature_dC);


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_TEMPERATURES),
                    data);
}


static bool Lithium_CAN_SendPowerVoltageFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteU16LE(
            &data[0],
            g_lithium.measurements.vin_mv);


    Lithium_CAN_WriteU16LE(
            &data[2],
            g_lithium.measurements.vsys_mv);


    Lithium_CAN_WriteU16LE(
            &data[4],
            g_lithium.measurements.charger_vbat_mv);


    data[6] =
            0U;


    data[7] =
            0U;


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_POWER_VOLTAGES),
                    data);
}


static bool Lithium_CAN_SendCurrentSocFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteS16LE(
            &data[0],
            Lithium_CAN_SaturateS16(
                    g_lithium.measurements.battery_current_ma));


    Lithium_CAN_WriteS16LE(
            &data[2],
            Lithium_CAN_SaturateS16(
                    g_lithium.measurements.input_current_ma));


    /*
     * SOC estimation is not implemented yet.  Transmit the
     * explicit invalid value instead of the boot placeholder
     * (100.00%), so the receiver cannot mistake it for a real
     * state-of-charge estimate.
     */
    Lithium_CAN_WriteU16LE(
            &data[4],
            0xFFFFU);


    Lithium_CAN_WriteU16LE(
            &data[6],
            g_lithium.active_balance_mask);


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_CURRENT_SOC),
                    data);
}


static bool Lithium_CAN_SendFaultSummaryFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteU32LE(
            &data[0],
            g_lithium.fault_flags);


    data[4] =
            Lithium_CAN_GetFaultBitIndex(
                    g_lithium.primary_fault);


    data[5] =
            (uint8_t)g_lithium.primary_fault_severity;


    Lithium_CAN_WriteU16LE(
            &data[6],
            g_lithium.charger_diagnostic_flags);


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_FAULT_SUMMARY),
                    data);
}


static bool Lithium_CAN_SendFaultMasksAFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteU32LE(
            &data[0],
            g_lithium.warning_fault_flags);


    Lithium_CAN_WriteU32LE(
            &data[4],
            g_lithium.recoverable_fault_flags);


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_FAULT_MASKS_A),
                    data);
}


static bool Lithium_CAN_SendFaultMasksBFrame(void)
{
    uint8_t data[8] = {0U};


    Lithium_CAN_WriteU32LE(
            &data[0],
            g_lithium.latched_fault_flags);


    data[4] =
            Lithium_CAN_PackTemperatureStates();


    data[5] =
            Lithium_CAN_GetBQStatusFlags();


    data[6] =
            g_lithium.measurements.charger_fault_status;


    data[7] =
            g_lithium.measurements.charger_charge_state;


    return
            Lithium_CAN_SendFrame(
                    Lithium_CAN_GetIdentifier(
                            LITHIUM_CAN_OFFSET_FAULT_MASKS_B),
                    data);
}


/* ============================================================
 * PERIODIC SCHEDULER
 *
 * At most one periodic frame is attempted per main-loop call.
 * ============================================================ */

static bool Lithium_CAN_SendOnePeriodicFrame(
        uint32_t now)
{
    if ((now -
         last_status_tx_ms) >=
        LITHIUM_CAN_STATUS_PERIOD_MS)
    {
        if (Lithium_CAN_SendStatusFrame())
        {
            last_status_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_fault_summary_tx_ms) >=
        LITHIUM_CAN_FAULT_PERIOD_MS)
    {
        if (Lithium_CAN_SendFaultSummaryFrame())
        {
            last_fault_summary_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_fault_masks_a_tx_ms) >=
        LITHIUM_CAN_FAULT_PERIOD_MS)
    {
        if (Lithium_CAN_SendFaultMasksAFrame())
        {
            last_fault_masks_a_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_fault_masks_b_tx_ms) >=
        LITHIUM_CAN_FAULT_PERIOD_MS)
    {
        if (Lithium_CAN_SendFaultMasksBFrame())
        {
            last_fault_masks_b_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_cells_tx_ms) >=
        LITHIUM_CAN_ELECTRICAL_PERIOD_MS)
    {
        if (Lithium_CAN_SendCellVoltageFrame())
        {
            last_cells_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_power_tx_ms) >=
        LITHIUM_CAN_ELECTRICAL_PERIOD_MS)
    {
        if (Lithium_CAN_SendPowerVoltageFrame())
        {
            last_power_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_current_tx_ms) >=
        LITHIUM_CAN_ELECTRICAL_PERIOD_MS)
    {
        if (Lithium_CAN_SendCurrentSocFrame())
        {
            last_current_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    if ((now -
         last_temperature_tx_ms) >=
        LITHIUM_CAN_THERMAL_PERIOD_MS)
    {
        if (Lithium_CAN_SendTemperatureFrame())
        {
            last_temperature_tx_ms =
                    now;


            return true;
        }


        return false;
    }


    return false;
}


/* ============================================================
 * DIAGNOSTIC CHANGE DETECTION
 * ============================================================ */

static void Lithium_CAN_UpdateDiagnosticChanges(void)
{
    uint32_t changed_faults;


    changed_faults =
            (g_lithium.fault_flags ^
             last_fault_flags) |
            (g_lithium.warning_fault_flags ^
             last_warning_flags) |
            (g_lithium.recoverable_fault_flags ^
             last_recoverable_flags) |
            (g_lithium.latched_fault_flags ^
             last_latched_flags);


    /*
     * If a charger diagnostic changes while the CHARGER or
     * CHARGE_TIMEOUT fault remains active, re-emit that fault so
     * the human-readable explanation follows the exact cause.
     */
    if (g_lithium.charger_diagnostic_flags !=
        last_charger_diagnostic_flags)
    {
        changed_faults |=
                g_lithium.fault_flags &
                (LITHIUM_FAULT_CHARGER |
                 LITHIUM_FAULT_CHARGE_TIMEOUT);
    }


    pending_active_faults |=
            changed_faults &
            g_lithium.fault_flags;


    pending_cleared_faults |=
            changed_faults &
            ~g_lithium.fault_flags;


    /*
     * A condition can clear and reappear while older diagnostic
     * frames are still waiting. Never emit a stale CLEAR after a
     * new activation, or a stale ACTIVE after the fault cleared.
     */
    pending_active_faults &=
            g_lithium.fault_flags;


    pending_cleared_faults &=
            ~g_lithium.fault_flags;


    last_fault_flags =
            g_lithium.fault_flags;


    last_warning_flags =
            g_lithium.warning_fault_flags;


    last_recoverable_flags =
            g_lithium.recoverable_fault_flags;


    last_latched_flags =
            g_lithium.latched_fault_flags;


    last_charger_diagnostic_flags =
            g_lithium.charger_diagnostic_flags;
}


/* ============================================================
 * DIAGNOSTIC TRANSMISSION STATE MACHINE
 * ============================================================ */

static bool Lithium_CAN_ProcessDiagnosticTx(void)
{
    uint8_t data[8] = {0U};


    if (diagnostic_tx_state ==
        CAN_DIAG_IDLE)
    {
        Lithium_CAN_BeginNextDiagnostic();


        if (diagnostic_tx_state ==
            CAN_DIAG_IDLE)
        {
            return false;
        }
    }


    if (diagnostic_tx_state ==
        CAN_DIAG_SEND_HEADER)
    {
        data[0] =
                diagnostic_sequence;


        data[1] =
                (uint8_t)diagnostic_event_state;


        data[2] =
                Lithium_CAN_GetFaultBitIndex(
                        diagnostic_fault);


        data[3] =
                (uint8_t)diagnostic_severity;


        data[4] =
                (uint8_t)(g_lithium.charger_diagnostic_flags &
                          0xFFU);


        data[5] =
                (uint8_t)((g_lithium.charger_diagnostic_flags >> 8) &
                          0xFFU);


        data[6] =
                diagnostic_text_length;


        data[7] =
                diagnostic_segment_count;


        if (!Lithium_CAN_SendFrame(
                Lithium_CAN_GetIdentifier(
                        LITHIUM_CAN_OFFSET_DIAGNOSTIC_EVENT),
                data))
        {
            return true;
        }


        if (diagnostic_segment_count == 0U)
        {
            diagnostic_tx_state =
                    CAN_DIAG_IDLE;
        }
        else
        {
            diagnostic_segment_index =
                    0U;


            diagnostic_tx_state =
                    CAN_DIAG_SEND_TEXT;
        }


        return true;
    }


    if (diagnostic_tx_state ==
        CAN_DIAG_SEND_TEXT)
    {
        uint16_t text_offset;
        uint8_t i;
        bool final_segment;


        text_offset =
                (uint16_t)diagnostic_segment_index *
                LITHIUM_CAN_DIAG_TEXT_BYTES_PER_FRAME;


        final_segment =
                ((diagnostic_segment_index + 1U) >=
                 diagnostic_segment_count);


        data[0] =
                diagnostic_sequence;


        data[1] =
                diagnostic_segment_index;


        if (final_segment)
        {
            data[1] |=
                    LITHIUM_CAN_DIAG_FINAL_SEGMENT_FLAG;
        }


        for (i = 0U;
             i < LITHIUM_CAN_DIAG_TEXT_BYTES_PER_FRAME;
             i++)
        {
            uint16_t character_index;


            character_index =
                    text_offset +
                    i;


            if ((diagnostic_text != NULL) &&
                (character_index <
                 diagnostic_text_length))
            {
                data[2U + i] =
                        (uint8_t)diagnostic_text[character_index];
            }
            else
            {
                data[2U + i] =
                        0U;
            }
        }


        if (!Lithium_CAN_SendFrame(
                Lithium_CAN_GetIdentifier(
                        LITHIUM_CAN_OFFSET_DIAGNOSTIC_TEXT),
                data))
        {
            return true;
        }


        diagnostic_segment_index++;


        if (diagnostic_segment_index >=
            diagnostic_segment_count)
        {
            diagnostic_tx_state =
                    CAN_DIAG_IDLE;
        }


        return true;
    }


    diagnostic_tx_state =
            CAN_DIAG_IDLE;


    return false;
}


static void Lithium_CAN_BeginNextDiagnostic(void)
{
    uint32_t selected_fault;
    size_t text_length;


    selected_fault =
            0U;


    diagnostic_event_state =
            LITHIUM_CAN_DIAG_EVENT_CLEARED;


    /*
     * Active events are emitted in severity order.
     */
    if ((pending_active_faults &
         g_lithium.latched_fault_flags) != 0U)
    {
        selected_fault =
                Lithium_CAN_GetLowestSetBit(
                        pending_active_faults &
                        g_lithium.latched_fault_flags);
    }
    else if ((pending_active_faults &
              g_lithium.recoverable_fault_flags) != 0U)
    {
        selected_fault =
                Lithium_CAN_GetLowestSetBit(
                        pending_active_faults &
                        g_lithium.recoverable_fault_flags);
    }
    else if ((pending_active_faults &
              g_lithium.warning_fault_flags) != 0U)
    {
        selected_fault =
                Lithium_CAN_GetLowestSetBit(
                        pending_active_faults &
                        g_lithium.warning_fault_flags);
    }
    else if (pending_active_faults != 0U)
    {
        selected_fault =
                Lithium_CAN_GetLowestSetBit(
                        pending_active_faults);
    }


    if (selected_fault != 0U)
    {
        pending_active_faults &=
                ~selected_fault;


        diagnostic_event_state =
                LITHIUM_CAN_DIAG_EVENT_ACTIVE;


        diagnostic_severity =
                Lithium_CAN_GetFaultSeverity(
                        selected_fault);
    }
    else if (pending_cleared_faults != 0U)
    {
        selected_fault =
                Lithium_CAN_GetLowestSetBit(
                        pending_cleared_faults);


        pending_cleared_faults &=
                ~selected_fault;


        diagnostic_event_state =
                LITHIUM_CAN_DIAG_EVENT_CLEARED;


        diagnostic_severity =
                FAULT_SEVERITY_NONE;
    }
    else
    {
        return;
    }


    diagnostic_fault =
            selected_fault;


    diagnostic_sequence++;


    if (diagnostic_event_state ==
        LITHIUM_CAN_DIAG_EVENT_ACTIVE)
    {
        diagnostic_text =
                Lithium_CAN_GetFaultText(
                        diagnostic_fault,
                        diagnostic_severity);
    }
    else
    {
        diagnostic_text =
                NULL;
    }


    if (diagnostic_text == NULL)
    {
        diagnostic_text_length =
                0U;


        diagnostic_segment_count =
                0U;
    }
    else
    {
        text_length =
                strlen(
                        diagnostic_text);


        if (text_length >
            LITHIUM_CAN_DIAG_MAX_TEXT_LENGTH)
        {
            text_length =
                    LITHIUM_CAN_DIAG_MAX_TEXT_LENGTH;
        }


        diagnostic_text_length =
                (uint8_t)text_length;


        diagnostic_segment_count =
                (uint8_t)((diagnostic_text_length +
                           LITHIUM_CAN_DIAG_TEXT_BYTES_PER_FRAME -
                           1U) /
                          LITHIUM_CAN_DIAG_TEXT_BYTES_PER_FRAME);
    }


    diagnostic_tx_state =
            CAN_DIAG_SEND_HEADER;
}


/* ============================================================
 * HUMAN-READABLE FAULT TEXT
 *
 * The LED reports severity only. These messages provide the
 * exact operator-facing explanation over CAN.
 * ============================================================ */

static const char *Lithium_CAN_GetFaultText(
        uint32_t fault,
        fault_severity_t severity)
{
    if (fault ==
        LITHIUM_FAULT_BQ25750_INIT)
    {
        if (severity ==
            FAULT_SEVERITY_WARNING)
        {
            return
                    "BQ25750 init attempt failed; charging remains disabled during startup.";
        }


        if (severity ==
            FAULT_SEVERITY_RECOVERABLE)
        {
            return
                    "BQ25750 startup initialization failed; controlled recovery is running.";
        }


        return
                "BQ25750 initialization failed after recovery attempts; reset and inspect hardware.";
    }


    if (fault ==
        LITHIUM_FAULT_BQ25750_COMM)
    {
        if (severity ==
            FAULT_SEVERITY_LATCHED)
        {
            return
                    "BQ25750 communication did not recover after retries; reset and inspect I2C/controller.";
        }


        return
                "BQ25750 communication lost or stale; charging is disabled while controller recovery runs.";
    }


    if (fault ==
        LITHIUM_FAULT_BQ76942_INIT)
    {
        if (severity ==
            FAULT_SEVERITY_WARNING)
        {
            return
                    "BQ76942 init attempt failed; cell monitoring is not yet qualified.";
        }


        if (severity ==
            FAULT_SEVERITY_RECOVERABLE)
        {
            return
                    "BQ76942 startup initialization failed; controlled recovery is running.";
        }


        return
                "BQ76942 initialization failed after recovery attempts; reset and inspect hardware.";
    }


    if (fault ==
        LITHIUM_FAULT_BQ76942_COMM)
    {
        if (severity ==
            FAULT_SEVERITY_LATCHED)
        {
            return
                    "BQ76942 communication did not recover after retries; cell safety data remains unavailable.";
        }


        return
                "BQ76942 communication lost or stale; cell safety is unknown and charging/balancing are disabled.";
    }


    if (fault ==
        LITHIUM_FAULT_CELL_OVERVOLTAGE)
    {
        if (severity ==
            FAULT_SEVERITY_LATCHED)
        {
            return
                    "Cell group reached 4.250 V severe overvoltage; charging is latched off.";
        }


        if ((g_lithium.charger_diagnostic_flags &
             LITHIUM_CHARGER_DIAG_BQ_VBAT_OV) != 0U)
        {
            return
                    "BQ25750 BAT_OVP backup is active; charging is inhibited while cell-level safety monitoring remains active.";
        }


        if (g_lithium.measurements.bq76942_cov_active)
        {
            return
                    "BQ76942 COV backup is active near 4.20 V; charging is inhibited and safe balancing may continue.";
        }


        return
                "Cell group reached 4.170 V; charging is inhibited until max cell falls to 4.150 V or lower.";
    }


    if (fault ==
        LITHIUM_FAULT_CELL_UNDERVOLTAGE)
    {
        if (g_lithium.measurements.bq76942_cuv_active)
        {
            return
                    "BQ76942 CUV backup is active near 2.53 V; low-cell recovery is limited to 1.0 A when allowed.";
        }


        return
                "Cell group is below 3.00 V; low-cell recovery charging is limited to 1.0 A.";
    }


    if (fault ==
        LITHIUM_FAULT_CELL_IMBALANCE)
    {
        return
                "Cell-group imbalance fault is active; inspect group voltages and balancing operation.";
    }


    if (fault ==
        LITHIUM_FAULT_TEMP_HIGH)
    {
        if (severity ==
            FAULT_SEVERITY_LATCHED)
        {
            return
                    "Confirmed battery temperature reached 60 C; critical thermal fault is latched.";
        }


        if ((g_lithium.charger_diagnostic_flags &
             LITHIUM_CHARGER_DIAG_BQ_TSHUT) != 0U)
        {
            return
                    "BQ25750 thermal shutdown is active; charging is disabled and the system remains in thermal recovery.";
        }


        return
                "Confirmed battery temperature reached 55 C; charging and balancing are disabled until thermal recovery.";
    }


    if (fault ==
        LITHIUM_FAULT_TEMP_LOW)
    {
        return
                "Confirmed battery temperature is below 0 C; charging is disabled until safe thermal recovery.";
    }


    if (fault ==
        LITHIUM_FAULT_CHARGER)
    {
        return
                Lithium_CAN_GetChargerFaultText();
    }


    if (fault ==
        LITHIUM_FAULT_INPUT_POWER)
    {
        if ((g_lithium.charger_diagnostic_flags &
             LITHIUM_CHARGER_DIAG_BQ_VAC_UV) != 0U)
        {
            return
                    "BQ25750 VAC_UV input protection is active; input voltage is too low and charging may be reduced or inhibited.";
        }


        return
                "Umbilical VIN is below 21 V while input is present; source droop may reduce charging through DPM.";
    }


    if (fault ==
        LITHIUM_FAULT_INVALID_MEASUREMENT)
    {
        return
                "Required battery measurement is invalid; charging/balancing decisions are not trusted.";
    }


    if (fault ==
        LITHIUM_FAULT_INTERNAL_STATE)
    {
        return
                "Firmware entered an invalid internal state; application fault is latched.";
    }


    if (fault ==
        LITHIUM_FAULT_TEMP_SENSOR)
    {
        if (severity ==
            FAULT_SEVERITY_WARNING)
        {
            return
                    Lithium_CAN_GetLocalThermistorWarningText();
        }


        return
                "Temperature-sensor redundancy is insufficient; charging and balancing are disabled.";
    }


    if (fault ==
        LITHIUM_FAULT_BATTERY_OVERCURRENT)
    {
        return
        		"Battery current reached the 19.5 A extreme-current diagnostic limit; fault is latched. External fuse remains primary short-circuit protection.";    }


    if (fault ==
        LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE)
    {
        return
                "Cell group is below 2.50 V; deep undervoltage is latched and automatic charging is blocked.";
    }


    if (fault ==
        LITHIUM_FAULT_CHARGE_TIMEOUT)
    {
        return
                Lithium_CAN_GetChargeTimeoutText();
    }


    if (fault ==
        LITHIUM_FAULT_INPUT_OVERVOLTAGE)
    {
        if ((g_lithium.charger_diagnostic_flags &
             LITHIUM_CHARGER_DIAG_BQ_VAC_OV) != 0U)
        {
            return
                    "BQ25750 VAC_OV input protection is active; charging is inhibited while the input overvoltage condition remains.";
        }


        if (severity ==
            FAULT_SEVERITY_RECOVERABLE)
        {
            return
                    "Umbilical VIN reached 30 V; input HIZ/isolation is active and battery supplies VSYS until recovery.";
        }


        return
                "Umbilical VIN is at least 27 V; high-input warning. Charging is blocked above 28 V.";
    }


    if (fault ==
        LITHIUM_FAULT_PACK_TEMP_BACKUP)
    {
        return
                Lithium_CAN_GetPackCenterThermistorText();
    }


    if (fault ==
        LITHIUM_FAULT_PACK_VOLTAGE_MISMATCH)
    {
        return
                "BQ76942 cell-sum and BQ25750 VBAT differ by over 200 mV for 3 checks; charging/balancing are disabled.";
    }


    if (fault ==
        LITHIUM_FAULT_CAN)
    {
        return
                "CAN telemetry link is unavailable; local warning is active and automatic CAN restart attempts continue.";
    }


    return
            "Unknown LITHIUM fault flag is active; inspect raw fault masks and firmware version.";
}


static const char *Lithium_CAN_GetLocalThermistorWarningText(void)
{
    bool th1_failed;
    bool th2_failed;
    bool th3_failed;


    th1_failed =
            (g_lithium.measurements.temperature_status[0] ==
             TEMP_SENSOR_STATUS_FAULT);


    th2_failed =
            (g_lithium.measurements.temperature_status[1] ==
             TEMP_SENSOR_STATUS_FAULT);


    th3_failed =
            (g_lithium.measurements.temperature_status[2] ==
             TEMP_SENSOR_STATUS_FAULT);


    if (th1_failed &&
        (!th2_failed) &&
        (!th3_failed))
    {
        return
                "TH1 thermistor failed; TH2, TH3 and pack-center sensor remain valid. Charging remains permitted in degraded mode.";
    }


    if (th2_failed &&
        (!th1_failed) &&
        (!th3_failed))
    {
        return
                "TH2 thermistor failed; TH1, TH3 and pack-center sensor remain valid. Charging remains permitted in degraded mode.";
    }


    if (th3_failed &&
        (!th1_failed) &&
        (!th2_failed))
    {
        return
                "TH3 thermistor failed; TH1, TH2 and pack-center sensor remain valid. Charging remains permitted in degraded mode.";
    }


    return
            "One local thermistor failed; three trusted thermal sources remain, so degraded charging is still allowed.";
}


static const char *Lithium_CAN_GetPackCenterThermistorText(void)
{
    bool all_local_sensors_available;


    all_local_sensors_available =
            (g_lithium.measurements.temperature_status[0] !=
             TEMP_SENSOR_STATUS_FAULT) &&
            (g_lithium.measurements.temperature_status[1] !=
             TEMP_SENSOR_STATUS_FAULT) &&
            (g_lithium.measurements.temperature_status[2] !=
             TEMP_SENSOR_STATUS_FAULT);


    if (all_local_sensors_available)
    {
        return
                "Pack-center 103AT-2 thermistor is invalid; TH1, TH2 and TH3 remain available for thermal supervision.";
    }


    return
            "Pack-center 103AT-2 thermistor is invalid and local-sensor redundancy is also reduced; check TH1/TH2/TH3 status.";
}


static const char *Lithium_CAN_GetChargerFaultText(void)
{
    uint16_t diagnostics;
    uint16_t recoverable_diagnostics;


    diagnostics =
            g_lithium.charger_diagnostic_flags;


    recoverable_diagnostics =
            diagnostics &
            LITHIUM_CHARGER_DIAG_RECOVERABLE_MASK;


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_BQ_IBAT_OCP) != 0U)
    {
        return
                "BQ25750 BAT_OCP charger-side protection triggered; charging is disabled while controlled recovery runs.";
    }


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_BQ_DRV_OKZ) != 0U)
    {
        return
                "BQ25750 DRV_OKZ fault is active; charger drive supply is not valid and controlled controller recovery is running.";
    }


    switch (recoverable_diagnostics)
    {
        case LITHIUM_CHARGER_DIAG_CONFIG_FAILURE:

            return
                    "BQ25750 charger configuration/readback failed; charging is disabled and controller re-init is running.";


        case LITHIUM_CHARGER_DIAG_WATCHDOG_EXPIRED:

            return
                    "BQ25750 charger watchdog expired; charging is disabled and controller re-init is running.";


        case LITHIUM_CHARGER_DIAG_DEVICE_FAULT:

            return
                    "BQ25750 reported a charger/device fault; charging is disabled and controller re-init is running.";


        case (LITHIUM_CHARGER_DIAG_CONFIG_FAILURE |
              LITHIUM_CHARGER_DIAG_WATCHDOG_EXPIRED):

            return
                    "BQ25750 configuration failure and watchdog expiry are both active; charging is disabled and re-init is running.";


        case (LITHIUM_CHARGER_DIAG_CONFIG_FAILURE |
              LITHIUM_CHARGER_DIAG_DEVICE_FAULT):

            return
                    "BQ25750 configuration failure and device fault are both active; charging is disabled and re-init is running.";


        case (LITHIUM_CHARGER_DIAG_WATCHDOG_EXPIRED |
              LITHIUM_CHARGER_DIAG_DEVICE_FAULT):

            return
                    "BQ25750 watchdog expiry and device fault are both active; charging is disabled and re-init is running.";


        case LITHIUM_CHARGER_DIAG_RECOVERABLE_MASK:

            return
                    "BQ25750 configuration, watchdog and device faults are active; charging is disabled and re-init is running.";


        default:

            return
                    "BQ25750 charger fault is active; charging is disabled while controlled controller recovery runs.";
    }
}


static const char *Lithium_CAN_GetChargeTimeoutText(void)
{
    uint16_t diagnostics;


    diagnostics =
            g_lithium.charger_diagnostic_flags;


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_LOW_CELL_TIMEOUT) != 0U)
    {
        return
                "Low-cell recovery exceeded 30 minutes; automatic recovery retry is blocked until reset/inspection.";
    }


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_FAST_CHARGE_TIMEOUT) != 0U)
    {
        return
                "Fast-charge safety timer expired after 8 hours; charging is latched off.";
    }


    if ((diagnostics &
         LITHIUM_CHARGER_DIAG_CV_TIMEOUT) != 0U)
    {
        return
                "CV-stage safety timer expired after 2 hours; charging is latched off.";
    }


    return
            "Charging safety timeout is active; charging is latched off until reset/inspection.";
}
