#include "charger_fsm.h"

#include "main.h"

#include "bq25750.h"
#include "lithium_config.h"


/* ============================================================
 * BQ25750 RAW FAULT STATUS FILTER
 *
 * REG0x24_Fault_Status bit 1 is DRV_OKZ_STAT.  The other
 * currently-used protection bits (VAC_UV, VAC_OV, IBAT_OCP,
 * VBAT_OV, TSHUT and CHG_TMR) are handled explicitly by the
 * application layer so they retain their existing system-fault
 * semantics instead of being collapsed into a generic charger
 * fault.
 * ============================================================ */

#define CHARGER_FSM_BQ25750_DRV_OKZ_FAULT_MASK          (1U << 1)


/* ============================================================
 * LOCAL CHARGER STATE
 * ============================================================ */

static bool hot_inhibit = false;
static bool cold_inhibit = false;
static bool high_cell_inhibit = false;

static uint8_t hot_recovery_count = 0U;
static uint8_t cold_recovery_count = 0U;
static uint8_t high_cell_recovery_count = 0U;


static uint16_t programmed_current_ma = 0U;
static uint16_t programmed_vfb_mv = 0U;


static uint32_t low_cell_recovery_start_ms = 0U;
static uint32_t charge_ramp_last_step_ms = 0U;


/* ============================================================
 * PRIVATE FUNCTIONS
 * ============================================================ */

static bool ChargerFSM_CanCharge(
        const lithium_context_t *ctx);

static bool ChargerFSM_NeedsLowCellRecovery(
        const lithium_context_t *ctx);

static uint16_t ChargerFSM_SelectCurrentMa(
        const lithium_context_t *ctx);

static uint16_t ChargerFSM_SelectVfbMv(
        const lithium_context_t *ctx);

static void ChargerFSM_UpdateInhibits(
        const lithium_context_t *ctx);

static uint16_t ChargerFSM_GetMinCellMv(
        const lithium_context_t *ctx);

static uint16_t ChargerFSM_GetMaxCellMv(
        const lithium_context_t *ctx);

static int16_t ChargerFSM_GetMinTrustedTemperaturedC(
        const lithium_context_t *ctx);

static int16_t ChargerFSM_GetMaxTrustedTemperaturedC(
        const lithium_context_t *ctx);

static uint8_t ChargerFSM_CountTrustedTemperatureSources(
        const lithium_context_t *ctx);

static bool ChargerFSM_HasBlockingFault(
        const lithium_context_t *ctx);

static bool ChargerFSM_Enable(
        uint16_t current_ma,
        uint16_t vfb_mv);

static void ChargerFSM_Disable(void);


/* ============================================================
 * INIT
 * ============================================================ */

void ChargerFSM_Init(
        lithium_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }


    ctx->charger_state =
            CHARGE_OFF;


    hot_inhibit =
            false;

    cold_inhibit =
            false;

    high_cell_inhibit =
            false;


    hot_recovery_count =
            0U;

    cold_recovery_count =
            0U;

    high_cell_recovery_count =
            0U;


    programmed_current_ma =
            0U;

    programmed_vfb_mv =
            0U;


    low_cell_recovery_start_ms =
            0U;

    charge_ramp_last_step_ms =
            0U;


    ctx->charger_diagnostic_flags =
            LITHIUM_CHARGER_DIAG_NONE;


    ChargerFSM_Disable();
}


/* ============================================================
 * RUN
 * ============================================================ */

void ChargerFSM_Run(
        lithium_context_t *ctx)
{
    uint16_t requested_current_ma;
    uint16_t requested_vfb_mv;
    uint16_t applied_current_ma;
    uint16_t previous_current_ma;

    uint32_t now;


    if (ctx == NULL)
    {
        return;
    }


    now =
            HAL_GetTick();


    ChargerFSM_UpdateInhibits(
            ctx);


    /*
     * Charging exists only during valid ground/input operation.
     */
    if ((!ctx->input_present) ||
        (ctx->operating_mode !=
         OPERATING_MODE_GROUND))
    {
        ChargerFSM_ForceOff(
                ctx);

        return;
    }


    switch (ctx->charger_state)
    {
        /* ----------------------------------------------------
         * OFF
         * ---------------------------------------------------- */

        case CHARGE_OFF:

            ChargerFSM_Disable();


            if (ChargerFSM_CanCharge(
                    ctx))
            {
                ctx->charger_state =
                        CHARGE_CHECK;
            }

            break;


        /* ----------------------------------------------------
         * CHECK
         * ---------------------------------------------------- */

        case CHARGE_CHECK:

            ChargerFSM_Disable();


            if (!ChargerFSM_CanCharge(
                    ctx))
            {
                ctx->charger_state =
                        CHARGE_OFF;

                break;
            }


            if (ChargerFSM_NeedsLowCellRecovery(
                    ctx))
            {
                low_cell_recovery_start_ms =
                        now;

                ctx->charger_state =
                        CHARGE_LOW_CELL_RECOVERY;

                break;
            }


            requested_current_ma =
                    ChargerFSM_SelectCurrentMa(
                            ctx);

            requested_vfb_mv =
                    ChargerFSM_SelectVfbMv(
                            ctx);


            /*
             * Normal charging always starts gently.  The existing
             * permission checks above remain authoritative; this
             * only limits how quickly the requested charge-current
             * ceiling is increased after permission is granted.
             *
             * Low-cell recovery is handled by its dedicated 1-A
             * state and therefore does not pass through this ramp.
             */
            applied_current_ma =
                    requested_current_ma;


            if ((LITHIUM_CHARGE_STARTUP_RAMP_ENABLE != 0U) &&
                (applied_current_ma >
                 LITHIUM_CHARGE_STARTUP_RAMP_INITIAL_MA))
            {
                applied_current_ma =
                        LITHIUM_CHARGE_STARTUP_RAMP_INITIAL_MA;
            }


            if (ChargerFSM_Enable(
                    applied_current_ma,
                    requested_vfb_mv))
            {
                charge_ramp_last_step_ms =
                        now;

                ctx->charger_state =
                        CHARGE_ACTIVE;
            }
            else
            {
                ctx->charger_diagnostic_flags |=
                        LITHIUM_CHARGER_DIAG_CONFIG_FAILURE;

                ctx->charger_state =
                        CHARGE_FAULT;
            }

            break;


        /* ----------------------------------------------------
         * LOW CELL RECOVERY
         * ---------------------------------------------------- */

        case CHARGE_LOW_CELL_RECOVERY:

            if (!ChargerFSM_CanCharge(
                    ctx))
            {
                ChargerFSM_Disable();

                ctx->charger_state =
                        CHARGE_OFF;

                break;
            }


            /*
             * Deep UV is not automatically recoverable.
             */
            if (ChargerFSM_GetMinCellMv(ctx) <
                LITHIUM_CELL_UV_LATCH_MV)
            {
                ChargerFSM_Disable();

                ctx->charger_state =
                        CHARGE_FAULT;

                break;
            }


            if ((now -
                 low_cell_recovery_start_ms) >=
                LITHIUM_LOW_CELL_RECOVERY_TIMEOUT_MS)
            {
                ChargerFSM_Disable();


                /*
                 * Maximum automatic low-cell recovery attempt
                 * has been exhausted.  Preserve the exact reason
                 * for the System FSM; this event is intentionally
                 * not auto-retried.
                 */
                ctx->charger_diagnostic_flags |=
                        LITHIUM_CHARGER_DIAG_LOW_CELL_TIMEOUT;


                ctx->charger_state =
                        CHARGE_FAULT;

                break;
            }


            /*
             * All groups recovered above the operational
             * low-cell boundary.
             */
            if (ChargerFSM_GetMinCellMv(ctx) >=
                LITHIUM_CELL_UV_WARNING_MV)
            {
                ChargerFSM_Disable();

                ctx->charger_state =
                        CHARGE_CHECK;

                break;
            }


            requested_vfb_mv =
                    ChargerFSM_SelectVfbMv(
                            ctx);


            if ((programmed_current_ma !=
                 LITHIUM_LOW_CELL_RECOVERY_CURRENT_MA) ||
                (programmed_vfb_mv !=
                 requested_vfb_mv))
            {
                if (!ChargerFSM_Enable(
                        LITHIUM_LOW_CELL_RECOVERY_CURRENT_MA,
                        requested_vfb_mv))
                {
                    ctx->charger_diagnostic_flags |=
                            LITHIUM_CHARGER_DIAG_CONFIG_FAILURE;

                    ctx->charger_state =
                            CHARGE_FAULT;
                }
            }

            break;


        /* ----------------------------------------------------
         * ACTIVE
         * ---------------------------------------------------- */

        case CHARGE_ACTIVE:

            if (!ChargerFSM_CanCharge(
                    ctx))
            {
                ChargerFSM_Disable();

                ctx->charger_state =
                        CHARGE_OFF;

                break;
            }


            if (ChargerFSM_NeedsLowCellRecovery(
                    ctx))
            {
                ChargerFSM_Disable();

                low_cell_recovery_start_ms =
                        now;

                ctx->charger_state =
                        CHARGE_LOW_CELL_RECOVERY;

                break;
            }


            if (ctx->measurements.charger_watchdog_expired ||
                ((ctx->measurements.charger_fault_status &
                  CHARGER_FSM_BQ25750_DRV_OKZ_FAULT_MASK) != 0U))
            {
                ChargerFSM_Disable();


                if (ctx->measurements.charger_watchdog_expired)
                {
                    ctx->charger_diagnostic_flags |=
                            LITHIUM_CHARGER_DIAG_WATCHDOG_EXPIRED;
                }


                if ((ctx->measurements.charger_fault_status &
                     CHARGER_FSM_BQ25750_DRV_OKZ_FAULT_MASK) != 0U)
                {
                    ctx->charger_diagnostic_flags |=
                            LITHIUM_CHARGER_DIAG_DEVICE_FAULT |
                            LITHIUM_CHARGER_DIAG_BQ_DRV_OKZ;
                }


                ctx->charger_state =
                        CHARGE_FAULT;

                break;
            }


            /*
             * BQ charger charge-state interpretation is handled
             * conservatively here:
             *
             * BQ25750 CHARGE_STAT = 111b explicitly means
             * Charge Termination Done.
             *
             * We additionally require low measured battery
             * current as a plausibility check before moving the
             * application FSM to CHARGE_DONE.
             */
            if (ctx->measurements.charger_charge_state == 7U)
            {
                if ((programmed_current_ma != 0U) &&
                    (ctx->measurements.battery_current_ma >= 0) &&
                    (ctx->measurements.battery_current_ma <=
                     (int32_t)
                     LITHIUM_TERMINATION_CURRENT_REGISTER_MA))
                {
                    ChargerFSM_Disable();

                    ctx->charger_state =
                            CHARGE_DONE;

                    break;
                }
            }


            requested_current_ma =
                    ChargerFSM_SelectCurrentMa(
                            ctx);

            requested_vfb_mv =
                    ChargerFSM_SelectVfbMv(
                            ctx);


            /*
             * Downward current changes (thermal derating, degraded
             * sensing, etc.) are applied immediately.  Upward
             * changes are limited to one configured ramp step per
             * interval.  Because ChargerFSM_CanCharge() and the raw
             * charger-fault checks have already passed in this loop,
             * every upward step is preceded by the same health gates
             * used for normal charging.
             */
            applied_current_ma =
                    requested_current_ma;


            if ((LITHIUM_CHARGE_STARTUP_RAMP_ENABLE != 0U) &&
                (requested_current_ma >
                 programmed_current_ma))
            {
                applied_current_ma =
                        programmed_current_ma;


                if ((now -
                     charge_ramp_last_step_ms) >=
                    LITHIUM_CHARGE_STARTUP_RAMP_STEP_MS)
                {
                    uint32_t stepped_current_ma;


                    stepped_current_ma =
                            (uint32_t)programmed_current_ma +
                            (uint32_t)LITHIUM_CHARGE_STARTUP_RAMP_STEP_MA;


                    if (stepped_current_ma >
                        requested_current_ma)
                    {
                        stepped_current_ma =
                                requested_current_ma;
                    }


                    applied_current_ma =
                            (uint16_t)stepped_current_ma;
                }
            }


            if ((applied_current_ma !=
                 programmed_current_ma) ||
                (requested_vfb_mv !=
                 programmed_vfb_mv))
            {
                previous_current_ma =
                        programmed_current_ma;


                if (!ChargerFSM_Enable(
                        applied_current_ma,
                        requested_vfb_mv))
                {
                    ChargerFSM_Disable();


                    ctx->charger_diagnostic_flags |=
                            LITHIUM_CHARGER_DIAG_CONFIG_FAILURE;


                    ctx->charger_state =
                            CHARGE_FAULT;
                }
                else if (applied_current_ma !=
                         previous_current_ma)
                {
                    charge_ramp_last_step_ms =
                            now;
                }
            }

            break;


        /* ----------------------------------------------------
         * DONE
         * ---------------------------------------------------- */

        case CHARGE_DONE:

            ChargerFSM_Disable();


            if (!ChargerFSM_CanCharge(
                    ctx))
            {
                break;
            }


            /*
             * Do not repeatedly restart the charger because of
             * normal post-charge voltage relaxation.
             */
            if (ChargerFSM_GetMaxCellMv(ctx) <=
                LITHIUM_CELL_RECHARGE_MV)
            {
                ctx->charger_state =
                        CHARGE_CHECK;
            }

            break;


        /* ----------------------------------------------------
         * FAULT
         * ---------------------------------------------------- */

        case CHARGE_FAULT:

            ChargerFSM_Disable();


            /*
             * CHARGE_FAULT never self-clears.
             *
             * The exact cause has already been stored in
             * charger_diagnostic_flags.  lithium_app.c owns the
             * global severity / recovery decision.  Recoverable
             * charger faults are cleared only after controlled
             * BQ25750 recovery and SELF_TEST; timeout faults are
             * latched and require reset / inspection.
             */
            break;


        default:

            ChargerFSM_ForceOff(
                    ctx);

            break;
    }
}


/* ============================================================
 * FORCE OFF
 * ============================================================ */

void ChargerFSM_ForceOff(
        lithium_context_t *ctx)
{
    ChargerFSM_Disable();


    low_cell_recovery_start_ms =
            0U;

    charge_ramp_last_step_ms =
            0U;


    if (ctx != NULL)
    {
        ctx->charger_state =
                CHARGE_OFF;
    }
}


/* ============================================================
 * CAN CHARGE
 * ============================================================ */

static bool ChargerFSM_CanCharge(
        const lithium_context_t *ctx)
{
    int16_t min_temp_dC;
    int16_t max_temp_dC;

    uint8_t trusted_temperature_sources;


    if (ctx == NULL)
    {
        return false;
    }


    if (ctx->system_state !=
        SYS_READY)
    {
        return false;
    }


    if ((!ctx->input_present) ||
        (ctx->operating_mode !=
         OPERATING_MODE_GROUND))
    {
        return false;
    }


    if ((!ctx->bq25750_initialized) ||
        (!ctx->bq76942_initialized))
    {
        return false;
    }


    if ((!ctx->measurements.bq25750_valid) ||
        (!ctx->measurements.bq76942_valid))
    {
        return false;
    }


    if (ChargerFSM_HasBlockingFault(
            ctx))
    {
        return false;
    }


    if (high_cell_inhibit ||
        hot_inhibit ||
        cold_inhibit)
    {
        return false;
    }


    if (ChargerFSM_GetMinCellMv(ctx) <
        LITHIUM_CELL_UV_LATCH_MV)
    {
        return false;
    }


    /*
     * Independent BQ76942 COV comparator backup.
     *
     * This gate does not require the application to enter a
     * recoverable global fault state, so safe passive balancing
     * of the high cell can continue while charging is inhibited.
     */
    if (ctx->measurements.bq76942_cov_active)
    {
        return false;
    }


    trusted_temperature_sources =
            ChargerFSM_CountTrustedTemperatureSources(
                    ctx);


    /*
     * Normal operation has four independent thermal sources:
     *
     * TH1 / TH2 / TH3 + pack-center backup.
     *
     * Charging is still permitted with exactly one confirmed
     * failed source, but never with fewer than three trusted
     * sources.
     */
    if (trusted_temperature_sources < 3U)
    {
        return false;
    }


    min_temp_dC =
            ChargerFSM_GetMinTrustedTemperaturedC(
                    ctx);

    max_temp_dC =
            ChargerFSM_GetMaxTrustedTemperaturedC(
                    ctx);


    if ((min_temp_dC <
         LITHIUM_CHARGE_TEMP_MIN_dC) ||
        (max_temp_dC >=
         LITHIUM_CHARGE_TEMP_MAX_dC))
    {
        return false;
    }


    if (ctx->measurements.vin_mv <
        LITHIUM_INPUT_VIN_MIN_MV)
    {
        return false;
    }


    if (ctx->measurements.vin_mv >
        LITHIUM_INPUT_CHARGE_MAX_MV)
    {
        return false;
    }


    return true;
}


/* ============================================================
 * LOW CELL RECOVERY
 * ============================================================ */

static bool ChargerFSM_NeedsLowCellRecovery(
        const lithium_context_t *ctx)
{
    uint16_t min_cell_mv;


    min_cell_mv =
            ChargerFSM_GetMinCellMv(
                    ctx);


    return
            (ctx->measurements.bq76942_cuv_active ||
             ((min_cell_mv >=
               LITHIUM_CELL_UV_LATCH_MV) &&
              (min_cell_mv <
               LITHIUM_CELL_UV_WARNING_MV)));
}


/* ============================================================
 * CURRENT SELECTION
 * ============================================================ */

static uint16_t ChargerFSM_SelectCurrentMa(
        const lithium_context_t *ctx)
{
    int16_t max_temp_dC;

    uint16_t selected_current_ma;


    max_temp_dC =
            ChargerFSM_GetMaxTrustedTemperaturedC(
                    ctx);


    if (max_temp_dC <
        LITHIUM_CHARGE_TEMP_COLD_DERATE_END_dC)
    {
        selected_current_ma =
                LITHIUM_CHARGE_CURRENT_COLD_MA;
    }
    else if (max_temp_dC <
             LITHIUM_CHARGE_TEMP_FULL_END_dC)
    {
        selected_current_ma =
                LITHIUM_CHARGE_CURRENT_NORMAL_MA;
    }
    else if (max_temp_dC <
             LITHIUM_CHARGE_TEMP_WARM1_END_dC)
    {
        selected_current_ma =
                LITHIUM_CHARGE_CURRENT_WARM1_MA;
    }
    else if (max_temp_dC <
             LITHIUM_CHARGE_TEMP_WARM2_END_dC)
    {
        selected_current_ma =
                LITHIUM_CHARGE_CURRENT_WARM2_MA;
    }
    else
    {
        selected_current_ma =
                LITHIUM_CHARGE_CURRENT_HOT_MA;
    }


    /*
     * Exactly one failed cell-group thermistor is allowed only
     * as a degraded operating mode while the remaining thermal
     * sources agree.
     *
     * Keep charging deliberately gentle in this condition.
     */
    if (((ctx->warning_fault_flags &
          LITHIUM_FAULT_TEMP_SENSOR) != 0U) &&
        (selected_current_ma >
         LITHIUM_CHARGE_CURRENT_SENSOR_DEGRADED_MA))
    {
        selected_current_ma =
                LITHIUM_CHARGE_CURRENT_SENSOR_DEGRADED_MA;
    }


    return selected_current_ma;
}


/* ============================================================
 * VOLTAGE SELECTION
 * ============================================================ */

static uint16_t ChargerFSM_SelectVfbMv(
        const lithium_context_t *ctx)
{
    int16_t max_temp_dC;


    max_temp_dC =
            ChargerFSM_GetMaxTrustedTemperaturedC(
                    ctx);


    if (max_temp_dC >=
        LITHIUM_CHARGE_TEMP_WARM1_END_dC)
    {
        return
                LITHIUM_CHARGE_VFB_WARM_MV;
    }


    return
            LITHIUM_CHARGE_VFB_NORMAL_MV;
}


/* ============================================================
 * LOCAL HYSTERESIS / INHIBITS
 * ============================================================ */

static void ChargerFSM_UpdateInhibits(
        const lithium_context_t *ctx)
{
    uint16_t max_cell_mv;

    int16_t min_temp_dC;
    int16_t max_temp_dC;


    if (ctx == NULL)
    {
        return;
    }


    max_cell_mv =
            ChargerFSM_GetMaxCellMv(
                    ctx);


    min_temp_dC =
            ChargerFSM_GetMinTrustedTemperaturedC(
                    ctx);

    max_temp_dC =
            ChargerFSM_GetMaxTrustedTemperaturedC(
                    ctx);


    /* --------------------------------------------------------
     * HOT
     * -------------------------------------------------------- */

    if (max_temp_dC >=
        LITHIUM_CHARGE_TEMP_MAX_dC)
    {
        hot_inhibit =
                true;

        hot_recovery_count =
                0U;
    }
    else if (hot_inhibit)
    {
        if (max_temp_dC <=
            LITHIUM_TEMP_HOT_RECOVER_dC)
        {
            if (hot_recovery_count <
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                hot_recovery_count++;
            }


            if (hot_recovery_count >=
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                hot_inhibit =
                        false;

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
     * COLD
     * -------------------------------------------------------- */

    if (min_temp_dC <
        LITHIUM_CHARGE_TEMP_MIN_dC)
    {
        cold_inhibit =
                true;

        cold_recovery_count =
                0U;
    }
    else if (cold_inhibit)
    {
        if (min_temp_dC >=
            LITHIUM_TEMP_COLD_RECOVER_dC)
        {
            if (cold_recovery_count <
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                cold_recovery_count++;
            }


            if (cold_recovery_count >=
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                cold_inhibit =
                        false;

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


    /* --------------------------------------------------------
     * INDIVIDUAL CELL HIGH
     * -------------------------------------------------------- */

    if (max_cell_mv >=
        LITHIUM_CELL_CHARGE_INHIBIT_MV)
    {
        high_cell_inhibit =
                true;

        high_cell_recovery_count =
                0U;
    }
    else if (high_cell_inhibit)
    {
        if (max_cell_mv <=
            LITHIUM_CELL_CHARGE_RECOVER_MV)
        {
            if (high_cell_recovery_count <
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                high_cell_recovery_count++;
            }


            if (high_cell_recovery_count >=
                LITHIUM_MONITOR_CONFIRM_COUNT)
            {
                high_cell_inhibit =
                        false;

                high_cell_recovery_count =
                        0U;
            }
        }
        else
        {
            high_cell_recovery_count =
                    0U;
        }
    }
}


/* ============================================================
 * CELL HELPERS
 * ============================================================ */

static uint16_t ChargerFSM_GetMinCellMv(
        const lithium_context_t *ctx)
{
    uint16_t value;

    uint8_t i;


    value =
            ctx->measurements.cell_mv[0];


    for (i = 1U;
         i < 3U;
         i++)
    {
        if (ctx->measurements.cell_mv[i] <
            value)
        {
            value =
                    ctx->measurements.cell_mv[i];
        }
    }


    return value;
}


static uint16_t ChargerFSM_GetMaxCellMv(
        const lithium_context_t *ctx)
{
    uint16_t value;

    uint8_t i;


    value =
            ctx->measurements.cell_mv[0];


    for (i = 1U;
         i < 3U;
         i++)
    {
        if (ctx->measurements.cell_mv[i] >
            value)
        {
            value =
                    ctx->measurements.cell_mv[i];
        }
    }


    return value;
}


/* ============================================================
 * TRUSTED TEMPERATURE HELPERS
 *
 * Actual thermistor plausibility/cross-confirmation is handled
 * centrally by lithium_app.c.
 *
 * FSM uses only sensors which application marked valid.
 * ============================================================ */

static int16_t ChargerFSM_GetMinTrustedTemperaturedC(
        const lithium_context_t *ctx)
{
    int16_t value =
            1000;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if (ctx->measurements.temperature_status[i] ==
            TEMP_SENSOR_STATUS_VALID)
        {
            if (ctx->measurements.temperature_dC[i] <
                value)
            {
                value =
                        ctx->measurements.temperature_dC[i];
            }
        }
    }


    if (ctx->measurements.pack_center_temperature_status ==
        TEMP_SENSOR_STATUS_VALID)
    {
        if (ctx->measurements.pack_center_temperature_dC <
            value)
        {
            value =
                    ctx->measurements.pack_center_temperature_dC;
        }
    }


    return value;
}


static int16_t ChargerFSM_GetMaxTrustedTemperaturedC(
        const lithium_context_t *ctx)
{
    int16_t value =
            -1000;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if (ctx->measurements.temperature_status[i] ==
            TEMP_SENSOR_STATUS_VALID)
        {
            if (ctx->measurements.temperature_dC[i] >
                value)
            {
                value =
                        ctx->measurements.temperature_dC[i];
            }
        }
    }


    if (ctx->measurements.pack_center_temperature_status ==
        TEMP_SENSOR_STATUS_VALID)
    {
        if (ctx->measurements.pack_center_temperature_dC >
            value)
        {
            value =
                    ctx->measurements.pack_center_temperature_dC;
        }
    }


    return value;
}


/* ============================================================
 * TRUSTED TEMPERATURE SOURCE COUNT
 * ============================================================ */

static uint8_t ChargerFSM_CountTrustedTemperatureSources(
        const lithium_context_t *ctx)
{
    uint8_t count =
            0U;

    uint8_t i;


    for (i = 0U;
         i < 3U;
         i++)
    {
        if (ctx->measurements.temperature_status[i] ==
            TEMP_SENSOR_STATUS_VALID)
        {
            count++;
        }
    }


    if (ctx->measurements.pack_center_temperature_status ==
        TEMP_SENSOR_STATUS_VALID)
    {
        count++;
    }


    return count;
}


/* ============================================================
 * BLOCKING FAULTS
 * ============================================================ */

static bool ChargerFSM_HasBlockingFault(
        const lithium_context_t *ctx)
{
    const uint32_t blocking_mask =
            LITHIUM_FAULT_BQ25750_MASK |
            LITHIUM_FAULT_BQ76942_MASK |
            LITHIUM_FAULT_CELL_OVERVOLTAGE |
            LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE |
            LITHIUM_FAULT_TEMP_HIGH |
            LITHIUM_FAULT_TEMP_LOW |
            LITHIUM_FAULT_TEMP_SENSOR |
            LITHIUM_FAULT_PACK_TEMP_BACKUP |
            LITHIUM_FAULT_CHARGER |
            LITHIUM_FAULT_INPUT_POWER |
            LITHIUM_FAULT_INPUT_OVERVOLTAGE |
            LITHIUM_FAULT_INVALID_MEASUREMENT |
            LITHIUM_FAULT_PACK_VOLTAGE_MISMATCH |
            LITHIUM_FAULT_CHARGE_TIMEOUT;


    return
            (((ctx->recoverable_fault_flags |
               ctx->latched_fault_flags) &
              blocking_mask) != 0U);
}


/* ============================================================
 * ENABLE
 * ============================================================ */

static bool ChargerFSM_Enable(
        uint16_t current_ma,
        uint16_t vfb_mv)
{
    /*
     * Hardware /CE remains disabled while all registers are
     * programmed.
     */
    HAL_GPIO_WritePin(
            BQ_CE_GPIO_Port,
            BQ_CE_Pin,
            GPIO_PIN_SET);


    if (BQ25750_SetChargeVoltageFeedbackMv(
            vfb_mv) !=
        BQ_STATUS_OK)
    {
        return false;
    }


    if (BQ25750_SetChargeCurrentMa(
            current_ma) !=
        BQ_STATUS_OK)
    {
        return false;
    }


    if (BQ25750_SetSoftwareChargeEnable(
            true) !=
        BQ_STATUS_OK)
    {
        return false;
    }


    programmed_current_ma =
            current_ma;

    programmed_vfb_mv =
            vfb_mv;


    /*
     * Last permission.
     */
    HAL_GPIO_WritePin(
            BQ_CE_GPIO_Port,
            BQ_CE_Pin,
            GPIO_PIN_RESET);


    return true;
}


/* ============================================================
 * DISABLE
 * ============================================================ */

static void ChargerFSM_Disable(void)
{
    /*
     * Hardware permission removed first.
     */
    HAL_GPIO_WritePin(
            BQ_CE_GPIO_Port,
            BQ_CE_Pin,
            GPIO_PIN_SET);


    if ((programmed_current_ma != 0U) ||
        (programmed_vfb_mv != 0U))
    {
        (void)
        BQ25750_SetSoftwareChargeEnable(
                false);
    }


    programmed_current_ma =
            0U;

    programmed_vfb_mv =
            0U;

    charge_ramp_last_step_ms =
            0U;
}
