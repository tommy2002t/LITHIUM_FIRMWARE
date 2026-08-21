#include "balancing_fsm.h"

#include "main.h"

#include "bq76942.h"
#include "lithium_config.h"


/* ============================================================
 * LOCAL STATE
 * ============================================================ */

static uint16_t active_balance_mask = 0U;

static uint32_t last_balance_command_ms = 0U;


/* ============================================================
 * PRIVATE FUNCTIONS
 * ============================================================ */

static bool BalancingFSM_CanBalance(
        const lithium_context_t *ctx);

static uint16_t BalancingFSM_SelectHighestCellMask(
        const lithium_context_t *ctx);

static uint16_t BalancingFSM_GetMinCellMv(
        const lithium_context_t *ctx);

static uint16_t BalancingFSM_GetMaxCellMv(
        const lithium_context_t *ctx);

static int16_t BalancingFSM_GetMinTrustedTempdC(
        const lithium_context_t *ctx);

static int16_t BalancingFSM_GetMaxTrustedTempdC(
        const lithium_context_t *ctx);

static uint8_t BalancingFSM_CountValidTemperatureSources(
        const lithium_context_t *ctx);


/* ============================================================
 * INIT
 * ============================================================ */

void BalancingFSM_Init(
        lithium_context_t *ctx)
{
    active_balance_mask =
            0U;


    last_balance_command_ms =
            HAL_GetTick();


    if (ctx != NULL)
    {
        ctx->active_balance_mask =
                0U;

        ctx->balancing_state =
                BALANCE_OFF;
    }
}


/* ============================================================
 * RUN
 * ============================================================ */

void BalancingFSM_Run(
        lithium_context_t *ctx)
{
    uint16_t min_cell_mv;
    uint16_t max_cell_mv;

    uint16_t delta_mv;

    uint16_t requested_mask;

    uint32_t now;


    if (ctx == NULL)
    {
        return;
    }


    if (!BalancingFSM_CanBalance(
            ctx))
    {
        BalancingFSM_ForceOff(
                ctx);

        return;
    }


    min_cell_mv =
            BalancingFSM_GetMinCellMv(
                    ctx);


    max_cell_mv =
            BalancingFSM_GetMaxCellMv(
                    ctx);


    delta_mv =
            (uint16_t)(
                    max_cell_mv -
                    min_cell_mv);


    now =
            HAL_GetTick();


    switch (ctx->balancing_state)
    {
        /* ----------------------------------------------------
         * OFF
         * ---------------------------------------------------- */

        case BALANCE_OFF:

            if ((min_cell_mv >=
                 LITHIUM_BALANCE_MIN_CELL_MV) &&
                (delta_mv >=
                 LITHIUM_BALANCE_START_DELTA_MV))
            {
                requested_mask =
                        BalancingFSM_SelectHighestCellMask(
                                ctx);


                if (BQ76942_SetBalancingMask(
                        requested_mask) ==
                    BQ_STATUS_OK)
                {
                    active_balance_mask =
                            requested_mask;

                    ctx->active_balance_mask =
                            requested_mask;


                    last_balance_command_ms =
                            now;


                    ctx->balancing_state =
                            BALANCE_ACTIVE;
                }
            }

            break;


        /* ----------------------------------------------------
         * ACTIVE
         * ---------------------------------------------------- */

        case BALANCE_ACTIVE:

            if ((min_cell_mv <
                 LITHIUM_BALANCE_MIN_CELL_MV) ||
                (delta_mv <=
                 LITHIUM_BALANCE_STOP_DELTA_MV))
            {
                BalancingFSM_ForceOff(
                        ctx);

                break;
            }


            /*
             * Always balance the currently highest group.
             */
            requested_mask =
                    BalancingFSM_SelectHighestCellMask(
                            ctx);


            /*
             * Refresh before BQ's 20-second host-controlled
             * balancing interval expires.
             */
            if ((requested_mask !=
                 active_balance_mask) ||
                ((now -
                  last_balance_command_ms) >=
                 LITHIUM_BALANCE_REFRESH_PERIOD_MS))
            {
                if (BQ76942_SetBalancingMask(
                        requested_mask) ==
                    BQ_STATUS_OK)
                {
                    active_balance_mask =
                            requested_mask;

                    ctx->active_balance_mask =
                            requested_mask;


                    last_balance_command_ms =
                            now;
                }
                else
                {
                    BalancingFSM_ForceOff(
                            ctx);
                }
            }

            break;


        default:

            BalancingFSM_ForceOff(
                    ctx);

            break;
    }
}


/* ============================================================
 * FORCE OFF
 * ============================================================ */

void BalancingFSM_ForceOff(
        lithium_context_t *ctx)
{
    if ((ctx != NULL) &&
        ctx->bq76942_initialized &&
        ((active_balance_mask != 0U) ||
         (ctx->balancing_state ==
          BALANCE_ACTIVE)))
    {
        (void)
        BQ76942_SetBalancingMask(
                0U);
    }


    active_balance_mask =
            0U;


    last_balance_command_ms =
            HAL_GetTick();


    if (ctx != NULL)
    {
        ctx->active_balance_mask =
                0U;

        ctx->balancing_state =
                BALANCE_OFF;
    }
}


/* ============================================================
 * CAN BALANCE
 * ============================================================ */

static bool BalancingFSM_CanBalance(
        const lithium_context_t *ctx)
{
    int16_t min_temp_dC;
    int16_t max_temp_dC;


    if (ctx == NULL)
    {
        return false;
    }


    if (ctx->system_state !=
        SYS_READY)
    {
        return false;
    }


    /*
     * Balancing is intentionally restricted to ground /
     * umbilical operation.
     *
     * We do not intentionally waste battery energy balancing
     * during flight.
     */
    if ((!ctx->input_present) ||
        (ctx->operating_mode !=
         OPERATING_MODE_GROUND))
    {
        return false;
    }


    if ((!ctx->bq76942_initialized) ||
        (!ctx->measurements.bq76942_valid))
    {
        return false;
    }


    /*
     * These faults always prohibit balancing.
     */
    if (((ctx->recoverable_fault_flags |
          ctx->latched_fault_flags) &
         (LITHIUM_FAULT_BQ76942_MASK |
          LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE |
          LITHIUM_FAULT_TEMP_SENSOR |
          LITHIUM_FAULT_TEMP_HIGH |
          LITHIUM_FAULT_TEMP_LOW |
          LITHIUM_FAULT_INVALID_MEASUREMENT)) != 0U)
    {
        return false;
    }


    /*
     * IMPORTANT:
     *
     * CELL_OVERVOLTAGE is intentionally NOT included above.
     *
     * If one cell reaches the software charge-inhibit region,
     * charging stops but balancing of that high cell is still
     * useful and allowed as long as it has not reached a severe
     * unsafe state.
     */


    /*
     * Match the charger thermal-confidence policy: balancing is
     * allowed only when at least three independent temperature
     * sources are explicitly VALID.  SUSPECT sensors are not
     * counted as trusted inputs.
     */
    if (BalancingFSM_CountValidTemperatureSources(
            ctx) < 3U)
    {
        return false;
    }


    min_temp_dC =
            BalancingFSM_GetMinTrustedTempdC(
                    ctx);


    max_temp_dC =
            BalancingFSM_GetMaxTrustedTempdC(
                    ctx);


    if (min_temp_dC <
        LITHIUM_BALANCE_EXT_TEMP_MIN_dC)
    {
        return false;
    }


    if (max_temp_dC >
        LITHIUM_BALANCE_EXT_TEMP_MAX_dC)
    {
        return false;
    }


    return true;
}


/* ============================================================
 * SELECT HIGHEST CELL
 * ============================================================ */

static uint16_t BalancingFSM_SelectHighestCellMask(
        const lithium_context_t *ctx)
{
    uint8_t index = 0U;


    if (ctx->measurements.cell_mv[1] >
        ctx->measurements.cell_mv[index])
    {
        index =
                1U;
    }


    if (ctx->measurements.cell_mv[2] >
        ctx->measurements.cell_mv[index])
    {
        index =
                2U;
    }


    if (index == 0U)
    {
        return
                BQ76942_BALANCE_GROUP1_MASK;
    }


    if (index == 1U)
    {
        return
                BQ76942_BALANCE_GROUP2_MASK;
    }


    return
            BQ76942_BALANCE_GROUP3_MASK;
}


/* ============================================================
 * CELL HELPERS
 * ============================================================ */

static uint16_t BalancingFSM_GetMinCellMv(
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


static uint16_t BalancingFSM_GetMaxCellMv(
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
 * TEMPERATURE HELPERS
 * ============================================================ */

static int16_t BalancingFSM_GetMinTrustedTempdC(
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


static int16_t BalancingFSM_GetMaxTrustedTempdC(
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
 * VALID TEMPERATURE SOURCE COUNT
 * ============================================================ */

static uint8_t BalancingFSM_CountValidTemperatureSources(
        const lithium_context_t *ctx)
{
    uint8_t count = 0U;
    uint8_t i;


    if (ctx == NULL)
    {
        return 0U;
    }


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
