#ifndef BQ76942_H
#define BQ76942_H


#include <stdint.h>
#include <stdbool.h>

#include "lithium_types.h"


/* ============================================================
 * PHYSICAL CELL CONNECTIONS
 *
 * LITHIUM 3S topology:
 *
 * Series group 1 -> VC1
 * Series group 2 -> VC2
 * Series group 3 -> VC10
 * ============================================================ */

#define BQ76942_BALANCE_GROUP1_MASK              (1U << 0)
#define BQ76942_BALANCE_GROUP2_MASK              (1U << 1)
#define BQ76942_BALANCE_GROUP3_MASK              (1U << 9)


/* ============================================================
 * BQ76942 DATA
 * ============================================================ */

typedef struct
{
    uint16_t cell_mv[3];

    uint16_t pack_mv;


    /*
     * TS1 / TS2 / TS3
     *
     * 0.1 degC
     */
    int16_t temperature_dC[3];


    /*
     * Internal BQ76942 die temperature.
     *
     * 0.1 degC
     */
    int16_t internal_temperature_dC;


    /*
     * Raw safety status.
     */
    uint8_t safety_status_a;
    uint8_t safety_status_b;
    uint8_t safety_status_c;


    uint16_t alarm_status;


    /*
     * Explicitly decoded protection information.
     */
    bool cell_undervoltage;
    bool cell_overvoltage;

    bool charge_overtemperature;
    bool charge_undertemperature;

    bool discharge_overtemperature;

    bool internal_overtemperature;


    /*
     * Current host-controlled balance state.
     */
    uint16_t active_balance_mask;

} bq76942_data_t;


/* ============================================================
 * INITIALIZATION
 *
 * Initialization may contain the strictly required BQ startup /
 * CFGUPDATE timing.
 *
 * Runtime monitoring itself is non-blocking.
 * ============================================================ */

bq_status_t BQ76942_Init(void);


/* ============================================================
 * NON-BLOCKING PERIODIC DATA ACQUISITION
 * ============================================================ */

bq_status_t BQ76942_StartReadData(void);

void BQ76942_Service(void);

bool BQ76942_DataReady(void);

bq_status_t BQ76942_GetData(
        bq76942_data_t *data);

bool BQ76942_IsBusy(void);


/* ============================================================
 * SPI CALLBACK NOTIFICATION
 *
 * Callback functions only raise flags.
 * ============================================================ */

void BQ76942_NotifySpiTransferComplete(void);

void BQ76942_NotifySpiTransferError(void);


/* ============================================================
 * HOST-CONTROLLED CELL BALANCING
 *
 * Valid masks:
 *
 * 0
 * BQ76942_BALANCE_GROUP1_MASK
 * BQ76942_BALANCE_GROUP2_MASK
 * BQ76942_BALANCE_GROUP3_MASK
 *
 * Only one group at a time is accepted.
 * ============================================================ */

bq_status_t BQ76942_SetBalancingMask(
        uint16_t cell_mask);


/* ============================================================
 * ALARM CONTROL
 * ============================================================ */

bq_status_t BQ76942_ClearAlarmStatus(
        uint16_t alarm_mask);


#endif /* BQ76942_H */
