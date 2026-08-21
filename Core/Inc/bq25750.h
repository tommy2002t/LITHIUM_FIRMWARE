#ifndef BQ25750_H
#define BQ25750_H


#include <stdint.h>
#include <stdbool.h>

#include "lithium_types.h"


/* ============================================================
 * BQ25750 DATA
 * ============================================================ */

typedef struct
{
    int32_t input_current_ma;
    int32_t battery_current_ma;

    uint16_t vin_mv;
    uint16_t vbat_mv;
    uint16_t vsys_mv;

    int16_t pack_center_temperature_dC;

    bool pack_center_temperature_valid;

    bool power_good;
    bool watchdog_expired;

    uint8_t charge_state;
    uint8_t ts_state;

    uint8_t fault_status;

    bool input_undervoltage;
    bool input_overvoltage;

    bool battery_overcurrent;
    bool battery_overvoltage;

    bool thermal_shutdown;

    bool charge_timer_expired;
    bool cv_timer_expired;

    bool acfet_on;
    bool batfet_on;

} bq25750_data_t;


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

bq_status_t BQ25750_Init(void);


/* ============================================================
 * NON-BLOCKING PERIODIC DATA ACQUISITION
 *
 * StartReadData()
 *      Starts a complete measurement sequence.
 *
 * Service()
 *      Advances the sequence from main().
 *
 * DataReady()
 *      True when a complete coherent snapshot is available.
 *
 * GetData()
 *      Copies the latest completed snapshot.
 * ============================================================ */

bq_status_t BQ25750_StartReadData(void);

void BQ25750_Service(void);

bool BQ25750_DataReady(void);

bq_status_t BQ25750_GetData(
        bq25750_data_t *data);


/* ============================================================
 * HAL CALLBACK NOTIFICATION
 *
 * These functions ONLY raise internal driver flags.
 *
 * Call them from the appropriate HAL I2C callbacks.
 * ============================================================ */

void BQ25750_NotifyI2CTransferComplete(void);

void BQ25750_NotifyI2CTransferError(void);


/* ============================================================
 * DRIVER STATUS
 * ============================================================ */

bool BQ25750_IsBusy(void);


/* ============================================================
 * CHARGE CONTROL
 * ============================================================ */

bq_status_t BQ25750_SetSoftwareChargeEnable(
        bool enable);

bq_status_t BQ25750_SetChargeCurrentMa(
        uint16_t current_ma);

bq_status_t BQ25750_SetChargeVoltageFeedbackMv(
        uint16_t vfb_mv);


/* ============================================================
 * INPUT DYNAMIC POWER MANAGEMENT
 * ============================================================ */

bq_status_t BQ25750_SetInputCurrentLimitMa(
        uint16_t current_ma);

bq_status_t BQ25750_SetInputVoltageDpmMv(
        uint16_t voltage_mv);


/* ============================================================
 * POWER-PATH CONTROL
 * ============================================================ */

bq_status_t BQ25750_SetInputHighImpedance(
        bool enable);

bool BQ25750_IsInputHighImpedanceActive(void);

bq_status_t BQ25750_ForceBatteryFetOff(
        bool force_off);

bool BQ25750_IsBatteryFetForcedOff(void);


/* ============================================================
 * WATCHDOG
 * ============================================================ */

bq_status_t BQ25750_ServiceWatchdog(void);


#endif /* BQ25750_H */
