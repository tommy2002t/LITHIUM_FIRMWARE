#ifndef LITHIUM_CONFIG_H
#define LITHIUM_CONFIG_H


/* ============================================================
 * MAIN APPLICATION TIMING
 * ============================================================ */

#define LITHIUM_SYSTEM_PERIOD_MS                         100U

#define LITHIUM_FAST_POWER_SAFETY_PERIOD_MS              10U


/* ============================================================
 * INITIALIZATION / RECOVERY
 * ============================================================ */

#define LITHIUM_INIT_MAX_ATTEMPTS                        3U
#define LITHIUM_INIT_RETRY_DELAY_MS                      250U

#define LITHIUM_RECOVERY_MAX_ATTEMPTS                    3U
#define LITHIUM_RECOVERY_RETRY_PERIOD_MS                 2000U

#define LITHIUM_SELF_TEST_SETTLING_TIME_MS               0U
#define LITHIUM_SELF_TEST_VALID_SET_COUNT                3U


/* ============================================================
 * COMMUNICATION HEALTH
 * ============================================================ */

#define LITHIUM_COMM_FAILURE_CONFIRM_COUNT               3U

#define LITHIUM_BQ_TRANSACTION_TIMEOUT_MS                5U
#define LITHIUM_BQ_TRANSACTION_RETRIES                   3U


/* ============================================================
 * MEASUREMENT MONITORING
 * ============================================================ */

#define LITHIUM_MONITOR_CONFIRM_COUNT                    3U

#define LITHIUM_DATA_STALE_WARNING_MS                    300U
#define LITHIUM_DATA_STALE_FAULT_MS                      1000U

#define LITHIUM_PACK_VOLTAGE_TOLERANCE_MV                200U
#define LITHIUM_PACK_VOLTAGE_TARGET_TOLERANCE_MV         100U

/*
 * Number of consecutive BQ76942-cell-sum versus
 * BQ25750-VBAT disagreements required before declaring
 * a pack-voltage cross-check fault.
 */
#define LITHIUM_PACK_XCHECK_COUNT                        3U


/* ============================================================
 * PACK / CELL CONFIGURATION
 * ============================================================ */

#define LITHIUM_SERIES_GROUP_COUNT                       3U
#define LITHIUM_PARALLEL_CELL_COUNT                      2U

#define LITHIUM_PACK_NOMINAL_CAPACITY_MAH                6240U
#define LITHIUM_PACK_MINIMUM_CAPACITY_MAH                6000U


/* ============================================================
 * NORMAL CHARGE VOLTAGE
 * ============================================================ */

#define LITHIUM_CHARGE_VFB_NORMAL_MV                     1536U
#define LITHIUM_PACK_CHARGE_NORMAL_MV                    12401U
#define LITHIUM_CELL_CHARGE_NORMAL_MV                    4134U

#define LITHIUM_CHARGE_VFB_WARM_MV                       1504U
#define LITHIUM_PACK_CHARGE_WARM_MV                      12143U
#define LITHIUM_CELL_CHARGE_WARM_MV                      4048U


/* ============================================================
 * INDIVIDUAL CELL OVERVOLTAGE POLICY
 * ============================================================ */

#define LITHIUM_CELL_CHARGE_INHIBIT_MV                   4170U
#define LITHIUM_CELL_CHARGE_RECOVER_MV                   4150U

#define LITHIUM_BQ76942_COV_TARGET_MV                    4200U
#define LITHIUM_BQ76942_COV_DELAY_MS                     500U
#define LITHIUM_BQ76942_COV_RECOVERY_HYST_MV             100U

#define LITHIUM_CELL_OV_LATCH_MV                         4250U


/* ============================================================
 * CELL UNDERVOLTAGE POLICY
 * ============================================================ */

#define LITHIUM_CELL_UV_WARNING_MV                       3000U
#define LITHIUM_CELL_UV_WARNING_RECOVER_MV               3100U

#define LITHIUM_CELL_UV_LATCH_MV                         2500U

#define LITHIUM_BQ76942_CUV_TARGET_MV                    2530U
#define LITHIUM_BQ76942_CUV_DELAY_MS                     2000U
#define LITHIUM_BQ76942_CUV_RECOVERY_HYST_MV             100U


/* ============================================================
 * LOW-CELL RECOVERY CHARGING
 * ============================================================ */

#define LITHIUM_LOW_CELL_RECOVERY_CURRENT_MA             1000U
#define LITHIUM_LOW_CELL_RECOVERY_TIMEOUT_MS             1800000UL


/* ============================================================
 * NORMAL CHARGE CURRENT
 * ============================================================ */

#define LITHIUM_CHARGE_CURRENT_NORMAL_MA                 5000U
#define LITHIUM_CHARGE_CURRENT_HW_LIMIT_MA               5000U


/* ============================================================
 * CHARGE STARTUP RAMP
 *
 * Normal charge permission and all existing safety checks remain
 * unchanged.  This only ramps an allowed charge-current request
 * upward after charge start.
 * ============================================================ */

#define LITHIUM_CHARGE_STARTUP_RAMP_ENABLE                1U
#define LITHIUM_CHARGE_STARTUP_RAMP_INITIAL_MA            1000U
#define LITHIUM_CHARGE_STARTUP_RAMP_STEP_MA               1000U
#define LITHIUM_CHARGE_STARTUP_RAMP_STEP_MS               1000UL


/* ============================================================
 * THERMAL CHARGE DERATING
 *
 * Temperatures are stored as 0.1 degC.
 * ============================================================ */

#define LITHIUM_CHARGE_TEMP_MIN_dC                       0
#define LITHIUM_CHARGE_TEMP_MAX_dC                       550

#define LITHIUM_TEMP_COLD_RECOVER_dC                     50
#define LITHIUM_TEMP_HOT_RECOVER_dC                      500

#define LITHIUM_CHARGE_TEMP_COLD_DERATE_END_dC           100
#define LITHIUM_CHARGE_TEMP_FULL_END_dC                  400
#define LITHIUM_CHARGE_TEMP_WARM1_END_dC                 450
#define LITHIUM_CHARGE_TEMP_WARM2_END_dC                 500

#define LITHIUM_CHARGE_CURRENT_COLD_MA                   2000U
#define LITHIUM_CHARGE_CURRENT_WARM1_MA                  3750U
#define LITHIUM_CHARGE_CURRENT_WARM2_MA                  2500U
#define LITHIUM_CHARGE_CURRENT_HOT_MA                    1500U

/*
 * Conservative current cap when exactly one of TH1/TH2/TH3
 * is confirmed faulty but the other two group sensors and
 * the independent pack-center sensor remain trustworthy.
 */
#define LITHIUM_CHARGE_CURRENT_SENSOR_DEGRADED_MA        3750U

#define LITHIUM_TEMP_SEVERE_HIGH_dC                      600
#define LITHIUM_TEMP_SEVERE_LOW_dC                       (-100)


/* ============================================================
 * FOUR-THERMISTOR VALIDATION POLICY
 * ============================================================ */

#define LITHIUM_TEMP_SENSOR_PLAUSIBLE_MIN_dC             (-400)
#define LITHIUM_TEMP_SENSOR_PLAUSIBLE_MAX_dC             900

#define LITHIUM_TEMP_SENSOR_DISAGREEMENT_dC              150
#define LITHIUM_TEMP_SENSOR_HARD_DISAGREEMENT_dC         250

#define LITHIUM_SINGLE_TEMP_EVENT_CONFIRM_COUNT          10U
#define LITHIUM_CROSS_CONFIRMED_TEMP_COUNT               2U

#define LITHIUM_TEMP_SENSOR_FAULT_CONFIRM_COUNT          20U

#define LITHIUM_TEMP_NOISE_FILTER_COUNT                  3U

#define LITHIUM_TEMP_MAX_SINGLE_STEP_dC                  50


/* ============================================================
 * BQ25750 INDEPENDENT TS BACKUP
 * ============================================================ */

#define LITHIUM_BQ25750_TS_COLD_C                        0
#define LITHIUM_BQ25750_TS_HOT_C                         60

#define LITHIUM_BQ25750_TS_ENABLE                        1U
#define LITHIUM_BQ25750_JEITA_ENABLE                     0U


/* ============================================================
 * BALANCING POLICY
 * ============================================================ */

#define LITHIUM_BALANCE_MIN_CELL_MV                      3900U
#define LITHIUM_BALANCE_START_DELTA_MV                   40U
#define LITHIUM_BALANCE_STOP_DELTA_MV                    20U

#define LITHIUM_BALANCE_MAX_SIMULTANEOUS_CELLS           1U

#define LITHIUM_BALANCE_EXT_TEMP_MIN_dC                  0
#define LITHIUM_BALANCE_EXT_TEMP_MAX_dC                  450

#define LITHIUM_BALANCE_INTERNAL_TEMP_MAX_dC             700

#define LITHIUM_BALANCE_IC_INTERVAL_S                    20U
#define LITHIUM_BALANCE_REFRESH_PERIOD_MS                10000UL


/* ============================================================
 * RECHARGE POLICY
 * ============================================================ */

#define LITHIUM_CELL_RECHARGE_MV                         4030U


/* ============================================================
 * 24-V UMBILICAL INPUT POLICY
 * ============================================================ */

#define LITHIUM_INPUT_NOMINAL_MV                         24000U

#define LITHIUM_INPUT_LOW_WARNING_MV                     21000U

#define LITHIUM_INPUT_VAC_DPM_MV                         20000U

#define LITHIUM_INPUT_VIN_MIN_MV                         18000U

#define LITHIUM_INPUT_HIGH_WARNING_MV                    27000U

#define LITHIUM_INPUT_CHARGE_MAX_MV                      28000U

#define LITHIUM_INPUT_HIZ_REQUEST_MV                     30000U

#define LITHIUM_INPUT_VIN_MAX_MV                         30000U

#define LITHIUM_INPUT_CURRENT_LIMIT_MA                   8000U


/* ============================================================
 * CHARGE TERMINATION
 * ============================================================ */

#define LITHIUM_TERMINATION_CURRENT_REGISTER_MA          300U

#define LITHIUM_ENABLE_CHARGE_TERMINATION                1U
#define LITHIUM_ENABLE_TOP_OFF_TIMER                     0U
#define LITHIUM_ENABLE_PFM_DURING_CHARGE                 0U


/* ============================================================
 * CHARGE SAFETY TIMERS
 * ============================================================ */

#define LITHIUM_CHARGE_SAFETY_TIMER_HOURS                8U

#define LITHIUM_CHARGE_TIMER_2X_ENABLE                   1U

#define LITHIUM_CV_TIMER_HOURS                           2U


/* ============================================================
 * WATCHDOG POLICY
 * ============================================================ */

#define LITHIUM_BQ25750_WATCHDOG_TIMEOUT_MS              40000UL
#define LITHIUM_BQ25750_WATCHDOG_SERVICE_MS              10000UL

#define LITHIUM_STM32_IWDG_TARGET_MS                     1000U


/* ============================================================
 * DISCHARGE OVERCURRENT / SHORT-CIRCUIT POLICY
 * ============================================================ */

/*
 * External BIFROST fuse: Bourns SF-1206HHA800R-2.
 *
 * 8 A is the continuous rated current of the fuse and is NOT a
 * firmware fault threshold.  The selected fuse is intentionally
 * used as the physical high-current / short-circuit protection;
 * currents in the 8...10 A region are not automatically treated
 * as faults by the firmware.
 *
 * Around the ~30-A class the fuse is already deep into its fast
 * clearing region according to its time-current characteristic.
 */
#define LITHIUM_EXTERNAL_FUSE_RATED_MA                   8000UL
#define LITHIUM_EXTERNAL_FUSE_FAST_CLEAR_REFERENCE_MA    30000UL

/*
 * Keep the existing ~19.5-A software threshold as an extreme
 * current / BQ25750 ADC-edge diagnostic, not as the fuse rating.
 */
#define LITHIUM_SW_OVERCURRENT_ADC_LIMIT_MA              19500L

#define LITHIUM_SW_OVERCURRENT_CONFIRM_COUNT             1U


/* ============================================================
 * GROUND / FLIGHT POWER-PATH POLICY
 * ============================================================ */

#define LITHIUM_GROUND_ALLOW_BATTERY_ISOLATION           1U
#define LITHIUM_FLIGHT_ALLOW_GENERIC_POWER_CUTOFF        0U


/* ============================================================
 * CAN
 * ============================================================ */

#define LITHIUM_CAN_ENABLE                               1U
#define LITHIUM_CAN_HAS_CHARGE_AUTHORITY                 0U

/*
 * Compile-time LITHIUM node identity.
 *
 * Each physical LITHIUM board must use a unique value.
 * Valid values for the current protocol allocation are 0...2.
 */
#define LITHIUM_CAN_NODE_ID                              0U

/*
 * Standard 11-bit identifier allocation.
 *
 * Each LITHIUM node owns a 16-ID block:
 *
 *   node 0 -> 0x500 ... 0x50F
 *   node 1 -> 0x510 ... 0x51F
 *   node 2 -> 0x520 ... 0x52F
 *
 * Before integration onto a shared rocket bus, this base range
 * must be checked against the final system-wide CAN ID map.
 */
#define LITHIUM_CAN_BASE_ID                              0x500U
#define LITHIUM_CAN_NODE_ID_STRIDE                       0x10U

/*
 * Periodic telemetry timing.
 */
#define LITHIUM_CAN_STATUS_PERIOD_MS                     100U
#define LITHIUM_CAN_ELECTRICAL_PERIOD_MS                 200U
#define LITHIUM_CAN_THERMAL_PERIOD_MS                    500U
#define LITHIUM_CAN_FAULT_PERIOD_MS                      500U

/*
 * If FDCAN fails to start, retry locally without affecting
 * charging, balancing or the battery power path.
 */
#define LITHIUM_CAN_RESTART_PERIOD_MS                    1000U


/* ============================================================
 * SOC
 * ============================================================ */

#define LITHIUM_SOC_INITIAL_CAPACITY_MAH                 \
        LITHIUM_PACK_MINIMUM_CAPACITY_MAH


#endif /* LITHIUM_CONFIG_H */
