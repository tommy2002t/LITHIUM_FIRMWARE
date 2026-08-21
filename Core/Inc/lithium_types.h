#ifndef LITHIUM_TYPES_H
#define LITHIUM_TYPES_H

#include <stdint.h>
#include <stdbool.h>


/* ============================================================
 * SYSTEM FSM
 * ============================================================ */

typedef enum
{
    SYS_BOOT = 0,
    SYS_INIT,
    SYS_SELF_TEST,
    SYS_READY,
    SYS_FAULT_RECOVERABLE,
    SYS_FAULT_LATCHED

} system_state_t;


/* ============================================================
 * CHARGER FSM
 * ============================================================ */

typedef enum
{
    CHARGE_OFF = 0,
    CHARGE_CHECK,
    CHARGE_LOW_CELL_RECOVERY,
    CHARGE_ACTIVE,
    CHARGE_DONE,
    CHARGE_FAULT

} charger_state_t;


/* ============================================================
 * BALANCING FSM
 * ============================================================ */

typedef enum
{
    BALANCE_OFF = 0,
    BALANCE_ACTIVE

} balancing_state_t;


/* ============================================================
 * POWER SOURCE
 * ============================================================ */

typedef enum
{
    POWER_UNKNOWN = 0,
    POWER_INPUT,
    POWER_BATTERY

} power_source_t;


/* ============================================================
 * OPERATING MODE
 *
 * Ground / flight mode is derived from the actual power
 * configuration instead of from a CAN command.
 * ============================================================ */

typedef enum
{
    OPERATING_MODE_UNKNOWN = 0,
    OPERATING_MODE_GROUND,
    OPERATING_MODE_FLIGHT

} operating_mode_t;


/* ============================================================
 * BQ DRIVER STATUS
 * ============================================================ */

typedef enum
{
    BQ_STATUS_OK = 0,

    BQ_STATUS_NOT_INITIALIZED,

    BQ_STATUS_BUSY,

    BQ_STATUS_COMM_ERROR,
    BQ_STATUS_TIMEOUT,

    BQ_STATUS_CRC_ERROR,

    BQ_STATUS_CONFIG_ERROR,
    BQ_STATUS_VERIFY_ERROR,

    BQ_STATUS_INVALID_DATA

} bq_status_t;


/* ============================================================
 * FAULT SEVERITY
 * ============================================================ */

typedef enum
{
    FAULT_SEVERITY_NONE = 0,
    FAULT_SEVERITY_WARNING,
    FAULT_SEVERITY_RECOVERABLE,
    FAULT_SEVERITY_LATCHED

} fault_severity_t;


/* ============================================================
 * CHARGER DIAGNOSTIC FLAGS
 *
 * These flags preserve the precise reason why the Charger FSM
 * entered CHARGE_FAULT.  They are intentionally separate from
 * the global fault bitmap so the System FSM keeps ownership of
 * severity / recovery policy while CAN can still report the
 * exact underlying charger event.
 * ============================================================ */

#define LITHIUM_CHARGER_DIAG_NONE                     0x0000U
#define LITHIUM_CHARGER_DIAG_CONFIG_FAILURE           (1U << 0)
#define LITHIUM_CHARGER_DIAG_WATCHDOG_EXPIRED         (1U << 1)
#define LITHIUM_CHARGER_DIAG_DEVICE_FAULT             (1U << 2)
#define LITHIUM_CHARGER_DIAG_LOW_CELL_TIMEOUT         (1U << 3)
#define LITHIUM_CHARGER_DIAG_FAST_CHARGE_TIMEOUT      (1U << 4)
#define LITHIUM_CHARGER_DIAG_CV_TIMEOUT               (1U << 5)

/*
 * Live BQ25750 hardware-protection reasons.
 *
 * These preserve the exact source of REG0x24_Fault_Status
 * without changing the existing global system-fault policy.
 */
#define LITHIUM_CHARGER_DIAG_BQ_VAC_UV                (1U << 6)
#define LITHIUM_CHARGER_DIAG_BQ_VAC_OV                (1U << 7)
#define LITHIUM_CHARGER_DIAG_BQ_IBAT_OCP              (1U << 8)
#define LITHIUM_CHARGER_DIAG_BQ_VBAT_OV               (1U << 9)
#define LITHIUM_CHARGER_DIAG_BQ_TSHUT                 (1U << 10)
#define LITHIUM_CHARGER_DIAG_BQ_DRV_OKZ               (1U << 11)

#define LITHIUM_CHARGER_DIAG_RECOVERABLE_MASK         \
    (LITHIUM_CHARGER_DIAG_CONFIG_FAILURE |            \
     LITHIUM_CHARGER_DIAG_WATCHDOG_EXPIRED |          \
     LITHIUM_CHARGER_DIAG_DEVICE_FAULT |               \
     LITHIUM_CHARGER_DIAG_BQ_IBAT_OCP |                \
     LITHIUM_CHARGER_DIAG_BQ_DRV_OKZ)

#define LITHIUM_CHARGER_DIAG_LATCHED_TIMEOUT_MASK     \
    (LITHIUM_CHARGER_DIAG_LOW_CELL_TIMEOUT |          \
     LITHIUM_CHARGER_DIAG_FAST_CHARGE_TIMEOUT |       \
     LITHIUM_CHARGER_DIAG_CV_TIMEOUT)


/* ============================================================
 * FAULT FLAGS
 * ============================================================ */

#define LITHIUM_FAULT_NONE                        0x00000000UL


/* BQ25750 */

#define LITHIUM_FAULT_BQ25750_INIT                (1UL << 0)
#define LITHIUM_FAULT_BQ25750_COMM                (1UL << 1)


/* BQ76942 */

#define LITHIUM_FAULT_BQ76942_INIT                (1UL << 2)
#define LITHIUM_FAULT_BQ76942_COMM                (1UL << 3)


/* Cell voltage */

#define LITHIUM_FAULT_CELL_OVERVOLTAGE            (1UL << 4)
#define LITHIUM_FAULT_CELL_UNDERVOLTAGE           (1UL << 5)
#define LITHIUM_FAULT_CELL_IMBALANCE              (1UL << 6)


/* Temperature */

#define LITHIUM_FAULT_TEMP_HIGH                   (1UL << 7)
#define LITHIUM_FAULT_TEMP_LOW                    (1UL << 8)


/* Charger / input */

#define LITHIUM_FAULT_CHARGER                     (1UL << 9)
#define LITHIUM_FAULT_INPUT_POWER                 (1UL << 10)


/* Measurements */

#define LITHIUM_FAULT_INVALID_MEASUREMENT         (1UL << 11)


/* Internal firmware state */

#define LITHIUM_FAULT_INTERNAL_STATE              (1UL << 12)


/*
 * Dedicated thermistor plausibility failure.
 *
 * This is intentionally separate from TEMP_HIGH / TEMP_LOW.
 * A broken sensor is not the same event as a genuinely hot pack.
 */
#define LITHIUM_FAULT_TEMP_SENSOR                 (1UL << 13)


/*
 * Battery current reached the highest range which the
 * BQ25750 ADC can reliably report.
 */
#define LITHIUM_FAULT_BATTERY_OVERCURRENT         (1UL << 14)


/*
 * Deep cell undervoltage.
 *
 * Automatic recovery charging is not permitted.
 */
#define LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE      (1UL << 15)


/*
 * Charge safety / CV timeout.
 */
#define LITHIUM_FAULT_CHARGE_TIMEOUT              (1UL << 16)


/*
 * High input voltage requiring charging inhibition or
 * input isolation.
 */
#define LITHIUM_FAULT_INPUT_OVERVOLTAGE           (1UL << 17)


/*
 * The independent pack-center BQ25750 TS measurement is
 * unavailable or implausible.
 */
#define LITHIUM_FAULT_PACK_TEMP_BACKUP            (1UL << 18)


/*
 * BQ76942 cell-sum and BQ25750 VBAT disagree persistently.
 */
#define LITHIUM_FAULT_PACK_VOLTAGE_MISMATCH       (1UL << 19)


/*
 * CAN telemetry link unavailable.
 *
 * This is diagnostics-only: it must never inhibit charging,
 * balancing or the battery / VSYS power path.
 */
#define LITHIUM_FAULT_CAN                         (1UL << 20)


/* ============================================================
 * FAULT GROUP MASKS
 * ============================================================ */

#define LITHIUM_FAULT_BQ25750_MASK                 \
    (LITHIUM_FAULT_BQ25750_INIT |                  \
     LITHIUM_FAULT_BQ25750_COMM)

#define LITHIUM_FAULT_BQ76942_MASK                 \
    (LITHIUM_FAULT_BQ76942_INIT |                  \
     LITHIUM_FAULT_BQ76942_COMM)

#define LITHIUM_FAULT_BQ_INIT_MASK                 \
    (LITHIUM_FAULT_BQ25750_INIT |                  \
     LITHIUM_FAULT_BQ76942_INIT)

#define LITHIUM_FAULT_TEMPERATURE_MASK             \
    (LITHIUM_FAULT_TEMP_HIGH |                     \
     LITHIUM_FAULT_TEMP_LOW |                      \
     LITHIUM_FAULT_TEMP_SENSOR |                   \
     LITHIUM_FAULT_PACK_TEMP_BACKUP)

#define LITHIUM_FAULT_CELL_VOLTAGE_MASK            \
    (LITHIUM_FAULT_CELL_OVERVOLTAGE |              \
     LITHIUM_FAULT_CELL_UNDERVOLTAGE |             \
     LITHIUM_FAULT_CELL_DEEP_UNDERVOLTAGE |        \
     LITHIUM_FAULT_CELL_IMBALANCE)

#define LITHIUM_FAULT_POWER_PATH_MASK              \
    (LITHIUM_FAULT_INPUT_POWER |                   \
     LITHIUM_FAULT_INPUT_OVERVOLTAGE |             \
     LITHIUM_FAULT_BATTERY_OVERCURRENT)


/* ============================================================
 * LED PATTERNS
 * ============================================================ */

typedef enum
{
    LED_PATTERN_OFF = 0,

    LED_PATTERN_BOOT,
    LED_PATTERN_INIT,
    LED_PATTERN_SELF_TEST,
    LED_PATTERN_READY,
    LED_PATTERN_CHARGING,

    LED_PATTERN_WARNING,
    LED_PATTERN_FAULT_RECOVERABLE,

    LED_PATTERN_BQ25750_ERROR,
    LED_PATTERN_BQ76942_ERROR,
    LED_PATTERN_BOTH_BQ_ERROR,

    LED_PATTERN_FAULT_LATCHED

} led_pattern_t;


/* ============================================================
 * THERMAL SENSOR STATUS
 * ============================================================ */

typedef enum
{
    TEMP_SENSOR_STATUS_UNKNOWN = 0,
    TEMP_SENSOR_STATUS_VALID,
    TEMP_SENSOR_STATUS_SUSPECT,
    TEMP_SENSOR_STATUS_FAULT

} temperature_sensor_status_t;


/* ============================================================
 * BATTERY MEASUREMENTS
 *
 * Voltage     -> mV
 * Current     -> mA
 * Temperature -> 0.1 degC
 * ============================================================ */

typedef struct
{
    /*
     * BQ76942 cell-group measurements.
     *
     * index 0 -> series group 1
     * index 1 -> series group 2
     * index 2 -> series group 3
     */
    uint16_t cell_mv[3];

    uint16_t pack_mv;


    /*
     * TH1 / TH2 / TH3.
     *
     * One thermistor is physically located in the center
     * of each pair of parallel cells.
     */
    int16_t temperature_dC[3];


    /*
     * Fourth 103AT-2 thermistor.
     *
     * Connected to the BQ25750 TS input and physically located
     * near the center of the complete 3S2P battery pack.
     */
    int16_t pack_center_temperature_dC;


    /*
     * Sensor plausibility state after cross-checking.
     */
    temperature_sensor_status_t temperature_status[3];
    temperature_sensor_status_t pack_center_temperature_status;


    /*
     * BQ25750 voltage / current measurements.
     */
    uint16_t vin_mv;
    uint16_t vsys_mv;
    uint16_t charger_vbat_mv;

    int32_t battery_current_ma;
    int32_t input_current_ma;


    /*
     * Complete driver snapshot validity.
     */
    bool bq25750_valid;
    bool bq76942_valid;


    /*
     * BQ25750 charger / power status.
     */
    bool charger_power_good;
    bool charger_watchdog_expired;

    uint8_t charger_charge_state;
    uint8_t charger_fault_status;


    /*
     * Independent BQ76942 comparator/protection status.
     *
     * These are not inferred from the ADC cell readings.
     * They come from the BQ76942 Safety Status A register and
     * therefore act as a second voltage-safety observation path.
     */
    bool bq76942_cuv_active;
    bool bq76942_cov_active;


    /*
     * True when the pack-center BQ25750 TS reading is valid.
     */
    bool pack_center_temperature_valid;


    /*
     * True only when all measurements required for the current
     * high-level safety decision are fresh and usable.
     */
    bool valid;

} lithium_measurements_t;


/* ============================================================
 * MAIN LITHIUM CONTEXT
 * ============================================================ */

typedef struct
{
    /* FSM states */

    system_state_t system_state;
    charger_state_t charger_state;
    balancing_state_t balancing_state;


    /* Power configuration */

    power_source_t power_source;
    operating_mode_t operating_mode;

    bool input_present;


    /*
     * True when the battery has intentionally been isolated
     * from VSYS while a valid ground input continues to power
     * the rocket.
     */
    bool battery_isolated;


    /*
     * True when the input path has intentionally been requested
     * to enter HIZ / isolation.
     */
    bool input_isolated;


    /* IC availability */

    bool bq25750_initialized;
    bool bq76942_initialized;


    /* Latest driver status */

    bq_status_t bq25750_status;
    bq_status_t bq76942_status;


    /* Initial SYS_INIT attempts */

    uint8_t bq25750_init_attempts;
    uint8_t bq76942_init_attempts;


    /* SYS_FAULT_RECOVERABLE retry rounds */

    uint8_t recovery_attempts;


    /* --------------------------------------------------------
     * Fault information
     * -------------------------------------------------------- */

    uint32_t fault_flags;

    uint32_t warning_fault_flags;
    uint32_t recoverable_fault_flags;
    uint32_t latched_fault_flags;


    /* Highest-priority currently displayed fault */

    uint32_t primary_fault;
    fault_severity_t primary_fault_severity;


    /*
     * Exact charger-fault reason(s).
     *
     * The Charger FSM sets diagnostic bits; lithium_app.c maps
     * them to WARNING / RECOVERABLE / LATCHED system faults.
     */
    uint16_t charger_diagnostic_flags;


    /* Latest measurements */

    lithium_measurements_t measurements;


    /*
     * Balancing mask used by CAN / diagnostics.
     */
    uint16_t active_balance_mask;


    /*
     * SOC estimate.
     *
     * 0...10000 corresponds to 0.00...100.00%.
     */
    uint16_t soc_centi_percent;


    /*
     * Integrated usable charge estimate.
     */
    int32_t remaining_charge_mah;


    /* Timing */

    uint32_t state_enter_time_ms;

    uint32_t last_init_attempt_ms;
    uint32_t last_recovery_attempt_ms;

    uint32_t last_measurement_time_ms;

} lithium_context_t;


/* ============================================================
 * INTERRUPT EVENT FLAGS
 *
 * Interrupt callbacks SET flags only.
 *
 * No SPI/I2C transactions and no FSM transitions are performed
 * directly inside the hardware callbacks.
 * ============================================================ */

typedef struct
{
    volatile bool bq25750_interrupt;

    volatile bool bq76942_alert;

    volatile bool power_good_changed;

    volatile bool i2c_transfer_complete;
    volatile bool i2c_transfer_error;

    volatile bool spi_transfer_complete;
    volatile bool spi_transfer_error;

} lithium_event_flags_t;


#endif /* LITHIUM_TYPES_H */
