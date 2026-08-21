#ifndef LITHIUM_CAN_H
#define LITHIUM_CAN_H

#include <stdint.h>


/* ============================================================
 * LITHIUM CAN PROTOCOL
 *
 * Transport:
 * - Classic CAN
 * - Standard 11-bit identifiers
 * - 8 data bytes per frame
 * - 500 kbit/s with the current STM32 FDCAN timing
 *
 * Each LITHIUM board owns one block of 16 CAN identifiers.
 * The actual node block is selected through
 * LITHIUM_CAN_NODE_ID in lithium_config.h.
 * ============================================================ */

#define LITHIUM_CAN_PROTOCOL_VERSION                    1U


/* ============================================================
 * FRAME OFFSETS INSIDE EACH NODE BLOCK
 * ============================================================ */

#define LITHIUM_CAN_OFFSET_STATUS                       0x0U
#define LITHIUM_CAN_OFFSET_CELL_VOLTAGES                0x1U
#define LITHIUM_CAN_OFFSET_TEMPERATURES                 0x2U
#define LITHIUM_CAN_OFFSET_POWER_VOLTAGES               0x3U
#define LITHIUM_CAN_OFFSET_CURRENT_SOC                  0x4U
#define LITHIUM_CAN_OFFSET_FAULT_SUMMARY                0x5U
#define LITHIUM_CAN_OFFSET_FAULT_MASKS_A                0x6U
#define LITHIUM_CAN_OFFSET_FAULT_MASKS_B                0x7U
#define LITHIUM_CAN_OFFSET_DIAGNOSTIC_EVENT             0x8U
#define LITHIUM_CAN_OFFSET_DIAGNOSTIC_TEXT              0x9U


/* ============================================================
 * DIAGNOSTIC EVENT STATE
 * ============================================================ */

typedef enum
{
    LITHIUM_CAN_DIAG_EVENT_CLEARED = 0U,
    LITHIUM_CAN_DIAG_EVENT_ACTIVE  = 1U

} lithium_can_diag_event_state_t;


/* ============================================================
 * PERIODIC FRAME PAYLOADS
 *
 * All multi-byte values are little-endian.
 * ============================================================ */

/*
 * STATUS frame - offset 0x0
 *
 * Byte 0 : protocol version
 * Byte 1 : system_state_t
 * Byte 2 : charger_state_t
 * Byte 3 : balancing_state_t
 * Byte 4 : power_source_t
 * Byte 5 : operating_mode_t
 * Byte 6 : primary fault severity
 * Byte 7 : status flags
 *
 * Status flags:
 * bit 0 : input present
 * bit 1 : battery isolated
 * bit 2 : input isolated / HIZ
 * bit 3 : BQ25750 initialized
 * bit 4 : BQ76942 initialized
 * bit 5 : complete measurements valid
 * bit 6 : charger power-good
 * bit 7 : reserved
 */

#define LITHIUM_CAN_STATUS_INPUT_PRESENT                (1U << 0)
#define LITHIUM_CAN_STATUS_BATTERY_ISOLATED             (1U << 1)
#define LITHIUM_CAN_STATUS_INPUT_ISOLATED               (1U << 2)
#define LITHIUM_CAN_STATUS_BQ25750_INITIALIZED           (1U << 3)
#define LITHIUM_CAN_STATUS_BQ76942_INITIALIZED           (1U << 4)
#define LITHIUM_CAN_STATUS_MEASUREMENTS_VALID            (1U << 5)
#define LITHIUM_CAN_STATUS_CHARGER_POWER_GOOD            (1U << 6)


/*
 * CELL VOLTAGES frame - offset 0x1
 *
 * Bytes 0..1 : group 1 voltage [mV]
 * Bytes 2..3 : group 2 voltage [mV]
 * Bytes 4..5 : group 3 voltage [mV]
 * Bytes 6..7 : BQ76942 pack voltage [mV]
 */


/*
 * TEMPERATURES frame - offset 0x2
 *
 * Signed int16 values in 0.1 degC.
 *
 * Bytes 0..1 : TH1
 * Bytes 2..3 : TH2
 * Bytes 4..5 : TH3
 * Bytes 6..7 : pack-center 103AT-2
 */


/*
 * POWER VOLTAGES frame - offset 0x3
 *
 * Bytes 0..1 : VIN [mV]
 * Bytes 2..3 : VSYS [mV]
 * Bytes 4..5 : BQ25750 VBAT [mV]
 * Bytes 6..7 : reserved
 */


/*
 * CURRENT / SOC frame - offset 0x4
 *
 * Bytes 0..1 : battery current [mA], signed int16
 * Bytes 2..3 : input current [mA], signed int16
 * Bytes 4..5 : SOC [0.01%]
 *              0xFFFF = SOC estimate not implemented / invalid
 * Bytes 6..7 : active balancing mask
 */


/*
 * FAULT SUMMARY frame - offset 0x5
 *
 * Bytes 0..3 : all active fault flags
 * Byte 4     : primary fault bit index, 0xFF if none
 * Byte 5     : primary fault severity
 * Bytes 6..7 : charger diagnostic flags
 */


/*
 * FAULT MASKS A frame - offset 0x6
 *
 * Bytes 0..3 : warning fault flags
 * Bytes 4..7 : recoverable fault flags
 */


/*
 * FAULT MASKS B frame - offset 0x7
 *
 * Bytes 0..3 : latched fault flags
 * Byte 4     : packed temperature-sensor states
 * Byte 5     : BQ / measurement status flags
 * Byte 6     : BQ25750 charger fault-status byte
 * Byte 7     : BQ25750 charge-state byte
 *
 * Temperature-state packing:
 * bits 1:0 -> TH1
 * bits 3:2 -> TH2
 * bits 5:4 -> TH3
 * bits 7:6 -> pack-center TS
 *
 * BQ status flags:
 * bit 0 -> BQ76942 CUV active
 * bit 1 -> BQ76942 COV active
 * bit 2 -> BQ25750 measurement valid
 * bit 3 -> BQ76942 measurement valid
 * bit 4 -> BQ25750 watchdog-expired status
 */

#define LITHIUM_CAN_BQ_STATUS_CUV_ACTIVE                (1U << 0)
#define LITHIUM_CAN_BQ_STATUS_COV_ACTIVE                (1U << 1)
#define LITHIUM_CAN_BQ_STATUS_BQ25750_VALID             (1U << 2)
#define LITHIUM_CAN_BQ_STATUS_BQ76942_VALID             (1U << 3)
#define LITHIUM_CAN_BQ_STATUS_WATCHDOG_EXPIRED          (1U << 4)


/* ============================================================
 * HUMAN-READABLE DIAGNOSTICS
 *
 * Classic CAN carries only 8 bytes, so diagnostic text is sent
 * as:
 *
 * 1. DIAGNOSTIC EVENT header - offset 0x8
 * 2. zero or more DIAGNOSTIC TEXT segments - offset 0x9
 *
 * Event header payload:
 * Byte 0 : diagnostic message sequence number
 * Byte 1 : event state (ACTIVE / CLEARED)
 * Byte 2 : fault bit index 0...31
 * Byte 3 : fault severity
 * Byte 4 : charger diagnostic flags low byte
 * Byte 5 : charger diagnostic flags high byte
 * Byte 6 : ASCII text length in bytes
 * Byte 7 : number of text segments
 *
 * Text-segment payload:
 * Byte 0 : diagnostic message sequence number
 * Byte 1 : segment index, bit 7 = final segment
 * Bytes 2..7 : up to 6 ASCII characters
 *
 * The exact human-readable message is generated locally from
 * the active fault, severity and current measurements.
 * ============================================================ */

#define LITHIUM_CAN_DIAG_TEXT_BYTES_PER_FRAME            6U
#define LITHIUM_CAN_DIAG_FINAL_SEGMENT_FLAG              0x80U
#define LITHIUM_CAN_DIAG_MAX_TEXT_LENGTH                 120U


/* ============================================================
 * APPLICATION API
 * ============================================================ */

void Lithium_CAN_Init(void);


/*
 * Non-blocking CAN service.
 *
 * Responsibilities:
 * - periodic binary telemetry
 * - fault-mask telemetry
 * - fault activation / clearing events
 * - segmented human-readable diagnostic text
 * - local CAN restart attempts if the peripheral did not start
 *
 * CAN has no authority to enable charging, disable VSYS or
 * directly change the System FSM.
 *
 * Loss of the CAN telemetry link is reported only as the
 * diagnostics-only LITHIUM_FAULT_CAN warning.  It does not
 * alter charging, balancing or the power path.
 */
void Lithium_CAN_Process(void);


#endif /* LITHIUM_CAN_H */
