#ifndef LITHIUM_APP_H
#define LITHIUM_APP_H


#include "lithium_types.h"


/* ============================================================
 * GLOBAL APPLICATION DATA
 * ============================================================ */

extern lithium_context_t g_lithium;

extern lithium_event_flags_t g_lithium_events;


/* ============================================================
 * EARLY STARTUP SAFETY
 * ============================================================ */

void Lithium_EarlySafeGPIO_Init(void);


/* ============================================================
 * APPLICATION INITIALIZATION
 * ============================================================ */

void Lithium_AppInit(void);


/* ============================================================
 * FAST MAIN-LOOP SERVICE
 *
 * Must be called continuously from main().
 *
 * Performs:
 *
 * - BQ25750 non-blocking driver service
 * - BQ76942 non-blocking driver service
 * - starts periodic measurement snapshots
 * - consumes completed measurements
 * - processes interrupt flags
 * - services BQ25750 watchdog
 * - performs fast power-current safety monitoring
 *
 * No SPI/I2C operation is performed directly inside an ISR.
 * ============================================================ */

void Lithium_ProcessEvents(void);


/* ============================================================
 * HIGH-LEVEL SYSTEM FSM
 *
 * Called every LITHIUM_SYSTEM_PERIOD_MS.
 * ============================================================ */

void Lithium_SystemFSM_Run(void);


/* ============================================================
 * FAULT MANAGEMENT
 * ============================================================ */

void Lithium_SetFault(
        uint32_t fault,
        fault_severity_t severity);


void Lithium_ClearFault(
        uint32_t fault);


#endif /* LITHIUM_APP_H */
