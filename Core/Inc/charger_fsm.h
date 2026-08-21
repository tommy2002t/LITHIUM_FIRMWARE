#ifndef CHARGER_FSM_H
#define CHARGER_FSM_H


#include "lithium_types.h"


/* ============================================================
 * CHARGER FSM INITIALIZATION
 * ============================================================ */

void ChargerFSM_Init(lithium_context_t *ctx);


/* ============================================================
 * CHARGER FSM EXECUTION
 * ============================================================ */

void ChargerFSM_Run(lithium_context_t *ctx);


/* ============================================================
 * FORCE SAFE CHARGER STATE
 *
 * Immediately requests the charger subsystem to return to its
 * safe OFF state.
 *
 * The actual BQ25750 EN_CHG control will be added when the
 * BQ25750 driver is implemented.
 * ============================================================ */

void ChargerFSM_ForceOff(lithium_context_t *ctx);


#endif /* CHARGER_FSM_H */
