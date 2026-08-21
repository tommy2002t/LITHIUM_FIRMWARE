#ifndef BALANCING_FSM_H
#define BALANCING_FSM_H


#include "lithium_types.h"


/* ============================================================
 * BALANCING FSM INITIALIZATION
 * ============================================================ */

void BalancingFSM_Init(lithium_context_t *ctx);


/* ============================================================
 * BALANCING FSM EXECUTION
 * ============================================================ */

void BalancingFSM_Run(lithium_context_t *ctx);


/* ============================================================
 * FORCE BALANCING OFF
 *
 * Immediately requests balancing to stop and places the
 * Balancing FSM in BALANCE_OFF.
 *
 * The actual BQ76942 balancing command will be added when the
 * BQ76942 driver is implemented.
 * ============================================================ */

void BalancingFSM_ForceOff(lithium_context_t *ctx);


#endif /* BALANCING_FSM_H */
