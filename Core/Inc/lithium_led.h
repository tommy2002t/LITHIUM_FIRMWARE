#ifndef LITHIUM_LED_H
#define LITHIUM_LED_H


#include "lithium_types.h"


/* ============================================================
 * LED INITIALIZATION
 *
 * Initializes the software-side LED state and forces both LEDs
 * OFF.
 * ============================================================ */

void Lithium_LED_Init(void);


/* ============================================================
 * SET LED PATTERN
 *
 * Changes the requested LED indication pattern.
 *
 * This function must never block.
 * ============================================================ */

void Lithium_LED_SetPattern(led_pattern_t pattern);


/* ============================================================
 * LED UPDATE
 *
 * Executes the currently selected LED pattern using HAL_GetTick.
 *
 * Must be called continuously from the main loop.
 *
 * No HAL_Delay() is used.
 * ============================================================ */

void Lithium_LED_Update(void);


#endif /* LITHIUM_LED_H */
