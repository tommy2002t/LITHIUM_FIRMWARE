#include "lithium_led.h"

#include "main.h"


#define LED_SHORT_PULSE_MS                 150U

#define LED_BOOT_PERIOD_MS                1000U
#define LED_INIT_PERIOD_MS                2000U
#define LED_SELF_TEST_PERIOD_MS           2000U

#define LED_CHARGING_PERIOD_MS            1000U
#define LED_CHARGING_ON_TIME_MS            500U

/*
 * WARNING
 *
 * Slow alternating red / green:
 *
 * RED   1.0 s
 * GREEN 1.0 s
 *
 * The LED communicates warning severity only.
 * Exact cause is reported through CAN diagnostics.
 */
#define LED_WARNING_PERIOD_MS             2000U
#define LED_WARNING_RED_TIME_MS           1000U

#define LED_RECOVERABLE_PERIOD_MS         1000U
#define LED_RECOVERABLE_ON_TIME_MS         500U

#define LED_BQ_ERROR_PERIOD_MS            2000U
#define LED_PULSE_GAP_MS                   150U

#define LED_LATCHED_PERIOD_MS              400U
#define LED_LATCHED_ON_TIME_MS             200U


static led_pattern_t current_pattern =
        LED_PATTERN_OFF;

static uint32_t pattern_start_time_ms = 0U;


static void Lithium_LED_Write(
        bool red_on,
        bool green_on);

static bool Lithium_LED_PulseActive(
        uint32_t elapsed_ms,
        uint32_t pulse_number);


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void Lithium_LED_Init(void)
{
    current_pattern =
            LED_PATTERN_OFF;


    pattern_start_time_ms =
            HAL_GetTick();


    Lithium_LED_Write(
            false,
            false);
}


/* ============================================================
 * PATTERN SELECTION
 * ============================================================ */

void Lithium_LED_SetPattern(
        led_pattern_t pattern)
{
    /*
     * Re-requesting the same pattern must not restart its
     * timing. This keeps all blink patterns stable even when
     * Lithium_UpdateStatusIndication() is called repeatedly.
     */
    if (pattern ==
        current_pattern)
    {
        return;
    }


    current_pattern =
            pattern;


    pattern_start_time_ms =
            HAL_GetTick();


    Lithium_LED_Write(
            false,
            false);
}


/* ============================================================
 * NON-BLOCKING LED UPDATE
 * ============================================================ */

void Lithium_LED_Update(void)
{
    uint32_t elapsed_ms;
    uint32_t phase_ms;


    elapsed_ms =
            HAL_GetTick() -
            pattern_start_time_ms;


    switch (current_pattern)
    {
        /* ----------------------------------------------------
         * OFF
         * ---------------------------------------------------- */

        case LED_PATTERN_OFF:
        {
            Lithium_LED_Write(
                    false,
                    false);


            break;
        }


        /* ----------------------------------------------------
         * BOOT
         *
         * One short green pulse.
         * ---------------------------------------------------- */

        case LED_PATTERN_BOOT:
        {
            phase_ms =
                    elapsed_ms %
                    LED_BOOT_PERIOD_MS;


            Lithium_LED_Write(
                    false,
                    phase_ms <
                    LED_SHORT_PULSE_MS);


            break;
        }


        /* ----------------------------------------------------
         * INIT
         *
         * Two short green pulses.
         * ---------------------------------------------------- */

        case LED_PATTERN_INIT:
        {
            phase_ms =
                    elapsed_ms %
                    LED_INIT_PERIOD_MS;


            Lithium_LED_Write(
                    false,
                    Lithium_LED_PulseActive(
                            phase_ms,
                            0U) ||
                    Lithium_LED_PulseActive(
                            phase_ms,
                            1U));


            break;
        }


        /* ----------------------------------------------------
         * SELF TEST
         *
         * Three short green pulses.
         * ---------------------------------------------------- */

        case LED_PATTERN_SELF_TEST:
        {
            phase_ms =
                    elapsed_ms %
                    LED_SELF_TEST_PERIOD_MS;


            Lithium_LED_Write(
                    false,
                    Lithium_LED_PulseActive(
                            phase_ms,
                            0U) ||
                    Lithium_LED_PulseActive(
                            phase_ms,
                            1U) ||
                    Lithium_LED_PulseActive(
                            phase_ms,
                            2U));


            break;
        }


        /* ----------------------------------------------------
         * READY
         *
         * Solid green.
         * ---------------------------------------------------- */

        case LED_PATTERN_READY:
        {
            Lithium_LED_Write(
                    false,
                    true);


            break;
        }


        /* ----------------------------------------------------
         * CHARGING
         *
         * Slow green blink:
         * 500 ms ON / 500 ms OFF.
         * ---------------------------------------------------- */

        case LED_PATTERN_CHARGING:
        {
            phase_ms =
                    elapsed_ms %
                    LED_CHARGING_PERIOD_MS;


            Lithium_LED_Write(
                    false,
                    phase_ms <
                    LED_CHARGING_ON_TIME_MS);


            break;
        }


        /* ----------------------------------------------------
         * WARNING
         *
         * Slow alternating RED / GREEN.
         *
         * Warning means:
         *
         * - something abnormal has been detected,
         * - the system still has enough confidence to continue
         *   its permitted operation,
         * - exact cause must be read through CAN diagnostics.
         *
         * 0...999 ms     -> RED
         * 1000...1999 ms -> GREEN
         * ---------------------------------------------------- */

        case LED_PATTERN_WARNING:
        {
            phase_ms =
                    elapsed_ms %
                    LED_WARNING_PERIOD_MS;


            Lithium_LED_Write(
                    phase_ms <
                    LED_WARNING_RED_TIME_MS,
                    phase_ms >=
                    LED_WARNING_RED_TIME_MS);


            break;
        }


        /* ----------------------------------------------------
         * RECOVERABLE FAULT
         *
         * Slow red blink:
         * 500 ms ON / 500 ms OFF.
         * ---------------------------------------------------- */

        case LED_PATTERN_FAULT_RECOVERABLE:
        {
            phase_ms =
                    elapsed_ms %
                    LED_RECOVERABLE_PERIOD_MS;


            Lithium_LED_Write(
                    phase_ms <
                    LED_RECOVERABLE_ON_TIME_MS,
                    false);


            break;
        }


        /* ----------------------------------------------------
         * BQ25750 INIT / AVAILABILITY ERROR
         *
         * One short red pulse.
         * ---------------------------------------------------- */

        case LED_PATTERN_BQ25750_ERROR:
        {
            phase_ms =
                    elapsed_ms %
                    LED_BQ_ERROR_PERIOD_MS;


            Lithium_LED_Write(
                    Lithium_LED_PulseActive(
                            phase_ms,
                            0U),
                    false);


            break;
        }


        /* ----------------------------------------------------
         * BQ76942 INIT / AVAILABILITY ERROR
         *
         * Two short red pulses.
         * ---------------------------------------------------- */

        case LED_PATTERN_BQ76942_ERROR:
        {
            phase_ms =
                    elapsed_ms %
                    LED_BQ_ERROR_PERIOD_MS;


            Lithium_LED_Write(
                    Lithium_LED_PulseActive(
                            phase_ms,
                            0U) ||
                    Lithium_LED_PulseActive(
                            phase_ms,
                            1U),
                    false);


            break;
        }


        /* ----------------------------------------------------
         * BOTH BQ DEVICES UNAVAILABLE
         *
         * Three short red pulses.
         * ---------------------------------------------------- */

        case LED_PATTERN_BOTH_BQ_ERROR:
        {
            phase_ms =
                    elapsed_ms %
                    LED_BQ_ERROR_PERIOD_MS;


            Lithium_LED_Write(
                    Lithium_LED_PulseActive(
                            phase_ms,
                            0U) ||
                    Lithium_LED_PulseActive(
                            phase_ms,
                            1U) ||
                    Lithium_LED_PulseActive(
                            phase_ms,
                            2U),
                    false);


            break;
        }


        /* ----------------------------------------------------
         * LATCHED APPLICATION FAULT
         *
         * Fast red blink:
         * 200 ms ON / 200 ms OFF.
         * ---------------------------------------------------- */

        case LED_PATTERN_FAULT_LATCHED:
        {
            phase_ms =
                    elapsed_ms %
                    LED_LATCHED_PERIOD_MS;


            Lithium_LED_Write(
                    phase_ms <
                    LED_LATCHED_ON_TIME_MS,
                    false);


            break;
        }


        /* ----------------------------------------------------
         * INVALID APPLICATION PATTERN
         * ---------------------------------------------------- */

        default:
        {
            /*
             * Invalid application LED pattern:
             * fast red, never solid red.
             *
             * Solid red remains reserved exclusively for the
             * MCU/HAL Error_Handler().
             */
            phase_ms =
                    elapsed_ms %
                    LED_LATCHED_PERIOD_MS;


            Lithium_LED_Write(
                    phase_ms <
                    LED_LATCHED_ON_TIME_MS,
                    false);


            break;
        }
    }
}


/* ============================================================
 * PHYSICAL LED WRITE
 * ============================================================ */

static void Lithium_LED_Write(
        bool red_on,
        bool green_on)
{
    HAL_GPIO_WritePin(
            LED1_RED_GPIO_Port,
            LED1_RED_Pin,
            red_on ?
                    GPIO_PIN_SET :
                    GPIO_PIN_RESET);


    HAL_GPIO_WritePin(
            LED2_GREEN_GPIO_Port,
            LED2_GREEN_Pin,
            green_on ?
                    GPIO_PIN_SET :
                    GPIO_PIN_RESET);
}


/* ============================================================
 * SHORT-PULSE HELPER
 * ============================================================ */

static bool Lithium_LED_PulseActive(
        uint32_t elapsed_ms,
        uint32_t pulse_number)
{
    uint32_t pulse_start_ms;
    uint32_t pulse_end_ms;


    pulse_start_ms =
            pulse_number *
            (LED_SHORT_PULSE_MS +
             LED_PULSE_GAP_MS);


    pulse_end_ms =
            pulse_start_ms +
            LED_SHORT_PULSE_MS;


    return
            ((elapsed_ms >=
              pulse_start_ms) &&
             (elapsed_ms <
              pulse_end_ms));
}
