/**
 * Contains setters that are local.
 */
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mcpwm_dc.h"
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "ledpwm.h"
#include "terminal.h"
#include "timeout.h"
#include "encoder/encoder.h"
#include "timer.h"

#include "mcpwm_dc_locals.h"
#include "mcpwm_dc_hw.h"
#include "mcpwm_dc.h"

static void start_control_loop(float dutycycle);

/**
 * Applies some checks and then sets the dutycycle. The control loop will take over this setpoint.
 * However when the motor is not in running state, make sure it enters that state (otherwise the control loop will neglect it)
 */
void mcpwm_dc_set_duty_internal_hl(float dutycycle)
{
    utils_truncate_number(&dutycycle, -conf->l_max_duty, conf->l_max_duty);

    dutycycle_set = dutycycle;

    if (state != MC_STATE_RUNNING)
    {
        start_control_loop(dutycycle);
    }
}

/**
 * Applies the duty cycle directly. This should only be used by the control loop and when starting the motor
 */
void mcpwm_dc_set_duty_direct_ll(float dutycycle)
{
    // Determine the direction
    float dutycycle_abs = fabsf(dutycycle);
    if (dutycycle_abs >= conf->l_min_duty)
    {
        direction = signbit(dutycycle) == 0 ? DIRECTION_FORWARD : DIRECTION_BACKWARD;
    }

    // Braking condition
    if (dutycycle_abs < conf->l_min_duty)
    {
        // NOTE: Contained checks for maximum speed of motor
        // Removed those because there is no encoder, but could be restored
        full_brake_ll();
        return;
    }

    // Clamp the duty cycle
    if (dutycycle_abs > conf->l_max_duty)
    {
        dutycycle_abs = conf->l_max_duty;
    }

    // Apply
    set_dutycycle_hw(dutycycle_abs);
    state = MC_STATE_RUNNING;
    set_direction_hw();
}

void stop_pwm_ll(void)
{
    state = MC_STATE_OFF;
    stop_pwm_hw();
}
void stop_pwm_motor_ll(void)
{
    state = MC_STATE_OFF;
    stop_pwm_hw();
}

void full_brake_ll(void)
{
    state = MC_STATE_FULL_BRAKE;
    full_brake_hw();
}

static void start_control_loop(float dutycycle)
{
    // Check if dutycycle set will make motor run (case 1) or stop (case 2)
    if (fabsf(dutycycle) >= conf->l_min_duty)
    {
        // dutycycle_now is updated by the back-emf detection. If the motor already
        // is spinning, it will be non-zero.
        // NOTE: not sure if above comment also applies to DC motors
        if (fabsf(dutycycle_now) < conf->l_min_duty)
        {
            dutycycle_now = SIGN(dutycycle) * conf->l_min_duty;
        }

        mcpwm_dc_set_duty_direct_ll(dutycycle_now);
    }
    else
    {
        // In case the motor is already spinning, set the state to running
        // so that it can be ramped down before the full brake is applied.
        if (fabsf(dutycycle_now) > 0.1)
        {
            state = MC_STATE_RUNNING;
        }
        else
        {
            full_brake_ll();
        }
    }
}