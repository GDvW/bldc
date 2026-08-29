/**
 * This file contains all setter functions and commands that are used by the outside world (outside the mcpwm_dc folder)
 */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "ledpwm.h"
#include "terminal.h"
#include "timeout.h"
#include "encoder/encoder.h"
#include "timer.h"
#include "mcpwm_dc.h"
#include "mcpwm_dc_locals.h"

void mcpwm_dc_set_parking_brake(bool output_enabled)
{
    parking_brake_output_set = output_enabled;
}

void mcpwm_dc_set_duty(float dutycycle)
{
    control_mode = CONTROL_MODE_DUTY;
    mcpwm_dc_set_duty_hl(dutycycle);
}

void mcpwm_dc_set_duty_noramp(float dutycycle)
{
    control_mode = CONTROL_MODE_DUTY;

    if (state != MC_STATE_RUNNING)
    {
        mcpwm_dc_set_duty_hl(dutycycle);
    }
    else
    {
        dutycycle_set = dutycycle;
        dutycycle_now = dutycycle;
        mcpwm_dc_set_duty_ll(dutycycle);
    }
}

void mcpwm_dc_set_current(float current)
{
    if (fabsf(current) < conf->cc_min_current)
    {
        control_mode = CONTROL_MODE_NONE;
        stop_pwm_motor_ll();
        return;
    }

    utils_truncate_number(&current, -conf->l_current_max * conf->l_current_max_scale,
                          conf->l_current_max * conf->l_current_max_scale);

    control_mode = CONTROL_MODE_CURRENT;
    current_set = current;

    if (state != MC_STATE_RUNNING)
    {
        mcpwm_dc_set_duty_hl(SIGN(current) * conf->l_min_duty);
    }
}

void mcpwm_dc_set_pid_speed(float rpm)
{
    control_mode = CONTROL_MODE_SPEED;
    rpm_set = rpm;
}

void mcpwm_dc_release_motor(void)
{
    current_set = 0.0;
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_motor_ll();
}

/**
 * Stops all PWM outputs
 */
void mcpwm_dc_stop_pwm(void)
{
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_ll();
}
