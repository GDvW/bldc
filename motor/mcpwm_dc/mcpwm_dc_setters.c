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
    state_parking_brake = MC_STATE_RUNNING;
    parking_brake_output_set = output_enabled;
}

void mcpwm_dc_set_duty(float dutycycle)
{
    speed_control_active = false;
    control_mode = CONTROL_MODE_DUTY;
    mcpwm_dc_set_duty_hl(dutycycle);
}

void mcpwm_dc_set_duty_noramp(float dutycycle)
{
    speed_control_active = false;
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
    speed_control_active = false;
    mcpwm_dc_set_current_hl(current);
}

void mcpwm_dc_set_pid_speed(float rpm)
{
    // Control mode will be set to another mode in a few moments, but that does not matter
    control_mode = CONTROL_MODE_SPEED;
    speed_control_active = true;
    rpm_set = rpm;
}

void mcpwm_dc_release_motor(void)
{
    current_set = 0.0;
    speed_control_active = false;
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_motor_ll();
}

/**
 * Stops all PWM outputs
 */
void mcpwm_dc_stop_pwm(void)
{
    speed_control_active = false;
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_ll();
}
