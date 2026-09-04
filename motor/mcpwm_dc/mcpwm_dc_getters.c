/**
 * This file contains all getter functions that are used by the outside world (outside the mcpwm_dc folder)
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

float mcpwm_dc_get_duty_cycle_set(void)
{
    return dutycycle_set;
}

float mcpwm_dc_get_duty_cycle_now(void)
{
    return dutycycle_now;
}

float mcpwm_dc_get_rpm(void)
{
    return rpm_now;
}

float mcpwm_dc_get_tot_current(void)
{
    return last_current_sample;
}

float mcpwm_dc_get_tot_current_filtered(void)
{
    return last_current_sample_filtered;
}

float mcpwm_dc_get_tot_current_directional(void)
{
    const float retval = mcpwm_dc_get_tot_current();
	return dutycycle_now > 0.0 ? retval : -retval;
}

float mcpwm_dc_get_tot_current_directional_filtered(void)
{
    const float retval = mcpwm_dc_get_tot_current_filtered();
	return dutycycle_now > 0.0 ? retval : -retval;
}

float mcpwm_dc_get_tot_current_in(void)
{
    return mcpwm_dc_get_tot_current() * fabsf(dutycycle_now) + mcpwm_dc_get_tot_pb_current() * fabsf(duty_now_parking_brake);
}

float mcpwm_dc_get_tot_current_in_filtered(void)
{
    return mcpwm_dc_get_tot_current_filtered() * fabsf(dutycycle_now) + mcpwm_dc_get_tot_pb_current_filtered() * fabsf(duty_now_parking_brake);
}

float mcpwm_dc_get_tot_pb_current(void)
{
    return pb_last_current_sample;
}

float mcpwm_dc_get_tot_pb_current_filtered(void)
{
    return pb_last_current_sample_filtered;
}

mc_state mcpwm_dc_get_state(void)
{
    return state;
}

mc_control_mode mcpwm_dc_get_control_mode(void)
{
    if (speed_control_active)
    {
        return CONTROL_MODE_SPEED;
    }
    return control_mode;
}

mc_control_mode mcpwm_dc_get_control_mode_base(void)
{
    if (speed_control_active)
    {
        return CONTROL_MODE_SPEED;
    }
    return control_mode;
}

float mcpwm_dc_get_switching_frequency_now(void)
{
    return switching_frequency_now;
}

bool mcpwm_dc_is_dccal_done(void)
{
    return dccal_done;
}

bool mcpwm_dc_init_done(void)
{
    return init_done;
}

float mcpwm_dc_get_last_adc_isr_duration(void)
{
    return last_adc_isr_duration;
}

float mcpwm_dc_get_last_inj_adc_isr_duration(void)
{
    return last_adc_inj_isr_duration;
}

bool mcpwm_dc_was_h_bridge_configured(void)
{
    return was_h_bridge_configured;
}