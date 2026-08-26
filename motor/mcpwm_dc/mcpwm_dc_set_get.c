/**
 * This file contains all getter/setter functions that are used by the outside world (outside the mcpwm_dc folder)
 * Actual logic should not be here.
 */

#include "mcpwm_dc.h"
#include "mcpwm_dc_locals.h"

// TODO: Add checks for valid value of setpoint

void mcpwm_dc_set_parking_brake_current(bool output_enabled)
{
    parking_brake_output_set = output_enabled;
}

void mcpwm_dc_set_current(float current)
{
    control_mode = CONTROL_MODE_CURRENT;
    current_set = current;
}

void mcpwm_dc_set_duty(float duty_cycle)
{
    control_mode = CONTROL_MODE_DUTY;
    mcpwm_dc_set_duty_cleaned(duty_cycle);
}

// TODO: Temporarily the same, but will change it
void mcpwm_dc_set_duty_noramp(float duty_cycle)
{
    control_mode = CONTROL_MODE_DUTY;
    mcpwm_dc_set_duty_cleaned(duty_cycle);
}

void mcpwm_dc_set_pid_speed(float rpm)
{
    control_mode = CONTROL_MODE_SPEED;
    rpm_set = rpm;
}

float mcpwm_dc_get_duty_cycle_set(void)
{
    return dutycycle_set;
}

float mcpwm_dc_get_duty_cycle_now(void)
{
    return dutycycle_now;
}

float mcpwm_dc_get_last_adc_isr_duration(void)
{
    return last_adc_isr_duration;
}

mc_state mcpwm_dc_get_state(void)
{
    return state;
}

float mcpwm_dc_get_switching_frequency_now(void)
{
    return switching_frequency_now;
}

float mcpwm_dc_get_tot_current(void)
{
    return last_current_sample;
}

float mcpwm_dc_get_tot_current_directional(void)
{
    return mcpwm_dc_get_tot_current() * fabsf(dutycycle_now);
}

float mcpwm_dc_get_tot_current_directional_filtered(void)
{
    return mcpwm_dc_get_tot_current_filtered() * fabsf(dutycycle_now);
}

float mcpwm_dc_get_tot_current_filtered(void)
{
    return last_current_sample_filtered;
}

float mcpwm_dc_get_tot_current_in(void)
{
    return mcpwm_dc_get_tot_current() * fabsf(dutycycle_now);
}

float mcpwm_dc_get_tot_current_in_filtered(void)
{
    return mcpwm_dc_get_tot_current_filtered() * fabsf(dutycycle_now);
}

bool mcpwm_dc_is_dccal_done(void)
{
    return dccal_done;
}

bool mcpwm_dc_init_done(void)
{
    return init_done;
}

/**
 * Stops all PWM outputs
 */
void mcpwm_dc_stop_pwm(void)
{
    control_mode = CONTROL_MODE_NONE;
    // TODO: Actually stop the motor
}