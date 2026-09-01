#ifndef MCPWM_DC
#define MCPWM_DC

// TODO: more consistency in naming. for example, mcpmw_dc only for public API
// TODO: add documentation

#include "conf_general.h"
#include <stdint.h>
#include <stdbool.h>

// Module functions
void mcpwm_dc_init(volatile mc_configuration *configuration);
void mcpwm_dc_deinit(void);
void mcpwm_dc_set_configuration(volatile mc_configuration *configuration);

// Interrupts
void mcpwm_dc_adc_inj_int_handler(void);
void mcpwm_dc_adc_int_handler(void *p, uint32_t flags);

// Setters
void mcpwm_dc_set_parking_brake(bool output_enabled);
void mcpwm_dc_set_duty(float dutycycle);
void mcpwm_dc_set_duty_noramp(float dutycycle);
void mcpwm_dc_set_current(float current);
void mcpwm_dc_set_pid_speed(float rpm);

void mcpwm_dc_release_motor(void);
void mcpwm_dc_stop_pwm(void);

// Getters
float mcpwm_dc_get_duty_cycle_set(void);
float mcpwm_dc_get_duty_cycle_now(void);
float mcpwm_dc_get_rpm(void);
float mcpwm_dc_get_tot_current(void);
float mcpwm_dc_get_tot_current_filtered(void);
float mcpwm_dc_get_tot_current_directional(void);
float mcpwm_dc_get_tot_current_directional_filtered(void);
float mcpwm_dc_get_tot_current_in(void);
float mcpwm_dc_get_tot_current_in_filtered(void);
mc_state mcpwm_dc_get_state(void);
mc_control_mode mcpwm_dc_get_control_mode(void);
// Get which speed control is really being used.
// If the normal control mode is speed, at the base, it is duty or current. This method retrieves that.
mc_control_mode mcpwm_dc_get_control_mode_base(void);
float mcpwm_dc_get_switching_frequency_now(void);
bool mcpwm_dc_is_dccal_done(void);
bool mcpwm_dc_init_done(void);
float mcpwm_dc_get_last_adc_isr_duration(void);
float mcpwm_dc_get_last_inj_adc_isr_duration(void);

void mcpwm_dc_meas_get_info(
    int *curr_adc_source_mask,
    int *curr_start_samples,
    int *curr0_sum,
    int *curr1_sum,
    int *curr2_sum,
    int *curr0_offset,
    int *curr1_offset,
    int *curr2_offset);

// Debug
bool mcpwm_dc_was_h_bridge_configured(void);

#endif