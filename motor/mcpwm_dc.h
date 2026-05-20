#ifndef MCPWM_DC_H_
#define MCPWM_DC_H_

#include "conf_general.h"

// Functions
void mcpwm_dc_init(volatile mc_configuration *configuration);
void mcpwm_dc_deinit(void);
bool mcpwm_dc_init_done(void);
void mcpwm_dc_set_configuration(volatile mc_configuration *configuration);
void mcpwm_adc_inj_int_handler(void);

void mcpwm_dc_set_brake_current(float current);
void mcpwm_dc_set_current(float current);
void mcpwm_dc_set_duty(float dutyCycle);
void mcpwm_dc_set_duty_noramp(float dutyCycle);
void mcpwm_dc_set_pid_speed(float rpm);
void mcpwm_dc_set_pid_pos(float pos);
int mcpwm_dc_set_tachometer_value(int steps);

float mcpwm_dc_get_duty_cycle_set(void);
float mcpwm_dc_get_duty_cycle_now(void);
float mcpwm_dc_get_last_adc_isr_duration(void);
float mcpwm_dc_get_rpm(void);
mc_state mcpwm_dc_get_state(void);
float mcpwm_dc_get_switching_frequency_now(void);
int mcpwm_dc_get_tachometer_value(bool reset);
int mcpwm_dc_get_tachometer_abs_value(bool reset);
float mcpwm_dc_get_tot_current(void);
float mcpwm_dc_get_tot_current_directional(void);
float mcpwm_dc_get_tot_current_directional_filtered(void);
float mcpwm_dc_get_tot_current_filtered(void);
float mcpwm_dc_get_tot_current_in(void);
float mcpwm_dc_get_tot_current_in_filtered(void);

void mcpwm_dc_release_motor(void);
void mcpwm_dc_stop_pwm(void);
bool mcpwm_dc_is_dccal_done(void);


void mcpwm_dc_adc_int_handler(void *p, uint32_t flags);

/*
 * Fixed parameters
 */
#define MCPWM_DETECT_STOP_TIME			500		// Ignore commands for this duration in msec after a detect command


#endif /* MCPWM_DC_H_ */
