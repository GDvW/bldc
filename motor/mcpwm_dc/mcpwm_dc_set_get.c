#include "mcpwm_dc.h"
#include "mcpwm_dc_locals.h"

void mcpwm_dc_set_parking_brake_current(float dutyCycle);
void mcpwm_dc_set_brake_current(float current);
void mcpwm_dc_set_current(float current);

void mcpwm_dc_set_duty(float dutyCycle){
    control_mode = CONTROL_MODE_DUTY;
    dutycycle_set = dutyCycle;
}

// TODO: Temporarily the same, but will change it
void mcpwm_dc_set_duty_noramp(float dutyCycle){
    control_mode = CONTROL_MODE_DUTY;
    dutycycle_set = dutyCycle;
}

void mcpwm_dc_set_pid_speed(float rpm){
    control_mode = CONTROL_MODE_SPEED;
    rpm_set = rpm;
}

float mcpwm_dc_get_duty_cycle_set(void){
    return dutycycle_set;
}

float mcpwm_dc_get_duty_cycle_now(void){
    return dutycycle_now;
}

float mcpwm_dc_get_last_adc_isr_duration(void);
float mcpwm_dc_get_rpm(void);
mc_state mcpwm_dc_get_state(void);
float mcpwm_dc_get_switching_frequency_now(void);
float mcpwm_dc_get_tot_current(void);
float mcpwm_dc_get_tot_current_directional(void);
float mcpwm_dc_get_tot_current_directional_filtered(void);
float mcpwm_dc_get_tot_current_filtered(void);
float mcpwm_dc_get_tot_current_in(void);
float mcpwm_dc_get_tot_current_in_filtered(void);