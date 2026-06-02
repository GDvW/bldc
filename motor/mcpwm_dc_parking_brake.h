#ifndef MOTOR_MCPWM_DC_PARKING_BRAKE_H
#define MOTOR_MCPWM_DC_PARKING_BRAKE_H

#include "conf_general.h"

void mcpwm_dc_set_parking_brake(bool engaged);
void update_duty_cycle_parking_brake(void);
void mcpwm_dc_parking_brake_set_configuration(volatile mc_configuration *configuration);
void mcpwm_dc_parking_brake_init(volatile mc_configuration *configuration);
void mcpwm_dc_parking_brake_stop_pwm(void);
bool mcpwm_dc_is_parking_brake_enabled(void);
bool mcpwm_dc_is_parking_brake_engaged(void);
float mcpwm_dc_get_parking_brake_duty(void);
uint16_t mcpwm_dc_get_ccr_called(void);
uint32_t mcpwm_reg_get_CCR2(void);
uint32_t mcpwm_reg_get_ARR(void);
uint32_t mcpwm_reg_get_CCER(void);
uint32_t mcpwm_reg_get_CCMR1(void);
uint32_t mcpwm_reg_get_BDTR(void);
#endif