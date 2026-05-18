/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mcpwm.h"
#include "mc_interface.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "ledpwm.h"
#include "terminal.h"
#include "timeout.h"
#include "encoder/encoder.h"
#include "timer.h"

// Structs
typedef struct {
	volatile bool updated;
	volatile unsigned int top;
	volatile unsigned int duty_motor;
	volatile unsigned int duty_brake;
	volatile unsigned int val_sample;
	volatile unsigned int curr1_sample;
	volatile unsigned int curr2_sample;
#ifdef HW_HAS_3_SHUNTS
	volatile unsigned int curr3_sample;
#endif
} mc_timer_struct;

// Private variables
static volatile int direction;
static volatile float dutycycle_set;
static volatile float dutycycle_now;

static volatile mc_configuration *conf;

static volatile mc_control_mode control_mode;
static volatile mc_state state;

static volatile bool init_done = false;

// Private functions
static void set_duty_cycle_hl(float dutyCycle);
static void set_duty_cycle_ll(float dutyCycle);
static void set_duty_cycle_hw(float dutyCycle);
static void stop_pwm_ll(void);
static void stop_pwm_hw(void);

//Initializes the motor controller
void mcpwm_dc_init(volatile mc_configuration *configuration){
	utils_sys_lock_cnt();

	init_done = false;

	conf = configuration;

	control_mode = CONTROL_MODE_NONE;
	state = MC_STATE_OFF;

	// Initialize clocks
	// TIM1 is used to generate PWM signals
	// TIM8 is used for ADC sampling

	// Create structs for configuration of clocks
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	// Make sure clocks are in a known state
	// Temporarily disabling ADC because that will only be needed later
	TIM_DeInit(TIM1);
	// TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	// TIM8->CNT = 0;

	// Turn on the TIM1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Configure clock params for TIM1
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	// Apply and initialize TIM1
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	// PWM1 -> high when CNT < CCR, low otherwise
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	// Initial duty cycle at 50%
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;

#ifndef INVERTED_TOP_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // gpio high = top fets on
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  // gpio high = bottom fets on
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	
	// Apply to OC1 till OC4 (Output compare, 1-3 used for PWM, 4 for ADC)
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	// Enable TIM1 and TIM8
	TIM_Cmd(TIM1, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	utils_sys_unlock_cnt();

	init_done = true;
}

void mcpwm_dc_deinit(void){
	if (!init_done){
		return;
	}

	init_done = false;
}

void mcpwm_dc_init_done(void){
	return init_done;
}

void mcpwm_dc_set_configuration(volatile mc_configuration *configuration){
	utils_sys_lock_cnt();
	conf = configuration;
	utils_sys_unlock_cnt();
}

void mcpwm_dc_set_brake_current(float current);
void mcpwm_dc_set_current(float current);

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_dc_set_duty(float dutyCycle){
	control_mode = CONTROL_MODE_DUTY;
	set_duty_cycle_hl(dutyCycle);
}

/**
 * Switch off all FETs.
 */
void mcpwm_dc_stop_pwm(void){
	control_mode = CONTROL_MODE_NONE;
	stop_pwm_ll();
}

void mcpwm_dc_set_duty_noramp(float dutyCycle);
void mcpwm_dc_set_pid_speed(float rpm);
void mcpwm_dc_set_pid_pos(float pos);
int mcpwm_dc_set_tachometer_value(int steps);

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
int mcpwm_dc_get_tachometer_value(bool reset);
int mcpwm_dc_get_tachometer_abs_value(bool reset);
float mcpwm_dc_get_tot_current(void);
float mcpwm_dc_get_tot_current_directional(void);
float mcpwm_dc_get_tot_current_directional_filtered(void);
float mcpwm_dc_get_tot_current_filtered(void);
float mcpwm_dc_get_tot_current_in(void);
float mcpwm_dc_get_tot_current_in_filtered(void);

void mcpwm_dc_release_motor(void);
bool mcpwm_dc_is_dccal_done(void);


void mcpwm_dc_adc_int_handler(void *p, uint32_t flags);

/**
 * High-level duty cycle setter. Will set the ramping goal of the duty cycle.
 * If motor is not running, it will be started in different ways depending on
 * whether it is moving or not.
 *
 * @param dutyCycle
 * The duty cycle in the range [-MCPWM_MAX_DUTY_CYCLE MCPWM_MAX_DUTY_CYCLE]
 * If the absolute value of the duty cycle is less than MCPWM_MIN_DUTY_CYCLE,
 * the motor phases will be shorted to brake the motor.
 */
static void set_duty_cycle_hl(float dutyCycle){
	// Clip the number between minimal and maximal duty cycle
	utils_truncate_number(&dutyCycle, -conf->l_max_duty, conf->l_max_duty);

	dutycycle_set = dutyCycle;

	if (state != MC_STATE_RUNNING) {
		// Check for valid duty cycle
		if (fabsf(dutyCycle) >= conf->l_min_duty) {
			// dutycycle_now is updated by the back-emf detection. If the motor already
			// is spinning, it will be non-zero.
			if (fabsf(dutycycle_now) < conf->l_min_duty) {
				dutycycle_now = SIGN(dutyCycle) * conf->l_min_duty;
			}

			set_duty_cycle_ll(dutycycle_now);
		} else {
			// In case the motor is already spinning, set the state to running
			// so that it can be ramped down before the full brake is applied.
			// WAS DC condition
			if (fabsf(dutycycle_now) > 0.1) {
				state = MC_STATE_RUNNING;
			} else {
				full_brake_ll();
			}
		}
	}
}
static void set_duty_cycle_ll(float dutyCycle);
static void set_duty_cycle_hw(float dutyCycle);

static void stop_pwm_ll(void){
	state = MC_STATE_OFF;
	stop_pwm_hw();
}
static void stop_pwm_hw(void){
#ifdef HW_HAS_DRV8313
	DISABLE_BR();
#endif

	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	set_switching_frequency(conf->m_bldc_f_sw_max);
}