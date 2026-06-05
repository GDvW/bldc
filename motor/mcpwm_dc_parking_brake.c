/**
 * Contains all functions that have to do with the parking brake to seperate the parking brake from the normal driving logic.
 * These functions are called from mcpwm_dc.c so they will only run if the motor type is DC.
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
#include "mcpwm_dc_parking_brake.h"

static volatile mc_configuration *conf;
static volatile float switching_frequency_now;

static volatile float dutycycle_parking_brake_now;
static volatile bool parking_brake_output_engaged;

static volatile uint16_t is_ccr_called;

static void apply_parking_brake(float dutycycle, bool output_engaged);

/**
 * Initializes the parking brake driver.
 * The pins and timer are initialized in mcpwm_dc.c because some things can be reused
 * 
 * @param mc_configuration The motor config
 */
void mcpwm_dc_parking_brake_init(volatile mc_configuration *configuration)
{
    utils_sys_lock_cnt();
    conf = configuration;
    utils_sys_unlock_cnt();

    dutycycle_parking_brake_now = 0;
    parking_brake_output_engaged = false;
    is_ccr_called = 0;
}

/**
 * Updates the configuration
 * 
 * @param mc_configuration The motor config
 */
void mcpwm_dc_parking_brake_set_configuration(volatile mc_configuration *configuration)
{
    // Stop everything first to be safe
    dutycycle_parking_brake_now = 0;
    parking_brake_output_engaged = false;
    mcpwm_dc_parking_brake_stop_pwm();

    utils_sys_lock_cnt();
    conf = configuration;
    utils_sys_unlock_cnt();
}

/**
 * The parking brake control loop
 * Should be called from mcpwm_dc_adc_int_handler in mcpwm_dc.c
 * Now only sets a fixed dutycycle. This should be changed to a PID controller for current.
 */
void update_duty_cycle_parking_brake(void)
{
    float dutycycle = parking_brake_output_engaged ? .5f : 0.0f;
    dutycycle_parking_brake_now = dutycycle;
    // Apply
    apply_parking_brake(dutycycle, parking_brake_output_engaged);
}

/**
 * Applies the dutycycle and state of the parking brake if configured so in config.
 *
 * @param dutycycle - the dutycycle to apply
 * @param output_engaged - If engaged is true, make sure the parking brake output is on.
 */
static void apply_parking_brake(float dutycycle, bool output_engaged)
{
    // Only run if parking brake is enabled in config
    if (!conf->dc_enable_parking_brake)
    {
        mcpwm_dc_parking_brake_stop_pwm();
        return;
    }

    // If parking brake output is not engaged, make sure no current flows
    if (!output_engaged)
    {
        TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive); 
        TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

        return;
    }

    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    utils_truncate_number(&dutycycle, conf->l_min_duty, conf->l_max_duty);

    TIM1->CCR2 = (uint16_t)((float)TIM1->ARR * dutycycle);
    is_ccr_called += 1;
}

/**
 * Stop the PWM output of the brake
 */
void mcpwm_dc_parking_brake_stop_pwm(void)
{
    TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/**
 * Enables the parking brake output. The real switching happens in the control loop
 *
 * @param output_engaged whether to engage the output
 */
void mcpwm_dc_set_parking_brake(bool output_engaged)
{
    parking_brake_output_engaged = output_engaged;
}

/**
 * Get whether the parking brake is enabled in config
 */
bool mcpwm_dc_is_parking_brake_enabled(void)
{
    return conf->dc_enable_parking_brake;
}

/**
 * Get whether the parking brake is engaged currently
 */
bool mcpwm_dc_is_parking_brake_engaged(void)
{
    return parking_brake_output_engaged;
}

/**
 * Get the currently set duty cycle
 */
float mcpwm_dc_get_parking_brake_duty(void)
{
    return dutycycle_parking_brake_now;
}

/**
 * Some getters to debug the parking brake module
 */
uint16_t mcpwm_dc_get_ccr_called(void)
{
    return is_ccr_called;
}
uint32_t mcpwm_reg_get_CCR2(void)
{
    return TIM1->CCR2;
}
uint32_t mcpwm_reg_get_ARR(void)
{
    return TIM1->ARR;
}
uint32_t mcpwm_reg_get_CCER(void)
{
    return TIM1->CCER;
}
uint32_t mcpwm_reg_get_CCMR1(void)
{
    return TIM1->CCMR1;
}
uint32_t mcpwm_reg_get_BDTR(void)
{
    return TIM1->BDTR;
}