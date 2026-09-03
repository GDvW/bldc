/**
 * Houses the main control loop
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

#include "mcpwm_dc.h"
#include "mcpwm_dc_locals.h"
#include "mcpwm_dc_hw.h"

void run_parking_brake_control_loop(void)
{
    const float p_duty_set = PARKING_BRAKE_DUTY;
    const float p_current_set = PARKING_BRAKE_DUTY;
    const mc_control_mode p_control_mode = PARKING_BRAKE_CONTROL_MODE;

    const float input_voltage = GET_INPUT_VOLTAGE();

    // Compensation for supply voltage variations
    const float voltage_scale = 20.0 / input_voltage;
    float ramp_step = conf->m_duty_ramp_step / (switching_frequency_now / 1000.0);
    float ramp_step_no_lim = ramp_step;

    float duty_now_tmp = duty_now_parking_brake;

    if (p_control_mode == CONTROL_MODE_CURRENT)
    {
    }
    else
    {
        utils_step_towards((float *)&duty_now_tmp, p_duty_set, ramp_step);
    }

    static int limit_delay = 0;

    // Apply limits in priority order
    if (current_nofilter > conf->lo_current_max)
    {
        utils_step_towards((float *)&duty_now_parking_brake, 0.0,
                           ramp_step_no_lim * fabsf(current_nofilter - conf->lo_current_max) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_nofilter < conf->lo_current_min)
    {
        utils_step_towards((float *)&duty_now_parking_brake, conf->l_max_duty,
                           ramp_step_no_lim * fabsf(current_nofilter - conf->lo_current_min) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_in_nofilter > conf->lo_in_current_max)
    {
        utils_step_towards((float *)&duty_now_parking_brake, 0.0,
                           ramp_step_no_lim * fabsf(current_in_nofilter - conf->lo_in_current_max) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_in_nofilter < conf->lo_in_current_min)
    {
        utils_step_towards((float *)&duty_now_parking_brake, conf->l_max_duty,
                           ramp_step_no_lim * fabsf(current_in_nofilter - conf->lo_in_current_min) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }

    if (limit_delay > 0)
    {
        limit_delay--;
    }
    else
    {
        duty_now_parking_brake = duty_now_tmp;
    }

    // When the set duty cycle is in the opposite direction, make sure that the motor
    // starts again after stopping completely
    if (fabsf(duty_now_parking_brake) < conf->l_min_duty)
    {
        if (p_duty_set >= conf->l_min_duty)
        {
            duty_now_parking_brake = conf->l_min_duty;
        }
    }

    set_dutycycle_parking_brake_hw(duty_now_parking_brake);
}