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
    const float pb_duty_set = PARKING_BRAKE_DUTY;
    const float pb_current_set = PARKING_BRAKE_CURRENT;
    const mc_control_mode pb_control_mode = PARKING_BRAKE_CONTROL_MODE;

    const float input_voltage = GET_INPUT_VOLTAGE();
    const float current_nofilter = mcpwm_dc_get_tot_pb_current();
    const float current_in_nofilter = mcpwm_dc_get_tot_current_in();

    // Compensation for supply voltage variations
    const float voltage_scale = 20.0 / input_voltage;
    float ramp_step = conf->m_duty_ramp_step / (switching_frequency_now / 1000.0);
    float ramp_step_no_lim = ramp_step;

    float duty_now_tmp = duty_now_parking_brake;

    if (pb_control_mode == CONTROL_MODE_CURRENT)
    {
        // Compute error
        const float error = pb_current_set - current_nofilter;
        float step = error * conf->cc_gain * voltage_scale;
        const float start_boost = conf->cc_startup_boost_duty * voltage_scale;

        // Do not ramp too much
        utils_truncate_number(&step, -conf->cc_ramp_step_max, conf->cc_ramp_step_max);

        // Switching frequency correction
        step /= switching_frequency_now / 1000.0;

        // Optionally apply startup boost.
        if (duty_now_tmp < start_boost)
        {
            utils_step_towards(&duty_now_tmp, start_boost, ramp_step);
        }
        else
        {
            duty_now_tmp += step;
        }

        // Truncation
        utils_truncate_number(&duty_now_tmp, conf->l_min_duty, conf->l_max_duty);
    }
    else
    {
        utils_step_towards(&duty_now_tmp, pb_duty_set, ramp_step);
    }

    static int limit_delay = 0;

    // Apply limits in priority order
    if (current_nofilter > conf->lo_current_max)
    {
        utils_step_towards((float *)&duty_now_parking_brake, 0.0,
                           ramp_step_no_lim * (current_nofilter - conf->lo_current_max) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_nofilter < conf->lo_current_min)
    {
        utils_step_towards((float *)&duty_now_parking_brake, conf->l_max_duty,
                           ramp_step_no_lim * (conf->lo_current_min - current_nofilter) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_in_nofilter > conf->lo_in_current_max)
    {
        utils_step_towards((float *)&duty_now_parking_brake, 0.0,
                           ramp_step_no_lim * (current_in_nofilter - conf->lo_in_current_max) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_in_nofilter < conf->lo_in_current_min)
    {
        utils_step_towards((float *)&duty_now_parking_brake, conf->l_max_duty,
                           ramp_step_no_lim * (conf->lo_in_current_min - current_in_nofilter) * conf->m_current_backoff_gain);
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

    set_dutycycle_parking_brake_hw(duty_now_parking_brake);
}