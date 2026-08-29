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

void run_control_loop(void)
{
    const float input_voltage = GET_INPUT_VOLTAGE();
    const float current_nofilter = mcpwm_dc_get_tot_current();
    const float current_in_nofilter = current_nofilter * fabsf(dutycycle_now);

    // Compensation for supply voltage variations
    const float voltage_scale = 20.0 / input_voltage;
    float ramp_step = conf->m_duty_ramp_step / (switching_frequency_now / 1000.0);
    float ramp_step_no_lim = ramp_step;

    float dutycycle_now_tmp = dutycycle_now;

    // Speed and pos have both their own control loop, but that only works through setting the current, so they share the same control loop
    if (control_mode == CONTROL_MODE_CURRENT ||
        control_mode == CONTROL_MODE_POS ||
        control_mode == CONTROL_MODE_SPEED)
    {
        // Compute error
        const float error = current_set - (direction == DIRECTION_FORWARD ? current_nofilter : -current_nofilter);
        float step = error * conf->cc_gain * voltage_scale;
        const float start_boost = conf->cc_startup_boost_duty * voltage_scale;

        // Do not ramp too much
        utils_truncate_number(&step, -conf->cc_ramp_step_max, conf->cc_ramp_step_max);

        // Switching frequency correction
        step /= switching_frequency_now / 1000.0;

        // Optionally apply startup boost.
        if (fabsf(dutycycle_now_tmp) < start_boost)
        {
            utils_step_towards(&dutycycle_now_tmp,
                               current_set > 0.0 ? start_boost : -start_boost, ramp_step);
        }
        else
        {
            dutycycle_now_tmp += step;
        }

        // Upper truncation
        utils_truncate_number((float *)&dutycycle_now_tmp, -conf->l_max_duty, conf->l_max_duty);

        // Lower truncation
        if (fabsf(dutycycle_now_tmp) < conf->l_min_duty)
        {
            if (dutycycle_now_tmp < 0.0 && current_set > 0.0)
            {
                dutycycle_now_tmp = conf->l_min_duty;
            }
            else if (dutycycle_now_tmp > 0.0 && current_set < 0.0)
            {
                dutycycle_now_tmp = -conf->l_min_duty;
            }
        }

        // The set dutycycle should be in the correct direction in case the output is lower
        // than the minimum duty cycle and the mechanism below gets activated.
        dutycycle_set = dutycycle_now_tmp >= 0.0 ? conf->l_min_duty : -conf->l_min_duty;
    }
    else if (control_mode == CONTROL_MODE_CURRENT_BRAKE)
    {
        // Compute error
        const float error = -fabsf(current_set) - current_nofilter;
        float step = error * conf->cc_gain * voltage_scale;

        // Do not ramp too much
        utils_truncate_number(&step, -conf->cc_ramp_step_max, conf->cc_ramp_step_max);

        // Switching frequency correction
        step /= switching_frequency_now / 1000.0;

        dutycycle_now_tmp += SIGN(dutycycle_now_tmp) * step;

        // Upper truncation
        utils_truncate_number((float *)&dutycycle_now_tmp, -conf->l_max_duty, conf->l_max_duty);

        // Lower truncation
        if (fabsf(dutycycle_now_tmp) < conf->l_min_duty)
        {
            dutycycle_now_tmp = 0.0;
            dutycycle_set = dutycycle_now_tmp;
        }
    }
    else
    {
        utils_step_towards((float *)&dutycycle_now_tmp, dutycycle_set, ramp_step);
    }

    static int limit_delay = 0;

    // Apply limits in priority order
    if (current_nofilter > conf->lo_current_max)
    {
        utils_step_towards((float *)&dutycycle_now, 0.0,
                           ramp_step_no_lim * fabsf(current_nofilter - conf->lo_current_max) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_nofilter < conf->lo_current_min)
    {
        utils_step_towards((float *)&dutycycle_now, direction == DIRECTION_FORWARD ? conf->l_max_duty : -conf->l_max_duty,
                           ramp_step_no_lim * fabsf(current_nofilter - conf->lo_current_min) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_in_nofilter > conf->lo_in_current_max)
    {
        utils_step_towards((float *)&dutycycle_now, 0.0,
                           ramp_step_no_lim * fabsf(current_in_nofilter - conf->lo_in_current_max) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }
    else if (current_in_nofilter < conf->lo_in_current_min)
    {
        utils_step_towards((float *)&dutycycle_now, direction == DIRECTION_FORWARD ? conf->l_max_duty : -conf->l_max_duty,
                           ramp_step_no_lim * fabsf(current_in_nofilter - conf->lo_in_current_min) * conf->m_current_backoff_gain);
        limit_delay = 1;
    }

    if (limit_delay > 0)
    {
        limit_delay--;
    }
    else
    {
        dutycycle_now = dutycycle_now_tmp;
    }

    // When the set duty cycle is in the opposite direction, make sure that the motor
    // starts again after stopping completely
    if (fabsf(dutycycle_now) < conf->l_min_duty)
    {
        if (dutycycle_set >= conf->l_min_duty)
        {
            dutycycle_now = conf->l_min_duty;
        }
        else if (dutycycle_set <= -conf->l_min_duty)
        {
            dutycycle_now = -conf->l_min_duty;
        }
    }

    mcpwm_dc_set_duty_direct_ll(dutycycle_now);
}