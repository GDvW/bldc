/**
 * Contains all stuff related to taking current/voltage measurements etc
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

static volatile int curr_start_samples; 
static volatile int curr0_sum;
static volatile int curr1_sum;
static volatile int curr0_offset;
static volatile int curr1_offset;
#ifdef HW_HAS_3_SHUNTS
static volatile int curr2_sum;
static volatile int curr2_offset;
#endif

// Calculate the dc levels in the current measurement when the motor is off
// To compensate for this when running
void do_dc_cal(void)
{
    DCCAL_ON();

    // Wait max 5 seconds
    int cnt = 0;
    while (IS_DRV_FAULT())
    {
        chThdSleepMilliseconds(1);
        cnt++;
        if (cnt > 5000)
        {
            break;
        }
    };

    chThdSleepMilliseconds(1000);
    // Reset all current measurement counters
    curr0_sum = 0;
    curr1_sum = 0;

#ifdef HW_HAS_3_SHUNTS
    curr2_sum = 0;
#endif

    // Wait till 4000 samples have been taken
    // This is populated by process_current_measurements via mcpwm_dc_adc_inj_int_handler
    curr_start_samples = 0;
    while (curr_start_samples < 4000)
    {
    };

    // Average it out
    curr0_offset = curr0_sum / curr_start_samples;
    curr1_offset = curr1_sum / curr_start_samples;

#ifdef HW_HAS_3_SHUNTS
    curr2_offset = curr2_sum / curr_start_samples;
#endif

    DCCAL_OFF();
    dccal_done = true;
}

/** Reads the curernts based on the flag set in curr_adc_source_mask
 * If the first bit is set to 0, curr0 is taken at the moment specified in the timer
 * Else it is taken at the same moment as the voltage
 */
void read_currents_raw(int curr_adc_source_mask, float *curr0, float *curr1, float *curr2)
{
    *curr0 = HW_GET_INJ_CURR1();
    *curr1 = HW_GET_INJ_CURR2();
#ifdef HW_HAS_3_SHUNTS
    *curr2 = HW_GET_INJ_CURR3();
#endif

#ifdef INVERTED_SHUNT_POLARITY
    *curr0 = 4095 - *curr0;
    *curr1 = 4095 - *curr1;
#ifdef HW_HAS_3_SHUNTS
    *curr2 = 4095 - *curr2;
#endif
#endif

    if (curr_adc_source_mask & (1 << 0))
    {
        *curr0 = GET_CURRENT1();
    }

    if (curr_adc_source_mask & (1 << 1))
    {
        *curr1 = GET_CURRENT2();
    }

#ifdef HW_HAS_3_SHUNTS
    if (curr_adc_source_mask & (1 << 2))
    {
        *curr2 = GET_CURRENT3();
    }
#endif
}

/**
 * Does everything for the current measurement processing 
 * (storing for the dc_cal, correcting for the dc level, etc)
 */
void process_current_measurements(float curr0, float curr1, float curr2){
    // Store results for the dc_cal
    curr_start_samples++;
    curr0_sum += curr0;
    curr1_sum += curr1;
#ifdef HW_HAS_3_SHUNTS
    curr2_sum += curr2;
#endif

    // Compensate for the dc level in the measured signal
    curr0 -= curr0_offset;
    curr1 -= curr1_offset;
#ifdef HW_HAS_3_SHUNTS
    curr2 -= curr2_offset;
#endif

    // Store raw ADC readings for raw sampling mode.
    ADC_curr_raw[0] = curr0;
    ADC_curr_raw[1] = curr1;
#ifdef HW_HAS_3_SHUNTS
    ADC_curr_raw[2] = curr2;
#endif

    // Scale to AMPs using calibrated scaling factors
    curr0 *= FAC_CURRENT1;
    curr1 *= FAC_CURRENT2;
#ifdef HW_HAS_3_SHUNTS
    curr2 *= FAC_CURRENT3;
#else
    curr2 = -(curr0 + curr1);
#endif

    // Store the currents for sampling
    ADC_curr_norm_value[0] = curr0;
    ADC_curr_norm_value[1] = curr1;
    ADC_curr_norm_value[2] = curr2;

    // Calculate the total motor current and filter it
    float curr_tot_sample = 0;
    if (direction)
    {
#ifdef HW_HAS_3_SHUNTS
        curr_tot_sample = -(GET_CURRENT3() - curr2_offset) * FAC_CURRENT3;
#else
        curr_tot_sample = -(GET_CURRENT2() - curr1_offset) * FAC_CURRENT2;
#endif
    }
    else
    {
        curr_tot_sample = -(GET_CURRENT1() - curr0_offset) * FAC_CURRENT1;
    }

    last_current_sample = curr_tot_sample;

    // Filter out outliers
    if (fabsf(last_current_sample) > (conf->l_abs_current_max * 1.2))
    {
        last_current_sample = SIGN(last_current_sample) * conf->l_abs_current_max * 1.2;
    }

    filter_add_sample((float *)current_fir_samples, last_current_sample,
                      CURR_FIR_TAPS_BITS, (uint32_t *)&current_fir_index);
    last_current_sample_filtered = filter_run_fir_iteration(
        (float *)current_fir_samples, (float *)current_fir_coeffs,
        CURR_FIR_TAPS_BITS, current_fir_index);
}
