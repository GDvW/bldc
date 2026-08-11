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

#include "mcpwm_dc_locals.h"

/**
 * Initializes all local variables to a default state
 */
void mcpwm_dc_init_locals()
{
    control_mode = CONTROL_MODE_NONE;
    state = MC_STATE_OFF;
    direction = DIRECTION_FORWARD;
    // use_regular_adc = false;
    dutycycle_set = 0.0;
    current_set = 0.0;
    rpm_set = 0.0;
    dutycycle_now = 0.0;
    // h_bridge_active = false;
    // slow_ramping_cycles = 0;
    dccal_done = false;
    switching_frequency_now = conf->m_dc_f_sw;
    last_current_sample = 0.0;
    last_current_sample_filtered = 0.0;

    filter_create_fir_lowpass((float *)current_fir_coeffs, CURR_FIR_FCUT, CURR_FIR_TAPS_BITS, 1);

    // Create current FIR filter
    filter_create_fir_lowpass((float *)current_fir_coeffs, CURR_FIR_FCUT, CURR_FIR_TAPS_BITS, 1);

    // Create amplitude FIR filter
    filter_create_fir_lowpass((float *)amp_fir_coeffs, AMP_FIR_FCUT, AMP_FIR_TAPS_BITS, 1);
}

volatile mc_control_mode control_mode;
volatile float dutycycle_set;
volatile float current_set;
volatile float dutycycle_now;

// Filters
// Current FIR filter
volatile float current_fir_coeffs[CURR_FIR_LEN];
volatile float current_fir_samples[CURR_FIR_LEN];
volatile int current_fir_index = 0;

// Amplitude FIR filter
volatile float amp_fir_coeffs[AMP_FIR_LEN];
volatile float amp_fir_samples[AMP_FIR_LEN];
volatile int amp_fir_index = 0;