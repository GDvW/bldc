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

    Motor controller for DC brushed motors.
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

// Structs
typedef struct
{
    // If updated is false, it will applied the next time update_timer_attempt is run
    // Else it will be ignored
    volatile bool updated;
    volatile unsigned int top;
    volatile unsigned int duty_motor;
    volatile unsigned int val_sample;
    volatile unsigned int curr1_sample;
    volatile unsigned int curr2_sample;
#ifdef HW_HAS_3_SHUNTS
    volatile unsigned int curr3_sample;
#endif
} mc_timer_struct;

// Struct with IIR filter states
// This is intended as 3 1st order IIR filters
// State will go from state1 to state2 to state3
// New state for single order IIR filter is alpha * state + (1 - alpha) * input
typedef struct
{
    float alpha;
    float state1;
    float state2;
    float state3;
} mc_dc_filter_struct;

// Private variables
// 0 for negative values, 1 for positive values of duty, current and rpm
static volatile int direction;
// Use the voltage-synchronized samples for this current sample
// Regular ADC -> ADC from DMA buffer
// Injected ADC -> triggered by timer
static volatile bool use_regular_adc;
// Are signed
static volatile float dutycycle_set;
static volatile float dutycycle_now;

static volatile float speed_pid_set_rpm;
static volatile float current_set;
static volatile float rpm_now;
static volatile bool h_bridge_active;

// Not used, but can be used to limit transients
static volatile unsigned int slow_ramping_cycles;

// Current measurement parameters
static volatile bool dccal_done; // Calibration done
static volatile int curr_start_samples;
static volatile int curr0_sum;
static volatile int curr1_sum;
static volatile int curr0_offset;
static volatile int curr1_offset;
#ifdef HW_HAS_3_SHUNTS
static volatile int curr2_sum;
static volatile int curr2_offset;
#endif

static volatile float switching_frequency_now;
static volatile float last_current_sample;
static volatile float last_current_sample_filtered;

static volatile float last_adc_isr_duration;
static volatile float last_inj_adc_isr_duration;
static volatile float pll_phase;
static volatile float pll_speed;

static volatile mc_configuration *conf;

// Control mode of device, e.g. speed, current or none
static volatile mc_control_mode control_mode;
// State of device: running, full_brake or none
static volatile mc_state state;
// Used to set the timers
static volatile mc_timer_struct timer_struct;

static volatile bool init_done = false;

// Current FIR filter
#define CURR_FIR_TAPS_BITS 4
#define CURR_FIR_LEN (1 << CURR_FIR_TAPS_BITS)
#define CURR_FIR_FCUT 0.15
static volatile float current_fir_coeffs[CURR_FIR_LEN];
static volatile float current_fir_samples[CURR_FIR_LEN];
static volatile int current_fir_index = 0;

// Amplitude FIR filter
#define AMP_FIR_TAPS_BITS 7
#define AMP_FIR_LEN (1 << AMP_FIR_TAPS_BITS)
#define AMP_FIR_FCUT 0.02
static volatile float amp_fir_coeffs[AMP_FIR_LEN];
static volatile float amp_fir_samples[AMP_FIR_LEN];
static volatile int amp_fir_index = 0;

// Owned by mcpwm_dc_adc_inj_int_handler
static mc_dc_filter_struct current_dc_filter;

// Speed low pass vars
static volatile float alpha_rpm;
static volatile float rpm_unfiltered;

// Ripple (speed) detection vars
static volatile float ripple_frequency;

// RPM thread
static THD_WORKING_AREA(speed_thread_wa, 512);
static THD_FUNCTION(speed_pid_thread, arg);
static volatile bool speed_pid_thd_stop;

// Private functions
static void set_duty_cycle_hl(float dutycycle);
static void set_duty_cycle_ll(float dutycycle);
static void set_duty_cycle_hw(float dutycycle);
static void stop_pwm_motor_ll(void);
static void stop_pwm_ll(void);
static void stop_pwm_motor_hw(void);
static void do_dc_cal(void);
static float do_dc_current_filtering(float current_sample);

static void set_next_timer_settings(mc_timer_struct *settings);
static void update_adc_sample_pos(mc_timer_struct *timer_tmp);
static void set_direction_hw(void);
static void update_timer_attempt(void);
static void set_switching_frequency(float frequency);
static void full_brake_hw(void);
static void full_brake_ll(void);
static float ripple_to_rpm(float frequency);
static void run_pid_control_speed(float dt);

// Initializes the motor controller
void mcpwm_dc_init(volatile mc_configuration *configuration)
{
    mcpwm_dc_parking_brake_init(configuration);

    utils_sys_lock_cnt();

    init_done = false;

    conf = configuration;

    // Set defaults
    control_mode = CONTROL_MODE_NONE;
    state = MC_STATE_OFF;
    direction = 1;
    use_regular_adc = false;
    dutycycle_set = 0.0;
    dutycycle_now = 0.0;
    speed_pid_set_rpm = 0.0;
    current_set = 0.0;
    rpm_now = 0.0;
    h_bridge_active = false;
    slow_ramping_cycles = 0;
    dccal_done = false;
    switching_frequency_now = conf->m_dc_f_sw;
    last_current_sample = 0.0;
    last_current_sample_filtered = 0.0;
    pll_phase = 0.0;
    pll_speed = 1.0;
    ripple_frequency = 0;

    // current dc filter init
    alpha_rpm = expf(-2.0f * (float)M_PI * conf->rpm_filter_cutoff / conf->m_dc_f_sw);
    current_dc_filter.alpha = expf(-2.0f * (float)M_PI * conf->current_filter_cutoff_pll / conf->m_dc_f_sw);
    current_dc_filter.state1 = 0;
    current_dc_filter.state2 = 0;
    current_dc_filter.state3 = 0;

    // Initialize the parking brake

    // Create current FIR filter
    filter_create_fir_lowpass((float *)current_fir_coeffs, CURR_FIR_FCUT, CURR_FIR_TAPS_BITS, 1);

    // Create amplitude FIR filter
    filter_create_fir_lowpass((float *)amp_fir_coeffs, AMP_FIR_FCUT, AMP_FIR_TAPS_BITS, 1);

    // Initialize clocks
    // TIM1 is used to generate PWM signals
    // TIM8 and TIM1->CC4 is used for ADC sampling

    // Create structs for configuration of clocks
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

    // Make sure clocks are in a known state
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM8);
    TIM1->CNT = 0;
    TIM8->CNT = 0;

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
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; // gpio high = bottom fets on
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

    // --- Structure declarations for ADC, DMA, and common ADC settings ---
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // --- Enable clocks for the peripherals used ---
    // GPIOA, GPIOC → analog input pins
    // DMA2 → required for ADC multi-mode (CDR register is only on DMA2)
    // These all live on the AHB1 bus.
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 |
                               RCC_AHB1Periph_GPIOA |
                               RCC_AHB1Periph_GPIOC,
                           ENABLE);

    // ADC1/2/3 live on the APB2 bus.
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |
                               RCC_APB2Periph_ADC2 |
                               RCC_APB2Periph_ADC3,
                           ENABLE);

    // --- Allocate DMA stream for ADC triple-mode ---
    // DMA2 Stream 4, Channel 0 is the ONLY stream that can read ADC->CDR.
    // Priority 5, ISR handler = mcpwm_dc_adc_int_handler.
    dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),
                      5,
                      (stm32_dmaisr_t)mcpwm_dc_adc_int_handler,
                      (void *)0);

    // --- DMA configuration for ADC triple-mode ---
    // DMA reads from ADC->CDR (combined data register for ADC1+2+3)
    // and writes into ADC_Value[] buffer.
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;                              // ADC triple-mode uses Channel 0
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;               // Destination buffer
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;             // Source = combined ADC data
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     // ADC → RAM
    DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;                         // Number of samples per cycle
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            // CDR is a single register
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // Increment buffer pointer
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16-bit ADC values
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;        // Continuous sampling
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;    // ADC timing is critical
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; // Direct mode
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream4, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream4, ENABLE);                 // Start DMA
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE); // Interrupt on buffer full

    // --- ADC Common Init ---
    // Triple regular simultaneous mode: ADC1, ADC2, ADC3 sample at the same time.
    // Prescaler /2 → ADC clock = 42 MHz (slightly above datasheet max, but VESC uses it).
    ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; // 12+12+12 bits packed into CDR
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // --- ADC channel configuration (shared for ADC1/2/3) ---
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 0–4095
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;           // Multiple channels
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    // Triggered by timer
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1; // TIM8 CC1 triggers ADC1
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV; // Number of regular channels

    // ADC1 uses external trigger
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC2/3 do NOT use external trigger (they follow ADC1 in triple mode)
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = 0;
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_Init(ADC3, &ADC_InitStructure);

    // Enable internal temperature sensor + Vrefint
    ADC_TempSensorVrefintCmd(ENABLE);

    // Enable DMA request after last transfer in triple mode
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

    // --- Injected channels (sampled at end of PWM cycle for current measurement) ---
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T8_CC2);
#ifdef HW_HAS_3_SHUNTS
    ADC_ExternalTrigInjectedConvConfig(ADC3, ADC_ExternalTrigInjecConv_T8_CC3);
#endif

    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Falling);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
#ifdef HW_HAS_3_SHUNTS
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC3, ADC_ExternalTrigInjecConvEdge_Falling);
#endif

    // Injected sequence length = number of shunt currents
    ADC_InjectedSequencerLengthConfig(ADC1, HW_ADC_INJ_CHANNELS);
    ADC_InjectedSequencerLengthConfig(ADC2, HW_ADC_INJ_CHANNELS);
#ifdef HW_HAS_3_SHUNTS
    ADC_InjectedSequencerLengthConfig(ADC3, HW_ADC_INJ_CHANNELS);
#endif

    // Configure the actual ADC channels (board-specific)
    hw_setup_adc_channels();

    // Enable JEOC interrupt (end of injected conversion)
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    nvicEnableVector(ADC_IRQn, 6);

    // Enable all three ADCs
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC3, ENABLE);

    // --- Timer 8 setup (drives ADC sampling) ---
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // Free-running counter
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    // Configure CC1/2/3 as PWM outputs to generate ADC triggers
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 500; // Trigger timing
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_CCPreloadControl(TIM8, ENABLE);

    // PWM outputs must be enabled for CCx events to fire
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    // --- TIM1 master → TIM8 slave ---
    // TIM1 update event resets TIM8, synchronizing ADC sampling with PWM cycle.
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
    TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

    // Enable TIM1 and TIM8
    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

    // Main Output Enable
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // ADC sampling locations
    stop_pwm_motor_hw();
    mc_timer_struct timer_tmp;
    timer_tmp.top = TIM1->ARR;
    timer_tmp.duty_motor = TIM1->ARR / 2;
    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);

    utils_sys_unlock_cnt();

    CURRENT_FILTER_ON();
    CURRENT_FILTER_ON_M2();

    // Calibrate current offset
    ENABLE_GATE();
    DCCAL_OFF();
    do_dc_cal();

    // Start rpm thread
    speed_pid_thd_stop = false;
    chThdCreateStatic(speed_thread_wa, sizeof(speed_thread_wa), NORMALPRIO, speed_pid_thread, NULL);

    // Check if the system has resumed from IWDG reset
    if (timeout_had_IWDG_reset())
    {
        mc_interface_fault_stop(FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET, false, false);
    }

    init_done = true;
}

void mcpwm_dc_deinit(void)
{
    if (!init_done)
    {
        return;
    }

    init_done = false;

    speed_pid_thd_stop = true;

    while (speed_pid_thd_stop)
    {
        chThdSleepMilliseconds(1);
    }

    TIM_DeInit(TIM1);
    TIM_DeInit(TIM8);
    ADC_DeInit();
    DMA_DeInit(DMA2_Stream4);
    nvicDisableVector(ADC_IRQn);
    dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
}

static THD_FUNCTION(speed_pid_thread, arg)
{
    (void)arg;

    chRegSetThreadName("dc speed pid");
    uint32_t last_time = timer_time_now();

    for (;;)
    {
        if (speed_pid_thd_stop)
        {
            speed_pid_thd_stop = false;
            return;
        }

        switch (conf->sp_pid_loop_rate)
        {
        case PID_RATE_25_HZ:
            chThdSleepMicroseconds(1000000 / 25);
            break;
        case PID_RATE_50_HZ:
            chThdSleepMicroseconds(1000000 / 50);
            break;
        case PID_RATE_100_HZ:
            chThdSleepMicroseconds(1000000 / 100);
            break;
        case PID_RATE_250_HZ:
            chThdSleepMicroseconds(1000000 / 250);
            break;
        case PID_RATE_500_HZ:
            chThdSleepMicroseconds(1000000 / 500);
            break;
        case PID_RATE_1000_HZ:
            chThdSleepMicroseconds(1000000 / 1000);
            break;
        case PID_RATE_2500_HZ:
            chThdSleepMicroseconds(1000000 / 2500);
            break;
        case PID_RATE_5000_HZ:
            chThdSleepMicroseconds(1000000 / 5000);
            break;
        case PID_RATE_10000_HZ:
            chThdSleepMicroseconds(1000000 / 10000);
            break;
        }

        float dt = timer_seconds_elapsed_since(last_time);
        last_time = timer_time_now();

        run_pid_control_speed(dt);
    }
}

static void run_pid_control_speed(float dt)
{
    static float i_term = 0;
    static float p_term = 0;
    static float d_term = 0;
    static float d_term_filter = 0;
    static float prev_error = 0;

    // PID is off. Return.
    if (control_mode != CONTROL_MODE_SPEED)
    {
        i_term = 0.0;
        prev_error = 0.0;
        return;
    }

    float rpm = mcpwm_dc_get_rpm();
    float error = speed_pid_set_rpm - rpm;

    // Too low RPM set. Reset state, release motor and return.
    if (fabsf(speed_pid_set_rpm) < conf->s_pid_min_erpm)
    {
        i_term = 0.0;
        prev_error = error;
        current_set = 0;
        return;
    }

    // PID with proper dt
    // Apply  * (1.0 / 20.0) to normalize. 
    // FOC uses these values, so to reuse the values from the config, also do that here
    p_term = error * conf->s_pid_kp * (1.0 / 20.0);
    i_term += error * conf->s_pid_ki * dt * (1.0 / 20.0);
    utils_truncate_number_abs(&i_term, 1.0);

    d_term = (error - prev_error) * (conf->s_pid_kd / dt) * (1.0 / 20.0);
    // Filter D
    UTILS_LP_FAST(d_term_filter, d_term, conf->s_pid_kd_filter);
    d_term = d_term_filter;

    // Store previous error
    prev_error = error;

    // Calculate output
    float output = p_term + i_term + d_term;
    utils_truncate_number_abs(&output, 1.0);

    // Integrator windup protection

    if (conf->s_pid_ki < 1e-9)
    {
        i_term = 0.0;
    }

    // Optionally disable braking
    if (!conf->s_pid_allow_braking)
    {
        if (rpm > 20.0 && output < 0.0)
        {
            output = 0.0;
        }

        if (rpm < -20.0 && output > 0.0)
        {
            output = 0.0;
        }
    }

    float current_set_tmp = output * conf->lo_current_max * conf->l_current_max_scale;

    utils_truncate_number(&current_set_tmp, conf->lo_current_min, conf->lo_current_max);

    current_set = current_set_tmp;

    if (state != MC_STATE_RUNNING) {  
        set_duty_cycle_hl(SIGN(output) * conf->l_min_duty);  
    }
}

bool mcpwm_dc_init_done(void)
{
    return init_done;
}

void mcpwm_dc_set_configuration(volatile mc_configuration *configuration)
{
    // Stop everything first to be safe
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_ll();

    mcpwm_dc_parking_brake_set_configuration(configuration);

    utils_sys_lock_cnt();
    conf = configuration;
    utils_sys_unlock_cnt();
}

/**
 * Does the dc calculation. This is used to remove the voltage level when there is no current flowing
 */
static void do_dc_cal(void)
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
    curr0_sum = 0;
    curr1_sum = 0;

#ifdef HW_HAS_3_SHUNTS
    curr2_sum = 0;
#endif

    curr_start_samples = 0;
    // The currents are measured by mcpwm_dc_adc_inj_int_handler and the timer is incremented
    // each time measured (which is each PWM cycle). So this loop ends some time
    while (curr_start_samples < 4000)
    {
    };
    curr0_offset = curr0_sum / curr_start_samples;
    curr1_offset = curr1_sum / curr_start_samples;

#ifdef HW_HAS_3_SHUNTS
    curr2_offset = curr2_sum / curr_start_samples;
#endif

    DCCAL_OFF();
    dccal_done = true;
}

// Functions used in mcpwm_dc_adc_inj_int_handler
static inline void read_currents_raw(float *c0, float *c1, float *c2)
{
    if (use_regular_adc)
    {
        *c0 = GET_CURRENT1();
        *c1 = GET_CURRENT2();
#ifdef HW_HAS_3_SHUNTS
        *c2 = GET_CURRENT3();
#endif
    }
    else
    {
        *c0 = HW_GET_INJ_CURR1();
        *c1 = HW_GET_INJ_CURR2();
#ifdef HW_HAS_3_SHUNTS
        *c2 = HW_GET_INJ_CURR3();
#endif

#ifdef INVERTED_SHUNT_POLARITY
        *c0 = 4095 - *c0;
        *c1 = 4095 - *c1;
#ifdef HW_HAS_3_SHUNTS
        *c2 = 4095 - *c2;
#endif
#endif
    }
}

void mcpwm_dc_adc_inj_int_handler(void)
{
    uint32_t t_start = timer_time_now();

    float curr0, curr1, curr2;

    read_currents_raw(&curr0, &curr1, &curr2);

    curr0_sum += curr0;
    curr1_sum += curr1;
#ifdef HW_HAS_3_SHUNTS
    curr2_sum += curr2;
#endif

    curr_start_samples++;

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

    // Ripple extraction
    float i_dc = do_dc_current_filtering(last_current_sample);

    // Only perform if there is a noticable amount of current
    if (fabsf(i_dc) > conf->min_current_pll_detection)
    {
        float ripple_signal = last_current_sample - i_dc;

        // Phase detector: multiply by quadrature signal
        float phase_error = ripple_signal * sinf(pll_phase);

        float dt = 1.0f / switching_frequency_now;

        float pll_ki_dt = conf->foc_pll_ki * dt;
        float pll_kp_dt = conf->foc_pll_kp * dt;

        // Update PLL
        pll_speed += phase_error * pll_ki_dt;
        pll_phase += pll_speed * dt + phase_error * pll_kp_dt;

        // Normalize phase
        if (pll_phase > M_PI)
            pll_phase -= 2.0 * M_PI;
        if (pll_phase < -M_PI)
            pll_phase += 2.0 * M_PI;

        // Convert PLL speed to frequency (rad/s to Hz)
        ripple_frequency = pll_speed / (2.0 * M_PI);

        // Update RPM
        rpm_unfiltered = ripple_to_rpm(fabsf(ripple_frequency));
        // Perform low pass
        rpm_now = alpha_rpm * rpm_now + (1.0f - alpha_rpm) * rpm_unfiltered;
    }

    last_inj_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}
float get_pll_phase(void)
{
    return pll_phase;
}
float get_pll_speed(void)
{
    return pll_speed;
}
float get_alpha(void)
{
    return current_dc_filter.alpha;
}
float get_i_dc(void)
{
    return current_dc_filter.state3;
}
float get_dt(void)
{
    return 1.0f / switching_frequency_now;
}
float get_rpm_unfiltered(void)
{
    return rpm_unfiltered;
}

float do_dc_current_filtering(float current_sample)
{
    float a = current_dc_filter.alpha;
    current_dc_filter.state1 = a * current_dc_filter.state1 + (1.0f - a) * current_sample;
    current_dc_filter.state2 = a * current_dc_filter.state2 + (1.0f - a) * current_dc_filter.state1;
    current_dc_filter.state3 = a * current_dc_filter.state3 + (1.0f - a) * current_dc_filter.state2;

    return current_dc_filter.state3;
}

/*
 * New ADC samples ready in DMA. Do commutation!
 */
void mcpwm_dc_adc_int_handler(void *p, uint32_t flags)
{
    // Suppress unused parameter warnings
    (void)p;
    (void)flags;

    // Time the execution time of the function
    uint32_t t_start = timer_time_now();

    // Update the timer if possible and needed
    update_timer_attempt();

    // Reset the watchdog
    timeout_feed_WDT(THREAD_MCPWM);

    const float input_voltage = GET_INPUT_VOLTAGE();

    update_duty_cycle_parking_brake();

    // Reset h_bridge_active if direction changed or motor stopped
    static int direction_before = 1;
    if (!(state == MC_STATE_RUNNING && direction == direction_before))
    {
        h_bridge_active = false;
    }
    direction_before = direction;

    // --- DC motor section (amp=amplitude) ---
    float motor_voltage_est = h_bridge_active
                                  ? dutycycle_now * (float)ADC_Value[ADC_IND_VIN_SENS]
                                  : ADC_V_L3 - ADC_V_L1; // BEMF differential before first commutation

    filter_add_sample((float *)amp_fir_samples, motor_voltage_est,
                      AMP_FIR_TAPS_BITS, (uint32_t *)&amp_fir_index);

    if (state == MC_STATE_RUNNING && !h_bridge_active)
    {
        set_direction_hw(); // sets H-bridge direction pins
    }

    const float current_nofilter = mcpwm_dc_get_tot_current();
    const float current_in_nofilter = current_nofilter * fabsf(dutycycle_now);

    // Only create new duty cycles if the motor is running
    // and the FETs are configured correctly
    if (state == MC_STATE_RUNNING && h_bridge_active)
    {
        // Compensation for supply voltage variations
        const float voltage_scale = 20.0 / input_voltage;
        float ramp_step = conf->m_duty_ramp_step / (switching_frequency_now / 1000.0);
        float ramp_step_no_lim = ramp_step;

        if (slow_ramping_cycles)
        {
            slow_ramping_cycles--;
            ramp_step *= 0.1;
        }

        float dutycycle_now_tmp = dutycycle_now;

        if (control_mode == CONTROL_MODE_CURRENT ||
            control_mode == CONTROL_MODE_POS ||
            control_mode == CONTROL_MODE_SPEED)
        {
            // Compute error
            const float error = current_set - (direction ? current_nofilter : -current_nofilter);
            float step = error * conf->cc_gain * voltage_scale;
            const float start_boost = conf->cc_startup_boost_duty * voltage_scale;

            // Do not ramp too much
            utils_truncate_number(&step, -conf->cc_ramp_step_max, conf->cc_ramp_step_max);

            // Switching frequency correction
            step /= switching_frequency_now / 1000.0;

            if (slow_ramping_cycles)
            {
                slow_ramping_cycles--;
                step *= 0.1;
            }

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

            if (slow_ramping_cycles)
            {
                slow_ramping_cycles--;
                step *= 0.1;
            }

            dutycycle_now_tmp += SIGN(dutycycle_now_tmp) * step;

            // Upper truncation
            utils_truncate_number((float *)&dutycycle_now_tmp, -conf->l_max_duty, conf->l_max_duty);

            // Lower truncation
            if (fabsf(dutycycle_now_tmp) < conf->l_min_duty)
            {
                if (fabsf(rpm_now) < conf->l_max_erpm_fbrake_cc)
                {
                    dutycycle_now_tmp = 0.0;
                    dutycycle_set = dutycycle_now_tmp;
                }
                else
                {
                    dutycycle_now_tmp = SIGN(dutycycle_now_tmp) * conf->l_min_duty;
                    dutycycle_set = dutycycle_now_tmp;
                }
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
            utils_step_towards((float *)&dutycycle_now, direction ? conf->l_max_duty : -conf->l_max_duty,
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
            utils_step_towards((float *)&dutycycle_now, direction ? conf->l_max_duty : -conf->l_max_duty,
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

        set_duty_cycle_ll(dutycycle_now);
    }

    mc_interface_mc_timer_isr(false);

    last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}

static void set_duty_cycle_hl(float dutycycle)
{
    utils_truncate_number(&dutycycle, -conf->l_max_duty, conf->l_max_duty);

    dutycycle_set = dutycycle;

    // Start motor if it isnt running, because control loop will only execute if motor is running
    if (state != MC_STATE_RUNNING)
    {
        if (fabsf(dutycycle) >= conf->l_min_duty)
        {
            // dutycycle_now is updated by the back-emf detection. If the motor already
            // is spinning, it will be non-zero.
            if (fabsf(dutycycle_now) < conf->l_min_duty)
            {
                dutycycle_now = SIGN(dutycycle) * conf->l_min_duty;
            }

            set_duty_cycle_ll(dutycycle_now);
        }
        else
        {
            // In case the motor is already spinning, set the state to running
            // so that it can be ramped down before the full brake is applied.
            if (conf->motor_type == MOTOR_TYPE_DC)
            {
                if (fabsf(dutycycle_now) > 0.1)
                {
                    state = MC_STATE_RUNNING;
                }
                else
                {
                    full_brake_ll();
                }
            }
            else
            {
                if (fabsf(rpm_now) > conf->l_max_erpm_fbrake)
                {
                    state = MC_STATE_RUNNING;
                }
                else
                {
                    full_brake_ll();
                }
            }
        }
    }
}

static void set_duty_cycle_ll(float dutycycle)
{
    // Determine the direction
    float dutycycle_abs = fabsf(dutycycle);
    if (dutycycle_abs >= conf->l_min_duty)
    {
        direction = signbit(dutycycle) == 0 ? 1 : 0;
    }

    // Braking condition
    if (dutycycle_abs < conf->l_min_duty)
    {
        // NOTE: Contained checks for maximum speed of motor
        // Removed those because there is no encoder, but could be restored
        full_brake_ll();
        return;
    }

    // Clamp the duty cycle
    if (dutycycle_abs > conf->l_max_duty)
    {
        dutycycle_abs = conf->l_max_duty;
    }

    // Apply
    set_duty_cycle_hw(dutycycle_abs);
    state = MC_STATE_RUNNING;
    set_direction_hw();
}

static void set_duty_cycle_hw(float dutycycle)
{
    mc_timer_struct timer_tmp;

    utils_sys_lock_cnt();
    timer_tmp = timer_struct;
    utils_sys_unlock_cnt();

    utils_truncate_number(&dutycycle, conf->l_min_duty, conf->l_max_duty);

    switching_frequency_now = conf->m_dc_f_sw;
    timer_tmp.top = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
    timer_tmp.duty_motor = (uint16_t)((float)timer_tmp.top * dutycycle);

    set_next_timer_settings(&timer_tmp);
}

static void update_adc_sample_pos(mc_timer_struct *timer_tmp)
{
    volatile uint32_t duty = timer_tmp->duty_motor;
    volatile uint32_t top = timer_tmp->top;
    volatile uint32_t val_sample = timer_tmp->val_sample;
    volatile uint32_t curr1_sample = timer_tmp->curr1_sample;
    volatile uint32_t curr2_sample = timer_tmp->curr2_sample;

#ifdef HW_HAS_3_SHUNTS
    volatile uint32_t curr3_sample = timer_tmp->curr3_sample;
#endif

    if (duty > (uint32_t)((float)top * conf->l_max_duty))
    {
        duty = (uint32_t)((float)top * conf->l_max_duty);
    }

    curr1_sample = top - 10; // Not used anyway
    curr2_sample = top - 10;
#ifdef HW_HAS_3_SHUNTS
    curr3_sample = top - 10;
#endif

    if (duty > 1000)
    {
        val_sample = duty / 2;
        use_regular_adc = false;
    }
    else
    {
        val_sample = duty + 800;
        use_regular_adc = true;
    }

    timer_tmp->val_sample = val_sample;
    timer_tmp->curr1_sample = curr1_sample;
    timer_tmp->curr2_sample = curr2_sample;
#ifdef HW_HAS_3_SHUNTS
    timer_tmp->curr3_sample = curr3_sample;
#endif
}

static void set_next_timer_settings(mc_timer_struct *settings)
{
    utils_sys_lock_cnt();
    timer_struct = *settings;
    timer_struct.updated = false;
    utils_sys_unlock_cnt();

    update_timer_attempt();
}

static void update_timer_attempt(void)
{
    utils_sys_lock_cnt();

    // Set the next timer settings if an update is far enough away
    if (!timer_struct.updated && TIM1->CNT > 10 && TIM1->CNT < (TIM1->ARR - 500))
    {
        // Disable preload register updates
        TIM1->CR1 |= TIM_CR1_UDIS;
        TIM8->CR1 |= TIM_CR1_UDIS;

        // Set the new configuration
        TIM1->ARR = timer_struct.top;
        TIM1->CCR1 = timer_struct.duty_motor;
        TIM1->CCR3 = timer_struct.duty_motor;
        TIM1->CCR4 = timer_struct.curr1_sample;
        TIM8->CCR1 = timer_struct.val_sample;
        TIM8->CCR2 = timer_struct.curr2_sample;
#ifdef HW_HAS_3_SHUNTS
        TIM8->CCR3 = timer_struct.curr3_sample;
#endif

        // Enables preload register updates
        TIM1->CR1 &= ~TIM_CR1_UDIS;
        TIM8->CR1 &= ~TIM_CR1_UDIS;
        timer_struct.updated = true;
    }

    utils_sys_unlock_cnt();
}

static void set_switching_frequency(float frequency)
{
    switching_frequency_now = frequency;
    alpha_rpm = expf(-2.0f * (float)M_PI * conf->rpm_filter_cutoff / frequency);
    current_dc_filter.alpha = expf(-2.0f * (float)M_PI * conf->current_filter_cutoff_pll / frequency);

    mc_timer_struct timer_tmp;

    utils_sys_lock_cnt();
    timer_tmp = timer_struct;
    utils_sys_unlock_cnt();

    timer_tmp.top = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
}

static float ripple_to_rpm(float frequency)
{
    // RPM = (frequency * 60) / commutator_segments
    return (frequency * 60.0) / conf->foc_encoder_ratio;
}

/**
 * Switch off all FETs.
 */
void mcpwm_dc_stop_pwm(void)
{
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_motor_ll();
}
static void stop_pwm_motor_ll(void)
{
    state = MC_STATE_OFF;
    stop_pwm_motor_hw();
}
static void stop_pwm_ll(void)
{
    state = MC_STATE_OFF;
    stop_pwm_motor_hw();
}
static void stop_pwm_motor_hw(void)
{
#ifdef HW_HAS_DRV8313
    DISABLE_BR();
#endif

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

    // if (!conf->dc_enable_parking_brake || parking_channel_included)
    // {
    // 	mcpwm_dc_parking_brake_stop_pwm();
    // }

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    set_switching_frequency(conf->m_dc_f_sw);
}
static void full_brake_ll(void)
{
    state = MC_STATE_FULL_BRAKE;
    full_brake_hw();
}

static void full_brake_hw(void)
{
#ifdef HW_HAS_DRV8313
    ENABLE_BR();
#endif

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

    set_switching_frequency(conf->m_dc_f_sw);
}

void mcpwm_dc_release_motor(void)
{
    current_set = 0.0;
    control_mode = CONTROL_MODE_NONE;
    stop_pwm_motor_ll();
}

/**
 * Sets the direction of the motor by enabling or disabling pins
 */
static void set_direction_hw(void)
{
    if (direction)
    {
        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
    }
    else
    {
        // +
        TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
        TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

        // -
        TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_Inactive);
        TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
        TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
    }

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    h_bridge_active = true;

    mc_timer_struct timer_tmp;

    utils_sys_lock_cnt();
    timer_tmp = timer_struct;
    utils_sys_unlock_cnt();

    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
}

/**
 * Easy setters. Sets the target values for control loops
 */

/**
 * Enables the parking brake if dutyCycle is correct. Bigger than 0 will apply the parking brake
 * Smaller will disengage it.
 *
 * @param dutycycle
 * The parameter on basis of which it is decided to engage/disengage the parking brake
 */
void mcpwm_dc_set_parking_brake_current(float dutycycle)
{
    mcpwm_dc_set_parking_brake(dutycycle > 0.0f);
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_dc_set_brake_current(float current)
{
    if (fabsf(current) < conf->cc_min_current)
    {
        control_mode = CONTROL_MODE_NONE;
        stop_pwm_motor_ll();
        return;
    }

    utils_truncate_number(&current, -fabsf(conf->lo_current_min), fabsf(conf->lo_current_min));

    control_mode = CONTROL_MODE_CURRENT_BRAKE;
    current_set = current;

    if (state != MC_STATE_RUNNING && state != MC_STATE_FULL_BRAKE)
    {
        // In case the motor is already spinning, set the state to running
        // so that it can be ramped down before the full brake is applied.
        if (fabsf(dutycycle_now) > 0.1)
        {
            state = MC_STATE_RUNNING;
        }
        else
        {
            full_brake_ll();
        }
    }
}
/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_dc_set_current(float current)
{
    if (fabsf(current) < conf->cc_min_current)
    {
        control_mode = CONTROL_MODE_NONE;
        stop_pwm_motor_ll();
        return;
    }

    utils_truncate_number(&current, -conf->l_current_max * conf->l_current_max_scale,
                          conf->l_current_max * conf->l_current_max_scale);

    control_mode = CONTROL_MODE_CURRENT;
    current_set = current;

    if (state != MC_STATE_RUNNING)
    {
        set_duty_cycle_hl(SIGN(current) * conf->l_min_duty);
    }
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutycycle
 * The duty cycle to use.
 */
void mcpwm_dc_set_duty(float dutycycle)
{
    control_mode = CONTROL_MODE_DUTY;
    set_duty_cycle_hl(dutycycle);
}

void mcpwm_dc_set_duty_noramp(float dutycycle)
{
    control_mode = CONTROL_MODE_DUTY;

    if (state != MC_STATE_RUNNING)
    {
        set_duty_cycle_hl(dutycycle);
    }
    else
    {
        dutycycle_set = dutycycle;
        dutycycle_now = dutycycle;
        set_duty_cycle_ll(dutycycle);
    }
}

/**
 * Use PID rpm control. Note that this value has to be multiplied by half of
 * the number of motor poles.
 *
 * @param rpm
 * The electrical RPM goal value to use.
 */
void mcpwm_dc_set_pid_speed(float rpm)
{
    control_mode = CONTROL_MODE_SPEED;
    speed_pid_set_rpm = rpm;
}

/**
 * Easy getters. Get target values for control loops or real values.
 */
float mcpwm_dc_get_duty_cycle_set(void)
{
    return dutycycle_set;
}

float mcpwm_dc_get_duty_cycle_now(void)
{
    return dutycycle_now;
}

float mcpwm_dc_get_last_adc_isr_duration(void)
{
    return last_adc_isr_duration;
}

float mcpwm_dc_get_rpm(void)
{
    return direction ? rpm_now : -rpm_now;
}

mc_state mcpwm_dc_get_state(void)
{
    return state;
}

float mcpwm_dc_get_switching_frequency_now(void)
{
    return switching_frequency_now;
}

/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current.
 *
 * @return
 * The motor current.
 */
float mcpwm_dc_get_tot_current(void)
{
    return last_current_sample;
}

/**
 * Get the FIR-filtered motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_dc_get_tot_current_filtered(void)
{
    return last_current_sample_filtered;
}

/**
 * Get the motor current. The sign of this value represents the direction
 * in which the motor generates torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_dc_get_tot_current_directional(void)
{
    const float retval = mcpwm_dc_get_tot_current();
    return dutycycle_now > 0.0 ? retval : -retval;
}

/**
 * Get the filtered motor current. The sign of this value represents the
 * direction in which the motor generates torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_dc_get_tot_current_directional_filtered(void)
{
    const float retval = mcpwm_dc_get_tot_current_filtered();
    return dutycycle_now > 0.0 ? retval : -retval;
}

/**
 * Get the input current to the motor controller.
 *
 * @return
 * The input current.
 */
float mcpwm_dc_get_tot_current_in(void)
{
    return mcpwm_dc_get_tot_current() * fabsf(dutycycle_now);
}

/**
 * Get the FIR-filtered input current to the motor controller.
 *
 * @return
 * The filtered input current.
 */
float mcpwm_dc_get_tot_current_in_filtered(void)
{
    return mcpwm_dc_get_tot_current_filtered() * fabsf(dutycycle_now);
}

/**
 * Returns true if dccal is done. Dccal is the process to remove the standard dc offset in the current measurements
 */
bool mcpwm_dc_is_dccal_done(void)
{
    return dccal_done;
}
