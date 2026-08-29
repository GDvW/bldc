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

static bool update_h_bridge(void);

void mcpwm_dc_init_hw()
{
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
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;         // Continuous sampling
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; // ADC timing is critical
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // Direct mode
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
    stop_pwm_hw();
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
}

void mcpwm_dc_deinit_hw(void)
{
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM8);
    ADC_DeInit();
    DMA_DeInit(DMA2_Stream4);
    nvicDisableVector(ADC_IRQn);
    dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
}

//  Updates timer_struct from the passed settings
void set_next_timer_settings(mc_timer_struct *settings)
{
    utils_sys_lock_cnt();
    timer_struct = *settings;
    timer_struct.updated = false;
    utils_sys_unlock_cnt();

    update_timer_attempt();
}

// Applies the timer settings set in timer_struct
void update_timer_attempt(void)
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
        TIM1->CCR2 = timer_struct.duty_brake;
        TIM1->CCR3 = timer_struct.duty_motor;
        TIM8->CCR1 = timer_struct.val_sample;
        TIM1->CCR4 = timer_struct.curr1_sample;
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

void set_switching_frequency(float frequency)
{
    switching_frequency_now = frequency;

    mc_timer_struct timer_tmp;

    utils_sys_lock_cnt();
    timer_tmp = timer_struct;
    utils_sys_unlock_cnt();

    timer_tmp.top = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
}

// Stop all pwm on all gates
void stop_pwm_hw(void)
{
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
// Stop all pwm on the motor
void stop_pwm_motor_hw(void)
{
#ifdef HW_HAS_DRV8313
    DISABLE_BR();
#endif

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

    TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
}

/**
 * Sets the direction of the motor by enabling or disabling pins
 */
void set_direction_hw(void)
{
    if (direction == DIRECTION_FORWARD)
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

    mc_timer_struct timer_tmp;

    utils_sys_lock_cnt();
    timer_tmp = timer_struct;
    utils_sys_unlock_cnt();

    update_adc_sample_pos(&timer_tmp);
    set_next_timer_settings(&timer_tmp);
}

void full_brake_hw(void)
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

/**
 * Directly set the duty cycle on the hardware
 */
void set_dutycycle_hw(float dutycycle)
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


void mcpwm_dc_adc_inj_int_handler(void)
{
    // Start timer for time keeping
    uint32_t t_start = timer_time_now();

    // Do current measuring
    float curr0, curr1, curr2;
    read_currents_raw(&curr0, &curr1, &curr2);
    process_current_measurements(curr0, curr1, curr2);

    last_adc_inj_isr_duration = timer_seconds_elapsed_since(t_start);
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

    // Update the timer settings (pwm and adc timing) if possible and needed
    update_timer_attempt();

    // Reset the watchdog
    timeout_feed_WDT(THREAD_MCPWM);
    
    // Update the h bridge to match the currently defined direction
    // This will only be done if the current state is running
    const bool h_bridge_updated = update_h_bridge();
    
    take_motor_voltage_measurement(h_bridge_updated);

    // Only create new duty cycles if the motor is running
    // and the FETs are configured correctly.
    // (If they have been configured this iteration, measurements are not correct, so wait till the next time)
    if (state == MC_STATE_RUNNING && !h_bridge_updated)
    {
        run_control_loop();
    }

    float dt = 1.0 / switching_frequency_now;
    mc_interface_mc_timer_isr(false, dt);

    last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
}

/**
 * Updates the H-bridge configuration so that the polarity is correct
 * Returns true if the H-bridge configuration was updated
 */
static bool update_h_bridge(void)
{
    static bool was_running = false;
    static direction_t direction_before = DIRECTION_FORWARD;

    const bool running = state == MC_STATE_RUNNING;
    // Update needed in case we are running now and
    // - direction has switched (H-bridge has to switch polarity)
    // - We are just starting. Other functions have configured the H-bridge in an invalid way
    const bool needs_update =
        running &&
        (!was_running || direction != direction_before);

    if (needs_update)
    {
        set_direction_hw();
    }

    was_running = running;
    direction_before = direction;

    return needs_update;
}