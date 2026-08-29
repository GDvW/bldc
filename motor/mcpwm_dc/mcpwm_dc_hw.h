#ifndef MCPWM_DC_HW_H
#define MCPWM_DC_HW_H

// Settings for the timers
// PWM
/**
 * Counter:
 * .......................  top/ARR
 *      /|    /|    /|    
 *     / |   / |   / |   /
 *    /  |  /  |  /  |  / 
 * ../...|./...|./...|./..  CCR/duty
 *  /    |/    |/    |/   
 * -----------------------
 * PWM signal
 * .......................  
 *  _     _     _     _   
 * | |   | |   | |   | |  
 * | |   | |   | |   | |  
 * | |___| |___| |___| |__
 * -----------------------
 */
typedef struct
{
    volatile bool updated; // If true, settings will be applied next time update_timer_attempt is called. Otherwise ignored
    volatile unsigned int top; 
    volatile unsigned int duty_motor;
    volatile unsigned int duty_brake;
    volatile unsigned int val_sample; // Point where the voltage is sampled. by setting curr_samp_volt, it can be set whether specific phase currents should also be measured at this moment
    volatile unsigned int curr1_sample; // Points in the PWM cycle where the current is sampled if not overriden by curr_samp_volt
    volatile unsigned int curr2_sample;
#ifdef HW_HAS_3_SHUNTS
    volatile unsigned int curr3_sample;
#endif
} mc_timer_struct;

extern volatile mc_timer_struct timer_struct;

void mcpwm_dc_init_hw(void);
void mcpwm_dc_deinit_hw(void);
void set_next_timer_settings(mc_timer_struct *settings);
void update_timer_attempt(void);
void stop_pwm_hw(void);
void update_adc_sample_pos(mc_timer_struct *t);
void do_dc_cal(void);
void set_switching_frequency(float frequency);
void set_dutycycle_hw(float dutycycle);
void set_direction_hw(void);

#endif