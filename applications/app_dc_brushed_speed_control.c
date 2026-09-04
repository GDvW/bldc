/**
 * Custom app to control speed on a brushed dc motor
 *
 * Needs:
 * - Check if the current motor type is brushed dc for safety -> via mc_interface
 * - voltage and current measurements -> via callback to allow custom filtering.
 * - RPM setpoint -> normal get
 * - Current dutycycle setpoint/current setpoint -> normal api
 * - If current control method is speed based -> via modified get_control_mode function
 *
 * Sets:
 * - Measured RPM
 * - Duty cycle or current
 *
 * I will add another file to the driver because setting current/duty from outside while not switching off speed control is important
 *
 * REMARK: nicest is to have rpm keeping updated even though no speed control is selected
 */
#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_dc/mcpwm_dc_app_interface.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define MOTOR_RESISTANCE 73 // Ohms
#define MOTOR_CONSTANT 0.01 // V/RPM

// Threads
static THD_FUNCTION(rpm_thread, arg);
static THD_WORKING_AREA(rpm_thread_wa, 1024);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// For current filtering
static volatile float current_filtered;
static volatile float rpm_now;

// Private functions
static void after_measurement_taken(void);
static void run_pid_control_speed(float dt);

/**
 * Called when the application is started
 * Set a callback
 */
void app_custom_sc_start(void)
{
    current_filtered = 0;

    mcpwm_dc_app_set_measurement_callback(after_measurement_taken);

    stop_now = false;
    chThdCreateStatic(rpm_thread_wa, sizeof(rpm_thread_wa),
                      NORMALPRIO, rpm_thread, NULL);
}

/**
 * Called when the application is stopped
 * Release callback.
 */
void app_custom_sc_stop(void)
{
    mcpwm_dc_app_set_measurement_callback(0);

    stop_now = true;
    while (is_running)
    {
        chThdSleepMilliseconds(1);
    }
}

/**
 * Set the app configuration.
 *
 * @param conf the app configuration
 */
void app_custom_sc_configure(app_configuration *conf)
{
    (void)conf;
}

static THD_FUNCTION(rpm_thread, arg)
{
    (void)arg;

    chRegSetThreadName("Speed control app");
    is_running = true;

    for (;;)
    {
        // Check if it is time to stop.
        if (stop_now)
        {
            is_running = false;
            return;
        }

        run_pid_control_speed(0.0005);

        timeout_reset(); // Reset timeout if everything is OK.

        chThdSleepMicroseconds(500); // Run at approximately 2kHz
    }
}

static void after_measurement_taken(void)
{
    // Filter the current
    // UTILS_LP_FAST(current_filtered, , 0.061); // 1st order IIR filter with fc = 100 Hz

    current_filtered = mc_interface_get_tot_current_directional();
    const float duty_now = mc_interface_get_duty_cycle_now();

    // Get the input voltage
    // const float motor_voltage = mc_interface_get_duty_cycle_now() * mc_interface_get_input_voltage_filtered();
    const float motor_voltage = duty_now * GET_INPUT_VOLTAGE();

    float bemf = motor_voltage - fabsf(current_filtered) * MOTOR_RESISTANCE;
    // if (bemf < 0.0){
    //     bemf = 0.0;
    // }

    rpm_now = SIGN(duty_now) * bemf / MOTOR_CONSTANT;
    mcpwm_dc_app_set_rpm_now(rpm_now);
}

static void run_pid_control_speed(float dt)
{
    static float i_term = 0;
    static float prev_error = 0;
    float p_term;
    float d_term;

    // Check if DC brushed motor is configured, because this app is only meant for that type motor
    const volatile mc_configuration *conf = mc_interface_get_configuration();

    if (conf->motor_type != MOTOR_TYPE_DC)
    {
        return;
    }

    const float duty_now = mc_interface_get_duty_cycle_now();
    const float rpm_set = mcpwm_dc_app_get_rpm_set();

    // PID is off. Return.
    if (!mcpwm_dc_app_is_speed_control_active())
    {
        i_term = duty_now;

        prev_error = 0.0;
        return;
    }

    float error = rpm_set - rpm_now;

    // Too low RPM set. Stop and return.
    if (fabsf(rpm_set) < conf->s_pid_min_erpm)
    {
        i_term = duty_now;
        prev_error = error;
        mcpwm_dc_app_set_duty(0.0);
        return;
    }

    // Compensation for supply voltage variations
    float scale = 1.0 / mc_interface_get_input_voltage_filtered();

    // Compute parameters
    p_term = error * conf->s_pid_kp * scale;
    i_term += error * conf->s_pid_ki * dt * scale;
    d_term = (error - prev_error) * (conf->s_pid_kd / dt) * scale;

    // Filter D
    static float d_filter = 0.0;
    UTILS_LP_FAST(d_filter, d_term, conf->s_pid_kd_filter);
    d_term = d_filter;

    // I-term wind-up protection
    utils_truncate_number(&i_term, -1.0, 1.0);

    // Store previous error
    prev_error = error;

    // Calculate output
    float duty_new = p_term + i_term + d_term;

    // Make sure that at least minimum output is used
    if (fabsf(duty_new) < conf->l_min_duty)
    {
        duty_new = SIGN(duty_new) * conf->l_min_duty;
    }

    // Do not output in reverse direction to oppose too high rpm
    if (rpm_set > 0.0 && duty_new < 0.0)
    {
        duty_new = conf->l_min_duty;
        i_term = 0.0;
    }
    else if (rpm_set < 0.0 && duty_new > 0.0)
    {
        duty_new = -conf->l_min_duty;
        i_term = 0.0;
    }

    mcpwm_dc_app_set_duty(duty_new);
}
