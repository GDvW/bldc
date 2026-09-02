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

#define MOTOR_RESISTANCE 73 //Ohms
#define MOTOR_CONSTANT 0.01 // V/RPM

// Threads
static THD_FUNCTION(rpm_thread, arg);
static THD_WORKING_AREA(rpm_thread_wa, 1024);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

// For current filtering
static volatile float current_filtered;

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

        timeout_reset(); // Reset timeout if everything is OK.

        chThdSleepMilliseconds(10); // Run at approximately
    }
}

static void after_measurement_taken(void)
{
    // Filter the current
    UTILS_LP_FAST(current_filtered, mc_interface_get_tot_current_directional(), 0.061); // 1st order IIR filter with fc = 100 Hz

    // Get the input voltage
    const float motor_voltage = mc_interface_get_duty_cycle_now() * GET_INPUT_VOLTAGE();

    const float bemf = motor_voltage - current_filtered * MOTOR_RESISTANCE;
    const float rpm = bemf / MOTOR_CONSTANT;
    mcpwm_dc_app_set_current_rpm(rpm);
}
