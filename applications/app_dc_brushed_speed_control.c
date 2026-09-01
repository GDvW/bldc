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

#include <math.h>
#include <string.h>
#include <stdio.h>

/**
 * Called when the application is started
 * Set a callback
 */
void app_custom_sc_start(void)
{
}

/**
 * Called when the application is stopped
 * Release callback.
 */
void app_custom_sc_stop(void)
{
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
