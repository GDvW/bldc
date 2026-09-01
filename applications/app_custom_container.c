/**
 * Custom app to control speed on a brushless dc motor
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

#define MCPWM_DC_DEBUG
#include "app_custom_debug.c"
#include "app_dc_brushed_speed_control.c"

/**
 * Called when the application is started
 * Set a callback
 */
void app_custom_start(void)
{
    app_custom_debug_start();
    app_custom_sc_start();
}

/**
 * Called when the application is stopped
 * Release callback.
 */
void app_custom_stop(void)
{
    app_custom_debug_stop();
    app_custom_sc_stop();
}

/**
 * Set the app configuration.
 *
 * @param conf the app configuration
 */
void app_custom_configure(app_configuration *conf)
{
    (void)conf;
    app_custom_sc_configure(conf);
}
