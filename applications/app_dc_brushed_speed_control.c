/**
 * Custom app to control speed on a brushed dc motor
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
