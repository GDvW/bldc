# Documentation about the mcwpm_dc driver

The main idea of the driver is that getters/setters set flags or states, and that one control loop takes care of them.

## Debugging

### Send on stop command

Age    : 5.83 s
Thread : USB process
Motor  : 1
Command: set_current
Param  : 0.000

Age    : 5.84 s
Thread : USB process
Motor  : 1
Command: release_motor_override
Param  : 0.000

Age    : 4.83 s
Thread : Timeout
Motor  : 1
Command: release_motor_override
Param  : 0.000

Age    : 0.01 s
Thread : Timeout
Motor  : 1
Command: set_current_brake
Param  : 0.000

### Needs to start

Age    : 51.32 s
Thread : USB process
Motor  : 1
Command: set_current
Param  : 0.000

Age    : 51.32 s
Thread : USB process
Motor  : 1
Command: release_motor_override
Param  : 0.000

Age    : 50.32 s
Thread : Timeout
Motor  : 1
Command: release_motor_override
Param  : 0.000

Age    : 10.85 s
Thread : Timeout
Motor  : 1
Command: set_current_brake
Param  : 0.000

Age    : 10.85 s
Thread : USB process
Motor  : 1
Command: set_duty
Param  : 0.000

Age    : 2.06 s
Thread : USB process
Motor  : 1
Command: set_duty
Param  : 0.200

### The `debug` command

-> debug (setting duty->50% after a STOP command, does not work)

DEBUG:
  Motor type: 1
  State: 2
  Control mode: 0
  H bridge: 0
  duty set: 0.500
  duty now: 0.500
  tot_current: 0.064
  tot_current_filtered: -0.006
  tot_current_directional: 0.064
  tot_current_directional_filtered: -0.006
  tot_current_in: 0.032
  tot_current_in_filtered: -0.003
  switching_frequency_now: 25000.000
  init_done: 1
  dccal_done: 1
  Latest ADC duration: 0.0104 ms
  Latest injected ADC duration: 0.0189 ms
  Timer:
    Top: 6720
    Duty motor: 3360
    Duty brake: 3360
    val_sample: 672
    curr1_sample: 6710
    curr2_sample: 1680
    curr3_sample: 6710
  Measurement:
    curr_adc_source_mask: 5
    curr_start_samples: 10706256
    curr0_sum: 2147483647
    curr1_sum: 2147483647
    curr2_sum: 2147483647
    curr0_offset: 2047
    curr1_offset: 2046
    curr2_offset: 2047
  Parking brake:
    Duty cycle: 0.000
    Current: 0.032 A
    Current filtered: 0.041 A
    State: 0
    Engaged: 0
    was_parking_h_bridge_updated: 0
    has_parking_h_bridge_been_updated: 0
  Timers:
    TIM1 ARR: 6720
    TIM1 CCR1: 3360
    TIM1 CCR2: 3360
    TIM1 CCR3: 3360
    TIM1 CCR4: 6710
    TIM8 CCR1: 672
    TIM8 CCR2: 1680
    TIM8 CCR3: 6710
    TIM1 CNT: 6204
-> debug (setting 0% duty)

DEBUG:
  Motor type: 1
  State: 3
  Control mode: 0
  H bridge: 0
  duty set: 0.000
  duty now: 0.005
  tot_current: 0.064
  tot_current_filtered: 0.061
  tot_current_directional: 0.064
  tot_current_directional_filtered: 0.061
  tot_current_in: 0.000
  tot_current_in_filtered: 0.000
  switching_frequency_now: 25000.000
  init_done: 1
  dccal_done: 1
  Latest ADC duration: 0.0054 ms
  Latest injected ADC duration: 0.0139 ms
  Timer:
    Top: 6720
    Duty motor: 37
    Duty brake: 3360
    val_sample: 672
    curr1_sample: 6710
    curr2_sample: 1680
    curr3_sample: 6710
  Measurement:
    curr_adc_source_mask: 5
    curr_start_samples: 11324769
    curr0_sum: 2147483647
    curr1_sum: 2147483647
    curr2_sum: 2147483647
    curr0_offset: 2047
    curr1_offset: 2046
    curr2_offset: 2047
  Parking brake:
    Duty cycle: 0.000
    Current: 0.032 A
    Current filtered: 0.000 A
    State: 0
    Engaged: 0
    was_parking_h_bridge_updated: 0
    has_parking_h_bridge_been_updated: 0
  Timers:
    TIM1 ARR: 6720
    TIM1 CCR1: 37
    TIM1 CCR2: 3360
    TIM1 CCR3: 37
    TIM1 CCR4: 6710
    TIM8 CCR1: 672
    TIM8 CCR2: 1680
    TIM8 CCR3: 6710
    TIM1 CNT: 3886
-> debug (Setting 50% duty, works)

DEBUG:
  Motor type: 1
  State: 2
  Control mode: 0
  H bridge: 0
  duty set: 0.500
  duty now: 0.500
  tot_current: 0.032
  tot_current_filtered: 0.002
  tot_current_directional: 0.032
  tot_current_directional_filtered: 0.002
  tot_current_in: 0.016
  tot_current_in_filtered: 0.001
  switching_frequency_now: 25000.000
  init_done: 1
  dccal_done: 1
  Latest ADC duration: 0.0104 ms
  Latest injected ADC duration: 0.0189 ms
  Timer:
    Top: 6720
    Duty motor: 3360
    Duty brake: 3360
    val_sample: 837
    curr1_sample: 6710
    curr2_sample: 1680
    curr3_sample: 6710
  Measurement:
    curr_adc_source_mask: 5
    curr_start_samples: 11750270
    curr0_sum: 2147483647
    curr1_sum: 2147483647
    curr2_sum: 2147483647
    curr0_offset: 2047
    curr1_offset: 2046
    curr2_offset: 2047
  Parking brake:
    Duty cycle: 0.000
    Current: 0.064 A
    Current filtered: 0.049 A
    State: 0
    Engaged: 0
    was_parking_h_bridge_updated: 0
    has_parking_h_bridge_been_updated: 0
  Timers:
    TIM1 ARR: 6720
    TIM1 CCR1: 3360
    TIM1 CCR2: 3360
    TIM1 CCR3: 3360
    TIM1 CCR4: 6710
    TIM8 CCR1: 837
    TIM8 CCR2: 1680
    TIM8 CCR3: 6710
    TIM1 CNT: 5524
-> debug (After pressing STOP)

DEBUG:
  Motor type: 1
  State: 0
  Control mode: 10
  H bridge: 0
  duty set: 0.500
  duty now: 0.500
  tot_current: -0.000
  tot_current_filtered: -0.017
  tot_current_directional: -0.000
  tot_current_directional_filtered: -0.017
  tot_current_in: 0.000
  tot_current_in_filtered: -0.008
  switching_frequency_now: 25000.000
  init_done: 1
  dccal_done: 1
  Latest ADC duration: 0.0067 ms
  Latest injected ADC duration: 0.0139 ms
  Timer:
    Top: 6720
    Duty motor: 3360
    Duty brake: 3360
    val_sample: 837
    curr1_sample: 6710
    curr2_sample: 1680
    curr3_sample: 6710
  Measurement:
    curr_adc_source_mask: 5
    curr_start_samples: 11844247
    curr0_sum: 2147483647
    curr1_sum: 2147483647
    curr2_sum: 2147483647
    curr0_offset: 2047
    curr1_offset: 2046
    curr2_offset: 2047
  Parking brake:
    Duty cycle: 0.000
    Current: 0.032 A
    Current filtered: 0.031 A
    State: 0
    Engaged: 0
    was_parking_h_bridge_updated: 0
    has_parking_h_bridge_been_updated: 0
  Timers:
    TIM1 ARR: 6720
    TIM1 CCR1: 3360
    TIM1 CCR2: 3360
    TIM1 CCR3: 3360
    TIM1 CCR4: 6710
    TIM8 CCR1: 837
    TIM8 CCR2: 1680
    TIM8 CCR3: 6710
    TIM1 CNT: 5918