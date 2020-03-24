
/*
    custom_machine_template.h
    Part of Grbl_ESP32

    copyright (c) 2020 -	Bart Dring. This file was intended for use on the ESP32
                        CPU. Do not use this with Grbl for atMega328P

    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Grbl_ESP32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

    ====================================================================

    See custom_machine_templete.cpp for getting started creating custom
    machines

*/
// =============== CPU MAP ========================
// Look at cpu_map.h for all the things that can go here

#define MACHINE_NAME "CPU_MAP_MASLOW_V2"

#define CUSTOM_CODE_FILENAME "Custom/maslow_v2.cpp"

#define DEFAULT_SPINDLE_FREQ 2000.0
#define DEFAULT_X_MAX_TRAVEL 100.0

#ifdef N_AXIS
    #undef N_AXIS
#endif
#define N_AXIS 5

#define NUM_TLC59711 1
#define TLC_DATA   5
#define TLC_CLOCK  21

#define RSENSE 10000
#define MOTOR_1_ADC ADC1_GPIO33_CHANNEL
#define MOTOR_1_FORWARD 1
#define MOTOR_1_BACKWARD 0
#define MOTOR_1_CS 17
#define MOTOR_2_ADC ADC1_GPIO34_CHANNEL
#define MOTOR_2_FORWARD 3
#define MOTOR_2_BACKWARD 2
#define MOTOR_2_CS 3
#define MOTOR_3_ADC ADC1_GPIO36_CHANNEL
#define MOTOR_3_FORWARD 5
#define MOTOR_3_BACKWARD 4
#define MOTOR_3_CS 22
#define MOTOR_4_ADC ADC1_GPIO35_CHANNEL
#define MOTOR_4_FORWARD 7
#define MOTOR_4_BACKWARD 6
#define MOTOR_4_CS 25
#define MOTOR_5_ADC ADC1_GPIO32_CHANNEL
#define MOTOR_5_FORWARD 9
#define MOTOR_5_BACKWARD 8
#define MOTOR_5_CS 13

#define DC_TOP_LEFT 0
#define DC_TOP_RIGHT 1
#define DC_BOTTOM_LEFT 2
#define DC_BOTTOM_RIGHT 3
#define DC_Z_AXIS 4

#define DC_TOP_LEFT_STEPS_PER_MM 10 //.1mm per step is required resolution
#define DC_TOP_RIGHT_STEPS_PER_MM 10
#define DC_BOTTOM_LEFT_STEPS_PER_MM 10
#define DC_BOTTOM_RIGHT_STEPS_PER_MM 10
#define DC_Z_AXIS_STEPS_PER_MM 100 //

#define USE_PIDCONTROL
#define USE_MACHINE_INIT
// ================ Custom function Prototypes ======================
#ifndef maslow_v2_h
    #define maslow_v2_h

    #include "memory"
    #include "TLC59711.h"
    #include "MotorUnit.h"
    #include "driver/adc.h"
    #include "esp_adc_cal.h"

    void compute_pid();
    void dc_motor_step(uint8_t step_mask, uint8_t dir_mask);
    void update_setpoints(float setpoint_1,
                            float setpoint_2,
                            float setpoint_3,
                            float setpoint_4,
                            float setpoint_5);
    void update_pid_tunes(float new_p,
                            float new_i,
                            float new_d);

    void update_control_mode(pid_mode new_mode);

#endif
