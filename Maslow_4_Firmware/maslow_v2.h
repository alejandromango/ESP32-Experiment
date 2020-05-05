
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


// Set the distance of the point where the cord connects to the sled to the bit
#define X_TL_OFFSET 0 // mm
#define X_TR_OFFSET 0 // mm
#define X_BL_OFFSET 0 // mm
#define X_BR_OFFSET 0 // mm

#define Y_TL_OFFSET 0 // mm
#define Y_TR_OFFSET 0 // mm
#define Y_BL_OFFSET 0 // mm
#define Y_BR_OFFSET 0 // mm

#define DC_TOP_LEFT 0 //X_AXIS
#define DC_TOP_RIGHT 2 //Z_AXIS
#define DC_BOTTOM_LEFT 3 //A_AXIS
#define DC_BOTTOM_RIGHT 4 //B_AXIS
#define DC_Z_AXIS 1 //Y_AXIS

#define DC_TOP_LEFT_STEPS_PER_MM 10 //.1mm per step is required resolution
#define DC_TOP_RIGHT_STEPS_PER_MM 10
#define DC_BOTTOM_LEFT_STEPS_PER_MM 10
#define DC_BOTTOM_RIGHT_STEPS_PER_MM 10
#define DC_Z_AXIS_STEPS_PER_MM 10 //

#define DC_TOP_LEFT_MAX_RATE 100.0 // mm/min
#define DC_TOP_RIGHT_MAX_RATE 100.0 // mm/min
#define DC_BOTTOM_LEFT_MAX_RATE 100.0 // mm/min
#define DC_BOTTOM_RIGHT_MAX_RATE 100.0 // mm/min
#define DC_Z_AXIS_MAX_RATE 100.0 // mm/min

#define DC_TOP_LEFT_MM_PER_REV -29.92//31.415 // Correct for 10mm pulley
#define DC_TOP_RIGHT_MM_PER_REV 29.92//31.415
#define DC_BOTTOM_LEFT_MM_PER_REV 29.92//31.415
#define DC_BOTTOM_RIGHT_MM_PER_REV -29.92//31.415
#define DC_Z_AXIS_MM_PER_REV 29.92//31.415 //

// Override defaults

#define DEFAULT_X_STEPS_PER_MM DC_TOP_LEFT_STEPS_PER_MM
#define DEFAULT_Y_STEPS_PER_MM DC_TOP_RIGHT_STEPS_PER_MM
#define DEFAULT_Z_STEPS_PER_MM DC_Z_AXIS_STEPS_PER_MM
#define DEFAULT_A_STEPS_PER_MM DC_BOTTOM_LEFT_STEPS_PER_MM
#define DEFAULT_B_STEPS_PER_MM DC_BOTTOM_RIGHT_STEPS_PER_MM

#define DEFAULT_X_MAX_RATE DC_TOP_LEFT_MAX_RATE // mm/min
#define DEFAULT_Y_MAX_RATE DC_TOP_RIGHT_MAX_RATE // mm/min
#define DEFAULT_Z_MAX_RATE DC_Z_AXIS_MAX_RATE // mm/min
#define DEFAULT_A_MAX_RATE DC_BOTTOM_LEFT_MAX_RATE // mm/min
#define DEFAULT_B_MAX_RATE DC_BOTTOM_RIGHT_MAX_RATE // mm/min

#define USE_PIDCONTROL
// #define USE_KINEMATICS
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
    void pid_step(uint8_t step_mask, uint8_t dir_mask);
    void pid_get_state();
    void motor_stop();
    void update_setpoints(double setpoint_1,
                            double setpoint_2,
                            double setpoint_3,
                            double setpoint_4,
                            double setpoint_5);
    bool machine_regulation();
    void print_setpoints();
    void update_pid_tunes(double new_p,
                            double new_i,
                            double new_d);

    void update_control_mode(pid_mode new_mode);

// **** Start of "extra" includes to transfer from grbl ****

    // Called if USE_MACHINE_INIT is defined
void machine_init();

// Called if USE_CUSTOM_HOMING is defined
bool user_defined_homing();

// Called if USE_KINEMATICS is defined
// void inverse_kinematics(float* target, plan_line_data_t* pl_data, float* position);
bool kinematics_pre_homing(uint8_t cycle_mask);
void kinematics_post_homing();

// Called if USE_FWD_KINEMATIC is defined
// void forward_kinematics(float* position);

// Called if MACRO_BUTTON_0_PIN or MACRO_BUTTON_1_PIN or MACRO_BUTTON_2_PIN is defined
void user_defined_macro(uint8_t index);

// Called if USE_M30 is defined
void user_m30();

// Called if USE_TOOL_CHANGE is defined
void user_tool_change(uint8_t new_tool);


// **** End of "extra" includes to transfer from grbl ****

#endif
