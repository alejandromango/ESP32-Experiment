
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

// ================ config.h overrides ========================================
/*

If you want to make some changes to config.h, it might be easier to do it here
so all your changes are in your files.

example to change baud rate
#ifdef BAUD_RATE
	#undef BAUDRATE
#endif
#define BAUD_RATE 9600

example to change the number of axes
#idef N_AXIS
	#undef N_AXIS
#endif
#define N_AXIS 4


*/


// =============== CPU MAP ========================
// Look at cpu_map.h for all the things that can go here


#define CPU_MAP_NAME 		"CPU_MAP_MASLOW_V2"

#define LIMIT_MASK      	B111 // you need this with as many switches you are using

// ============== Enable custom features =======================

// #define #USE_MACHINE_INIT
// #define USE_CUSTOM_HOMING
// #define USE_KINEMATICS
// #define USE_FWD_KINEMATIC
// #define USE_TOOL_CHANGE
// #define USE_M30
// #define USE_MACHINE_TRINAMIC_INIT

// ===================== $$ Defaults ==========================================
/* 	These are default values for any of the $$ settings.
	This will automatically set them when you upload new firmware or if you
 	reset them with $RST=$.
	All default values are set in the defaults.h file. You would only need to
	put values here that are different from those values
	Below are a few examples
*/
#define DEFAULT_SPINDLE_FREQ 2000.0
#define DEFAULT_X_MAX_TRAVEL 100.0



#ifndef maslow_v2_h
#define maslow_v2_h

// Includes from template
// #include "grbl.h"

// Custom includes and defines
#include "TLC59711.h"
#include "MotorUnit.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define NUM_TLC59711 1
#define TLC_CLOCK   5
#define TLC_DATA  21

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

// ================ Function Prototypes from template ================
void machine_init();
// bool user_defined_homing();
// void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position);
// void forward_kinematics(float *position);
// void kinematics_post_homing();
// void user_tool_change(uint8_t new_tool);
// void user_defined_macro(uint8_t index);
// void user_m30();
// void machine_trinamic_setup();

// ================ Custom function Prototypes ======================
void compute_pid();
void update_setpoints(float setpoint_1,
                        float setpoint_2,
                        float setpoint_3,
                        float setpoint_4,
                        float setpoint_5);
void update_pid_tunes(float new_p,
                        float new_i,
                        float new_d);

void update_control_mode(mode new_mode);


extern std::unique_ptr<TLC59711> tlc;
extern std::unique_ptr<MotorUnit> motor1;
extern std::unique_ptr<MotorUnit> motor2;
extern std::unique_ptr<MotorUnit> motor3;
extern std::unique_ptr<MotorUnit> motor4;
extern std::unique_ptr<MotorUnit> motor5;

#endif