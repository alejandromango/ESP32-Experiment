#ifndef maslow_v2_h
#define maslow_v2_h

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

class machine{
public:
    void machine_init();
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

    std::unique_ptr<TLC59711> tlc;
    std::unique_ptr<MotorUnit> motor1;
    std::unique_ptr<MotorUnit> motor2;
    std::unique_ptr<MotorUnit> motor3;
    std::unique_ptr<MotorUnit> motor4;
    std::unique_ptr<MotorUnit> motor5;


};

#endif