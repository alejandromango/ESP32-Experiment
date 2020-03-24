#include "maslow_v2.h"
#include "TLC59711.h"
#include "MotorUnit.h"
esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
esp_err_t config_err_1 = adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_2 = adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_3 = adc1_config_channel_atten(ADC1_GPIO36_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_4 = adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_5 = adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_2_5);
TLC59711 tlc(NUM_TLC59711, TLC_CLOCK, TLC_DATA); // MASLOW_TODO: Will this get garbage collected?
MotorUnit motor1(&tlc, MOTOR_1_FORWARD, MOTOR_1_BACKWARD, MOTOR_1_ADC, RSENSE, adc_1_characterisitics, MOTOR_1_CS);
MotorUnit motor2(&tlc, MOTOR_2_FORWARD, MOTOR_2_BACKWARD, MOTOR_2_ADC, RSENSE, adc_1_characterisitics, MOTOR_2_CS);
MotorUnit motor3(&tlc, MOTOR_3_FORWARD, MOTOR_3_BACKWARD, MOTOR_3_ADC, RSENSE, adc_1_characterisitics, MOTOR_3_CS);
MotorUnit motor4(&tlc, MOTOR_4_FORWARD, MOTOR_4_BACKWARD, MOTOR_4_ADC, RSENSE, adc_1_characterisitics, MOTOR_4_CS);
MotorUnit motor5(&tlc, MOTOR_5_FORWARD, MOTOR_5_BACKWARD, MOTOR_5_ADC, RSENSE, adc_1_characterisitics, MOTOR_5_CS);



void machine_init(){
    tlc.begin();
    tlc.write();
    Serial.println("machine setup complete");

}

void dc_motor_step(uint8_t step_mask, uint8_t dir_mask){
    motor1.step(step_mask & (1<<DC_TOP_LEFT), dir_mask & (1<<DC_TOP_LEFT), 1.0/DC_TOP_LEFT_STEPS_PER_MM);
    motor2.step(step_mask & (1<<DC_TOP_RIGHT), dir_mask & (1<<DC_TOP_RIGHT), 1.0/DC_TOP_RIGHT_STEPS_PER_MM);
    motor3.step(step_mask & (1<<DC_BOTTOM_LEFT), dir_mask & (1<<DC_BOTTOM_LEFT), 1.0/DC_BOTTOM_LEFT_STEPS_PER_MM);
    motor4.step(step_mask & (1<<DC_BOTTOM_RIGHT), dir_mask & (1<<DC_BOTTOM_RIGHT), 1.0/DC_BOTTOM_RIGHT_STEPS_PER_MM);
    motor5.step(step_mask & (1<<DC_Z_AXIS), dir_mask & (1<<DC_Z_AXIS), 1.0/DC_Z_AXIS_STEPS_PER_MM);
}

void compute_pid(){
    motor1.computePID();
    motor2.computePID();
    motor3.computePID();
    motor4.computePID();
    motor5.computePID();
}

void update_setpoints(float setpoint_1,
                               float setpoint_2,
                               float setpoint_3,
                               float setpoint_4,
                               float setpoint_5){
    motor1.setSetpoint(setpoint_1);
    motor2.setSetpoint(setpoint_2);
    motor3.setSetpoint(setpoint_3);
    motor4.setSetpoint(setpoint_4);
    motor5.setSetpoint(setpoint_5);
}
void update_pid_tunes(float new_p, float new_i, float new_d){
    motor1.setPIDTune(new_p, new_i, new_d);
    motor2.setPIDTune(new_p, new_i, new_d);
    motor3.setPIDTune(new_p, new_i, new_d);
    motor4.setPIDTune(new_p, new_i, new_d);
    motor5.setPIDTune(new_p, new_i, new_d);
}

void update_control_mode(pid_mode new_mode){
    motor1.setControlMode(new_mode);
    motor2.setControlMode(new_mode);
    motor3.setControlMode(new_mode);
    motor4.setControlMode(new_mode);
    motor5.setControlMode(new_mode);
}
