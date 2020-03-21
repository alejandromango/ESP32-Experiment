#include "maslow_v2.h"

std::unique_ptr<TLC59711> tlc;
std::unique_ptr<MotorUnit> motor1;
std::unique_ptr<MotorUnit> motor2;
std::unique_ptr<MotorUnit> motor3;
std::unique_ptr<MotorUnit> motor4;
std::unique_ptr<MotorUnit> motor5;

void machine_init(){
    esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
    esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
    esp_err_t config_err_1 = adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_2_5);
    esp_err_t config_err_2 = adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_2_5);
    esp_err_t config_err_3 = adc1_config_channel_atten(ADC1_GPIO36_CHANNEL, ADC_ATTEN_DB_2_5);
    esp_err_t config_err_4 = adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_2_5);
    esp_err_t config_err_5 = adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_2_5);
    tlc.reset(new TLC59711(NUM_TLC59711, TLC_CLOCK, TLC_DATA)); // MASLOW_TODO: Will this get garbage collected?
    motor1.reset(new MotorUnit(tlc.get(), MOTOR_1_FORWARD, MOTOR_1_BACKWARD, MOTOR_1_ADC, RSENSE, adc_1_characterisitics, MOTOR_1_CS));
    motor2.reset(new MotorUnit(tlc.get(), MOTOR_2_FORWARD, MOTOR_2_BACKWARD, MOTOR_2_ADC, RSENSE, adc_1_characterisitics, MOTOR_2_CS));
    motor3.reset(new MotorUnit(tlc.get(), MOTOR_3_FORWARD, MOTOR_3_BACKWARD, MOTOR_3_ADC, RSENSE, adc_1_characterisitics, MOTOR_3_CS));
    motor4.reset(new MotorUnit(tlc.get(), MOTOR_4_FORWARD, MOTOR_4_BACKWARD, MOTOR_4_ADC, RSENSE, adc_1_characterisitics, MOTOR_4_CS));
    motor5.reset(new MotorUnit(tlc.get(), MOTOR_5_FORWARD, MOTOR_5_BACKWARD, MOTOR_5_ADC, RSENSE, adc_1_characterisitics, MOTOR_5_CS));
    Serial.println("machine setup complete");
    tlc->begin();
    tlc->write();
}

void compute_pid(){
    motor1->computePID();
    motor2->computePID();
    motor3->computePID();
    motor4->computePID();
    motor5->computePID();
}

void update_setpoints(float setpoint_1,
                               float setpoint_2,
                               float setpoint_3,
                               float setpoint_4,
                               float setpoint_5){
    motor1->setSetpoint(setpoint_1);
    motor2->setSetpoint(setpoint_2);
    motor3->setSetpoint(setpoint_3);
    motor4->setSetpoint(setpoint_4);
    motor5->setSetpoint(setpoint_5);
}
void update_pid_tunes(float new_p, float new_i, float new_d){
    motor1->setPIDTune(new_p, new_i, new_d);
    motor2->setPIDTune(new_p, new_i, new_d);
    motor3->setPIDTune(new_p, new_i, new_d);
    motor4->setPIDTune(new_p, new_i, new_d);
    motor5->setPIDTune(new_p, new_i, new_d);
}

void update_control_mode(mode new_mode){
    motor1->setControlMode(new_mode);
    motor2->setControlMode(new_mode);
    motor3->setControlMode(new_mode);
    motor4->setControlMode(new_mode);
    motor5->setControlMode(new_mode);
}
