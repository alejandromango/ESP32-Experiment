// #ifdef maslow_v2_h

#include "maslow_v2.h"
esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
esp_err_t config_err_1 = adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_2 = adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_3 = adc1_config_channel_atten(ADC1_GPIO36_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_4 = adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_5 = adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_2_5);
TLC59711 tlc(NUM_TLC59711, TLC_CLOCK, TLC_DATA); // MASLOW_TODO: Will this get garbage collected?
MotorUnit motor1(&tlc, MOTOR_1_FORWARD, MOTOR_1_BACKWARD, MOTOR_1_ADC, RSENSE, adc_1_characterisitics, MOTOR_1_CS, DC_TOP_LEFT_MM_PER_REV, 1);
MotorUnit motor2(&tlc, MOTOR_2_FORWARD, MOTOR_2_BACKWARD, MOTOR_2_ADC, RSENSE, adc_1_characterisitics, MOTOR_2_CS, DC_TOP_RIGHT_MM_PER_REV, 1);
MotorUnit motor3(&tlc, MOTOR_3_FORWARD, MOTOR_3_BACKWARD, MOTOR_3_ADC, RSENSE, adc_1_characterisitics, MOTOR_3_CS, DC_BOTTOM_LEFT_MM_PER_REV, 1);
MotorUnit motor4(&tlc, MOTOR_4_FORWARD, MOTOR_4_BACKWARD, MOTOR_4_ADC, RSENSE, adc_1_characterisitics, MOTOR_4_CS, DC_BOTTOM_RIGHT_MM_PER_REV, 1);
MotorUnit motor5(&tlc, MOTOR_5_FORWARD, MOTOR_5_BACKWARD, MOTOR_5_ADC, RSENSE, adc_1_characterisitics, MOTOR_5_CS, DC_Z_AXIS_MM_PER_REV, 1);

static float x_max = 285.75;
static float y_max = 336.55;

enum homingStates {TOP_LEFT_HOMING, TOP_RIGHT_HOMING, BOTTOM_LEFT_HOMING, BOTTOM_RIGHT_HOMING, Z_HOMING, DONE};
homingStates homingState = BOTTOM_LEFT_HOMING;


void machine_init(){
    tlc.begin();
    tlc.write();
    delay(10); //Allow Angle sensors to boot
    Serial.println("machine setup complete");

}

/*
 Apply inverse kinematics for maslow CNC
 Take X, Y, and Z value and

*/
// void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position) {
//     //static float last_angle = 0;
//     //static float last_radius = 0;
//     float x, y, z;                  // distances in each cartesian axis
//     float maslow_target[N_AXIS];    // target location in maslow cable lengths
//     //grbl_sendf(CLIENT_SERIAL, "Position: %4.2f %4.2f %4.2f \r\n", position[X_AXIS] - x_offset, position[Y_AXIS], position[Z_AXIS]);
//     //grbl_sendf(CLIENT_SERIAL, "Target: %4.2f %4.2f %4.2f \r\n", target[X_AXIS] - x_offset, target[Y_AXIS], target[Z_AXIS]);
//     x = target[X_AXIS];
//     y = target[Y_AXIS];
//     z = target[Z_AXIS];

//     // MASLOWTODO: Drop equations for each of the cable lengths here.
//     maslow_target[DC_TOP_LEFT] = sqrt(pow(x - X_TL_OFFSET, 2) + pow(y - Y_TL_OFFSET, 2));
//     maslow_target[DC_TOP_RIGHT] = sqrt(pow(x_max - x - X_TR_OFFSET, 2) + pow(y - Y_TR_OFFSET, 2));
//     maslow_target[DC_BOTTOM_LEFT] = sqrt(pow(x - X_BL_OFFSET, 2) + pow(y - y_max - Y_BL_OFFSET, 2));
//     maslow_target[DC_BOTTOM_RIGHT] = sqrt(pow(x- x_max - X_BR_OFFSET, 2) + pow(y - y_max - Y_BR_OFFSET, 2));
//     maslow_target[DC_Z_AXIS] = z;

//     Serial.printf("Targets, Clockwise: %f, %f, %f, %f",
//                                 maslow_target[DC_TOP_LEFT],
//                                 maslow_target[DC_TOP_RIGHT],
//                                 maslow_target[DC_BOTTOM_LEFT],
//                                 maslow_target[DC_BOTTOM_RIGHT]);

//     mc_line(maslow_target, pl_data);
// }

/*
  user_defined_homing() is called at the begining of the normal Grbl_ESP32 homing
  sequence.  If user_defined_homing() returns false, the rest of normal Grbl_ESP32
  homing is skipped if it returns false, other normal homing continues.  For
  example, if you need to manually prep the machine for homing, you could implement
  user_defined_homing() to wait for some button to be pressed, then return true.
*/
bool user_defined_homing()
{
  // True = done with homing, false = continue with normal Grbl_ESP32 homing
  static bool direction = true; // False is backwards, true is forwards
  static double currentPos = motor1.getControllerState();
  static double lastPos = currentPos;
  static int swapCounter = 0;
  static int shakinIt = 0;
  static double minPos = MAXFLOAT;
  unsigned long reading_time;
switch(homingState){
case TOP_LEFT_HOMING:
  break;
case TOP_RIGHT_HOMING:
  Serial.println("Motor POsitions 1-5");
  motor1.getControllerState();
  motor2.getControllerState();
  motor3.getControllerState();
  currentPos = motor4.getControllerState();
  motor5.getControllerState();
  // Serial.printf("Motorstate, Current: %g, Last: %g\n",
  //                 currentPos,
  //                 lastPos);
  // Serial.printf("Current state: %g\n", currentPos);
  if (currentPos < minPos){
    minPos = currentPos;
    shakinIt = 0;
  }
  if (shakinIt > 10 & fabs(currentPos-minPos) < 0.1){
    Serial.printf("Found max = %g\n", minPos);
    motor4.motor->stop();
    return true;
  }
  if (direction){
      if ((round(lastPos * 10) / 10) <= (round(currentPos * 10) / 10)){
        swapCounter++;
        if (swapCounter > 5){
          shakinIt++;
          direction = false;
          Serial.printf("Switching to backward, Current: %g, Last: %g, Min: %g\n",
                          currentPos,
                          lastPos,
                          minPos);
          swapCounter = 0;
        }
      } else {
        swapCounter = 0;
      }
  } else {
      if ((round(lastPos * 10) / 10) <= (round(currentPos * 10) / 10)){
        swapCounter++;
        if (swapCounter > 5){
          shakinIt++;
          direction = true;
          Serial.printf("Switching to forward, Current: %g, Last: %g, Min: %g\n",
                          currentPos,
                          lastPos,
                          minPos);
          swapCounter = 0;
        }
      } else {
        swapCounter = 0;
      }
  }

  motor1.motor->fullBackward();
  if (direction){
    motor4.motor->forward(40000);
  } else {
    motor4.motor->backward(40000);
  }
  lastPos = currentPos;
  return false;
case BOTTOM_LEFT_HOMING:
  currentPos = motor1.getControllerState();
  motor2.getControllerState();
  motor3.getControllerState();
  motor4.getControllerState();
  motor5.getControllerState();
  // Serial.printf("Motorstate, Current: %g, Last: %g\n",
  //                 currentPos,
  //                 lastPos);
  // Serial.printf("Current state: %g\n", currentPos);
  if (currentPos < minPos){
    minPos = currentPos;
    shakinIt = 0;
  }
  if (shakinIt > 10 & fabs(currentPos-minPos) < 0.1){
    Serial.printf("Found max = %g\n", minPos);
    motor1.motor->stop();
    homingState = TOP_RIGHT_HOMING;
    direction = true; // False is backwards, true is forwards
    currentPos = motor1.getControllerState();
    lastPos = currentPos;
    swapCounter = 0;
    shakinIt = 0;
    minPos = MAXFLOAT;
    return false;
  }
  if (direction){
      if ((round(lastPos * 10) / 10) <= (round(currentPos * 10) / 10)){
        swapCounter++;
        if (swapCounter > 5){
          shakinIt++;
          direction = false;
          Serial.printf("Switching to backward, Current: %g, Last: %g, Min: %g\n",
                          currentPos,
                          lastPos,
                          minPos);
          swapCounter = 0;
        }
      } else {
        swapCounter = 0;
      }
  } else {
      if ((round(lastPos * 10) / 10) <= (round(currentPos * 10) / 10)){
        swapCounter++;
        if (swapCounter > 5){
          shakinIt++;
          direction = true;
          Serial.printf("Switching to forward, Current: %g, Last: %g, Min: %g\n",
                          currentPos,
                          lastPos,
                          minPos);
          swapCounter = 0;
        }
      } else {
        swapCounter = 0;
      }
  }

  motor4.motor->fullBackward();
  if (direction){
    motor1.motor->forward(40000);
  } else {
    motor1.motor->backward(40000);
  }
  lastPos = currentPos;
  return false;
case BOTTOM_RIGHT_HOMING:
case Z_HOMING:
  Serial.println("Motors Forward:");
  motor1.motor->fullForward();
  motor2.motor->fullForward();
  motor3.motor->fullForward();
  motor4.motor->fullForward();
  motor5.motor->fullForward();
  delay(5000);
  return false;
case DONE:
  Serial.println("Finished homing");
  return true;
}

}

/*
 Forward kinematics for maslow CNC
 MASLOWTODO: Back calculate the XYZ position based on the actual cable lengths.
*/
// void forward_kinematics(float *position) {
//     float original_position[N_AXIS];  // temporary storage of original
//     float print_position[N_AXIS];
//     int32_t current_position[N_AXIS];  // Copy current state of the system position variable
//     memcpy(current_position, sys_position, sizeof(sys_position));
//     system_convert_array_steps_to_mpos(print_position, current_position);

// }

/*
  kinematics_pre_homing() is called before normal homing
  You can use it to do special homing or just to set stuff up

  cycle_mask is a bit mask of the axes being homed this time.
*/
bool kinematics_pre_homing(uint8_t cycle_mask)
{
  return false; // finish normal homing cycle
}

/*
  kinematics_post_homing() is called at the end of normal homing
*/
void kinematics_post_homing()
{
}

void pid_step(uint8_t step_mask, uint8_t dir_mask){
    motor1.step(step_mask & (1<<DC_TOP_LEFT), dir_mask & (1<<DC_TOP_LEFT), 1.0/DC_TOP_LEFT_STEPS_PER_MM);
    motor2.step(step_mask & (1<<DC_TOP_RIGHT), dir_mask & (1<<DC_TOP_RIGHT), 1.0/DC_TOP_RIGHT_STEPS_PER_MM);
    motor3.step(step_mask & (1<<DC_BOTTOM_LEFT), dir_mask & (1<<DC_BOTTOM_LEFT), 1.0/DC_BOTTOM_LEFT_STEPS_PER_MM);
    motor4.step(step_mask & (1<<DC_BOTTOM_RIGHT), dir_mask & (1<<DC_BOTTOM_RIGHT), 1.0/DC_BOTTOM_RIGHT_STEPS_PER_MM);
    motor5.step(step_mask & (1<<DC_Z_AXIS), dir_mask & (1<<DC_Z_AXIS), 1.0/DC_Z_AXIS_STEPS_PER_MM);

}

void pid_get_state(){
    motor1.updateControllerState();
    motor2.updateControllerState();
    motor3.updateControllerState();
    motor4.updateControllerState();
    motor5.updateControllerState();
}

void motor_stop(){
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    motor5.stop();
}

void compute_pid(){
    motor1.computePID();
    motor2.computePID();
    motor3.computePID();
    motor4.computePID();
    motor5.computePID();
}

void update_setpoints(double setpoint_1,
                               double setpoint_2,
                               double setpoint_3,
                               double setpoint_4,
                               double setpoint_5){
    motor1.setSetpoint(setpoint_1);
    motor2.setSetpoint(setpoint_2);
    motor3.setSetpoint(setpoint_3);
    motor4.setSetpoint(setpoint_4);
    motor5.setSetpoint(setpoint_5);
}

bool machine_regulation(){
    return (motor1.getRegulationState() &
            motor2.getRegulationState() &
            motor3.getRegulationState() &
            motor4.getRegulationState() &
            motor5.getRegulationState());
}

void print_setpoints(){
    Serial.printf("Motor setpoints: %g, %g, %g, %g, %g\n",
                                    motor1.getSetpoint(),
                                    motor2.getSetpoint(),
                                    motor3.getSetpoint(),
                                    motor4.getSetpoint(),
                                    motor5.getSetpoint());
}

void update_pid_tunes(double new_p, double new_i, double new_d){
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
// #endif
