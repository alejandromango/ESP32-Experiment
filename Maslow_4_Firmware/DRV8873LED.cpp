/***************************************************
 *  This is a two-wire library for the TI DRV8873LED chip
 *
 *  Two pins are required to send data: clock and data pin.
 ****************************************************/

#include "DRV8873LED.h"

/*!
 *  @brief  Instantiates a new DRV8873LED class for generic two-wire control
 *  @param  tlc
 *          LED driver object
 *  @param  forwardPin
 *          LED driver output to run motor forward (if other is at 0%)
 *  @param  backwardPin
 *          LED driver output to run motor backward (if other is at 0%)
 */
DRV8873LED::DRV8873LED(TLC59711 *tlc,
                       uint8_t forwardPin,
                       uint8_t backwardPin,
                       adc1_channel_t readbackPin,
                       double senseResistor,
                       esp_adc_cal_characteristics_t *cal){
    _driver = tlc;
    _forward = forwardPin;
    _back = backwardPin;
    _readback = readbackPin;
    _rsense = senseResistor;
    _cal_values = cal;

}

/*!
 *  @brief  Run the motors forward at the given speed
 *  @param speed
 *          The speed the motor should spin (0-65535)
 */
void DRV8873LED::forward(uint16_t speed){
    runAtSpeed(FORWARD, speed);
}

/*!
 *  @brief  Run the motors forward at max speed
 */
void DRV8873LED::fullForward(){
    runAtSpeed(FORWARD, 65535);
}

/*!
 *  @brief  Run the motors backward at the given speed
 *  @param speed
 *          The speed the motor should spin (0-65535)
 */
void DRV8873LED::backward(uint16_t speed){
    runAtSpeed(BACKWARD, speed);
}

/*!
 *  @brief  Run the motors backward at max speed
 */
void DRV8873LED::fullBackward(){
    runAtSpeed(BACKWARD, 65535);
}

void DRV8873LED::runAtPID(int signed_speed){
    if(signed_speed < 0){
        runAtSpeed(BACKWARD, abs(signed_speed));
    }else{
        runAtSpeed(FORWARD, abs(signed_speed));
    }
}

/*!
 *  @brief  Run the motors in the given direction at the given speed
 *  @param  direction
 *          direction backward (0) or forward (1, or ~0)
 *  @param speed
 *          The speed the motor should spin (0-65535)
 */
void DRV8873LED::runAtSpeed(uint8_t direction, uint16_t speed){
    if(direction == 0){
        _driver->setPWM(_forward, 65535);
        _driver->setPWM(_back, 65535 - speed);

    }else{
        _driver->setPWM(_back, 65535);
        _driver->setPWM(_forward, 65535 - speed);
    }
    _driver->write();
}


void DRV8873LED::stop(){
    _driver->setPWM(_forward, 65535);
    _driver->setPWM(_back, 65535);
    _driver->write();

}

void DRV8873LED::highZ(){
    _driver->setPWM(_forward, 0);
    _driver->setPWM(_back, 0);
    _driver->write();

}

double DRV8873LED::readCurrent(){
    int adcReadback = 0;
    for(int i= 0; i < multisamples; i++){
        adcReadback += adc1_get_raw(_readback);
        delayMicroseconds(10);
    }
    adcReadback = adcReadback/multisamples;
    double cal_mV = esp_adc_cal_raw_to_voltage(adcReadback, _cal_values);
    return (cal_mV/_rsense)*1100.0;
}
