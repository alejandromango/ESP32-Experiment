/***************************************************
 *  This is a two-wire library for the TI DRV8873LED chip
 *
 *  Two pins are required to send data: clock and data pin.
 *
 *  Code based on Adafruit_DRV8873LED by Limor Fried/Ladyada (Adafruit Industries).
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
DRV8873LED::DRV8873LED(TLC59711 *tlc, uint8_t forwardPin, uint8_t backwardPin, uint8_t readbackPin, double senseResistor){
    _driver = tlc;
    _forward = forwardPin;
    _back = backwardPin;
    _readback = readbackPin;
    _rsense = senseResistor;

}

/*!
 *  @brief  Run the motors forward at the given speed
 *  @param speed
 *          The speed the motor should spin (0-65535)
 */
void DRV8873LED::forward(uint16_t speed){
    _driver->setPWM(_forward, 65535);
    _driver->setPWM(_back, speed);

}

/*!
 *  @brief  Run the motors forward at max speed
 */
void DRV8873LED::fullForward(){
    _driver->setPWM(_forward, 65535);
    _driver->setPWM(_back, 0);

}

/*!
 *  @brief  Run the motors backward at the given speed
 *  @param speed
 *          The speed the motor should spin (0-65535)
 */
void DRV8873LED::backward(uint16_t speed){
    _driver->setPWM(_back, 65535);
    _driver->setPWM(_forward, speed);

}

/*!
 *  @brief  Run the motors backward at max speed
 */
void DRV8873LED::fullBackward(){
    _driver->setPWM(_back, 65535);
    _driver->setPWM(_forward, 0);

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
        _driver->setPWM(_back, 65535);
        _driver->setPWM(_forward, speed);

    }else{
        _driver->setPWM(_forward, 65535);
        _driver->setPWM(_back, speed);
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
    int adcReadback = analogRead(_readback);

    return ((((adcReadback*3)/4095.0)+0.14)/_rsense)*1100.0;
}
