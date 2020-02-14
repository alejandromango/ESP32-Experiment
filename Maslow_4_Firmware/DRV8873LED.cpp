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
DRV8873LED::DRV8873LED(TLC59711 *tlc, uint8_t forwardPin, uint8_t backwardPin){
    _driver = tlc;
    _forward = forwardPin;
    _back = backwardPin;

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
        _driver->setPWM(_forward, 65535-speed);

    }else{
        _driver->setPWM(_forward, 65535);
        _driver->setPWM(_back, 65535-speed);
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
