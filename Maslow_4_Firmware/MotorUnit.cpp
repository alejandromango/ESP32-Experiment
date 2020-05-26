/***************************************************
 *  This is a library for control of a DC motor via a TLC Led driver with angle
 *  sensor feedback on an ESP32.
 *
 *  By Alexander Martin-Ginnold for Maslow CNC
 ****************************************************/
#include "MotorUnit.h"

/*!
 *  @brief  Instantiates a new MotorUnit class. Instantiates classes for
 *          all other necessary controls.
 *  @param  tlc Pointer to a TLC59711 object to output PWM signals
 *  @param  forwardPin Output pin number for the TLC59711. If this pin is at max
 *          output and the other pin is at 0 the motor turns forward
 *  @param  backwardPin Output pin number for the TLC59711. If this pin is at
 *          max output and the other pin is at 0 the motor turns backward
 *  @param  readbackPin ESP32 adc_channel_t pin number for current readback
 *  @param  senseResistor Value in Ohms of the sense resistor for this channel
 *  @param  cal ESP32 adc calibration results for more accurate readings
 *  @param  angleCS ESP32 pin for the chip select of the angle sensor
 *
 */
MotorUnit::MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS,
               double mmPerRev,
               double desiredAccuracy){
    _mmPerRevolution = mmPerRev;
    accuracy = desiredAccuracy;
    pid.reset(new MiniPID(0,0,0));
    updatePIDTune();
    pid->setOutputLimits(-65535,65535);
    motor.reset(new DRV8873LED(tlc, forwardPin, backwardPin, readbackPin, senseResistor, cal));
    angleSensor.reset(new AS5048A(angleCS));
    angleSensor->init();
}

/*!
 *  @brief  Set a new setpoint for the PID loop
 *  @param newSetpoint Setpoint in the appropriate units for the control mode
 */
void MotorUnit::setSetpoint(double newSetpoint){
    setpoint = newSetpoint;
}

void MotorUnit::step(bool step, bool direction, double mm_per_step){
    if(step){
        if(!direction){
            setpoint += mm_per_step;
        }else{
            setpoint -= mm_per_step;
        }
    }
}

/*!
 *  @brief  Retrive the current setpoint of the PID loop
 *  @return Setpoint in the appropriate units for the control mode
 */
double MotorUnit::getSetpoint(){
    return setpoint;
}

/*!
 *  @brief  Retrive the current error of the PID loop
 *  @return Error in the appropriate units for the control mode
 */
double MotorUnit::getError(){
    return errorDist;
}

/*!
 *  @brief  Retrive the current output of the PID loops
 *  @return This will be an int within the set PID output range
 */
int MotorUnit::getOutput(){
    return output;
}

/*!
 *  @brief  Retrive the current input to the PID loop
 *  @return Current motor state in the appropriate units for the control mode
 */
double MotorUnit::getInput(){
    return currentState;
}

/*!
 *  @brief  Retrive the current regulation state of the PID loop
 *  @return A bool representing whether or not the PID loop is within accuracy requirements
 */
bool MotorUnit::getRegulationState(){
    return inRegulation;
}

/*!
 *  @brief  Set the active control mode
 *  @param newMode The enum member of the desired mode
 */
void MotorUnit::setControlMode(pid_mode newMode){
    controlMode = newMode;
    updatePIDTune();
    stop();
}

/*!
 *  @brief  Retrive the active control mode
 *  @return The enum member of the current mode
 */
pid_mode MotorUnit::getControlMode(){
    return controlMode;
}

/*!
 *  @brief  Calculate the position of the motor revolutions
 *  @param angle Total angle displacement from 0 in degrees
 *  @return Total revolutions displacement from 0
 */
double MotorUnit::getRevolutionsFromAngle(double angle){
    return angle/360;
}

/*!
 *  @brief  Calculate the position of the motor in distance units
 *  @param angle Total angle displacement from 0 in degrees
 *  @return The position displacement from 0 in mm
 */
double MotorUnit::getDistanceFromAngle(double angle){
    return getRevolutionsFromAngle(angle) * _mmPerRevolution;
}

/*!
 *  @brief  Set the pulley pitch
 *  @param newPitch Desired linear travel per encoder revolution in mm
 */
void MotorUnit::setPitch(double newPitch){
    _mmPerRevolution = newPitch;
}

/*!
 *  @brief  Retrive the pulley pitch
 *  @return The linear travel per encoder revolution in mm
 */
double MotorUnit::getPitch(){
    return _mmPerRevolution;
}

/*!
 *  @brief  Sets the PID tuning for the current control mode
 *  @param  kP The desired proportional tune
 *  @param  kI The desired integral tune
 *  @param  kD The desired derivative tune
 */
void MotorUnit::setPIDTune(double kP, double kI, double kD){
    if(controlMode == CURRENT){
        ampProportional = kP;
        ampIntegral = kI;
        ampDerivative = kD;
    }else if(controlMode == DISTANCE){
        mmProportional = kP;
        mmIntegral = kI;
        mmDerivative = kD;
    }else if(controlMode == SPEED){
        vProportional = kP;
        vIntegral = kI;
        vDerivative = kD;
    }else{
        rProportional = kP;
        rIntegral = kI;
        rDerivative = kD;
    }

    updatePIDTune();
}

/*!
 *  @brief  Applies the appropriate PID tune based on the active control mode
 */
void MotorUnit::updatePIDTune(){
    if(controlMode == CURRENT){
        activeP = ampProportional;
        activeI = ampIntegral;
        activeD = ampDerivative;
    }else if(controlMode == DISTANCE){
        activeP = mmProportional;
        activeI = mmIntegral;
        activeD = mmDerivative;
    }else if(controlMode == SPEED){
        activeP = vProportional;
        activeI = vIntegral;
        activeD = vDerivative;
    }else{
        activeP = rProportional;
        activeI = rIntegral;
        activeD = rDerivative;
    }
    pid->setPID(activeP, activeI, activeD);
}

/*!
 *  @brief  Retrive the proportional portion of the pid tune
 *  @return activeI
 */
double MotorUnit::getP(){
    return activeP;
}

/*!
 *  @brief  Retrive the integral portion of the pid tune
 *  @return activeI
 */
double MotorUnit::getI(){
    return activeI;
}

/*!
 *  @brief  Retrive the derivative portion of the pid tune
 *  @return activeD
 */
double MotorUnit::getD(){
    return activeD;
}

/*!
 *  @brief  Compute the necessary output to achieve the desired setpoint and
 *  command the motor to that output
 */
void MotorUnit::computePID(){
    errorDist = setpoint - currentState;
    output = int(pid->getOutput(currentState,setpoint));
    inRegulation = (fabs(errorDist) < accuracy);// & (mmPerSecond < 0.1);

    if(!disabled){
        motor->runAtPID(output);
    }else{
        motor->stop();
        Serial.println("Motor disabled");
    }
}

void MotorUnit::updateControllerState(){
    currentState = getControllerState();
}

/*!
 *  @brief  Retrieve the current state of the motor appropriate for the currently
 *  set control mode.
 *  @return Actual state of the motor. This could be position in revolutions or mm, current
 *  in mA, or speed in mm/s.
 */
double MotorUnit::getControllerState(){
    // lastInterval = (millis() - lastUpdate)/1000.0;
    // if(lastInterval < 0.05){return mmPosition;}
    // lastUpdate = millis();
    mampsCurrent = motor->readCurrent();
    previousAngleTotal = angleTotal;
    mostVal = 0;
    instances = 0;
    mostInstances = 0;
    for(int i = 0; i < ANGLE_READS; i++){
        angles[i] = angleSensor->getRawRotation();
        for(int j = 0; j <= i; j++){
            if(angles[i] == angles[j]){
                instances++;
                if (instances > mostInstances){
                    mostInstances = instances;
                    mostVal = angles[i];
                }
            }
        }
        instances = 0;
    }
    angleCurrent = angleSensor->RotationRawToAngle(mostVal);
    angleSensor->AbsoluteAngleRotation(&angleTotal, &angleCurrent, &anglePrevious);
    mmPosition = getDistanceFromAngle(angleTotal);
    double tempSpeed = (getDistanceFromAngle(angleTotal) - getDistanceFromAngle(previousAngleTotal))/lastInterval;
    mmPerSecond = (mmPerSecond+tempSpeed)/2;
    revolutionPosition = getRevolutionsFromAngle(angleTotal);
    if(controlMode == CURRENT){
        return mampsCurrent;
    }else{
        if(controlMode == DISTANCE){
            // Serial.printf("Returning mmposition %g, angle: %g, mm_rev: %g\n", mmPosition, angleTotal, _mmPerRevolution);
            return mmPosition;
        }else if(controlMode == SPEED){
            // mmPerSecond = (getDistanceFromAngle(angleTotal) - getDistanceFromAngle(previousAngleTotal))/lastInterval;
            return mmPerSecond;
        }else{
            return revolutionPosition;
        }
    }
}

/*!
 *  @brief Stop the motor immediately (don't wait for PID to get around to it)
 */
void MotorUnit::eStop(){
    _disableControl();
    motor->stop();
}

/*!
 *  @brief  Resets the motor after an emergency stop. Makes sure the motor does
 *  not move after being re-enabled
 */
void MotorUnit::reset(){
    stop();
    _enableControl();
}

/*!
 *  @brief  Stops the motor by manipulating the setpoint.
 *  Conditional based on control mode (stop means different things for
 *  speed and position)
 */
void MotorUnit::stop(){
    if(controlMode == CURRENT || controlMode == SPEED){
        setpoint = 0;
    }else{
        setpoint = getControllerState();
    }
    computePID();
}

/*!
 *  @brief  Disables PID loop (loop still runs, but always commands motor to stop)
 */
void MotorUnit::_disableControl(){
    disabled = true;
}

/*!
 *  @brief  Enables PID loop (output will now be sent to motors)
 */
void MotorUnit::_enableControl(){
    disabled = false;
}
