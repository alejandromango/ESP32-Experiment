#include "MotorUnit.h"

MotorUnit::MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS){
    pid = MiniPID(0,0,0);
    updatePIDTune();
    pid.setOutputLimits(-65535,65535);
    motor = DRV8873LED(tlc, forwardPin, backwardPin, readbackPin, senseResistor, cal);
    angleSensor = AS5048A(angleCS);
    angleSensor.init();
}

void MotorUnit::setSetpoint(float newSetpoint){
    setpoint = newSetpoint;
}

float MotorUnit::getSetpoint(){
    return setpoint;
}

float MotorUnit::getError(){
    return errorDist;
}

float MotorUnit::getOutput(){
    return output;
}

void MotorUnit::setControlMode(mode newMode){
    controlMode = newMode;
    updatePIDTune();
    stop();
}

mode MotorUnit::getControlMode(){
    return controlMode;
}

float MotorUnit::getRevolutionsFromAngle(float angle){
    return angle/360;
}

float MotorUnit::getDistanceFromAngle(float angle){
    return getRevolutionsFromAngle(angle) * _mmPerRevolution;
}

void MotorUnit::setPitch(float newPitch){
    _mmPerRevolution = newPitch;
}

float MotorUnit::getPitch(){
    return _mmPerRevolution;
}

void MotorUnit::setPIDTune(float kP, float kI, float kD){
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

void MotorUnit::updatePIDTune(){
    if(controlMode == CURRENT){
        pid.setPID(ampProportional, ampIntegral, ampDerivative);
    }else if(controlMode == DISTANCE){
        pid.setPID(mmProportional, mmIntegral, mmDerivative);
    }else if(controlMode == SPEED){
        pid.setPID(vProportional, vIntegral, vDerivative);
    }else{
        pid.setPID(rProportional, rIntegral, rDerivative);
    }
    computePID();
}

void MotorUnit::computePID(){
    lastInterval = (millis() - lastUpdate)/1000;
    lastUpdate = millis();
    currentState = getControllerState();
    errorDist = setpoint - currentState;
    output = int(pid.getOutput(currentState,setpoint));

    if(~disabled){
        motor.runAtPID(output);
    }else{
        motor.stop();
    }
}

float MotorUnit::getControllerState(){
    if(controlMode == CURRENT){
        mampsCurrent = motor.readCurrent();
        return mampsCurrent;
    }else{
        previousAngleTotal = angleTotal;
        angleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation());
        angleSensor.AbsoluteAngleRotation(&angleTotal, &angleCurrent, &anglePrevious);
        if(controlMode == DISTANCE){
            mmPosition = getDistanceFromAngle(angleTotal);
            return mmPosition;
        }else if(controlMode == SPEED){
            mmPerSecond = (getDistanceFromAngle(angleTotal) - getDistanceFromAngle(previousAngleTotal))/lastInterval;
            return mmPerSecond;
        }else{
            revolutionPosition = getRevolutionsFromAngle(angleTotal);
            return revolutionPosition;
        }
    }
}

void MotorUnit::eStop(){
    _disableControl();
    motor.stop();
}

void MotorUnit::reset(){
    _enableControl();
    stop();
}

void MotorUnit::stop(){
    if(controlMode == CURRENT || controlMode == SPEED){
        setpoint = 0;
    }else{
        setpoint = getControllerState();
    }
    computePID();
}

void MotorUnit::_disableControl(){
    disabled = true;
}

void MotorUnit::_enableControl(){
    disabled = false;
}
