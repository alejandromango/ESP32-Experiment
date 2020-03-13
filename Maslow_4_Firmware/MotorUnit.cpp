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
}

float MotorUnit::getRevolutionsFromAngle(float angle){
    return angle/360;
}

float MotorUnit::getDistanceFromAngle(float angle){
    return getRevolutionsFromAngle(angle) * _mmPerRevolution;
}

void MotorUnit::computePID(){

    if(controlMode == CURRENT){
        mampsCurrent = motor.readCurrent();
        errorDist = setpoint - mampsCurrent;
        output = int(pid.getOutput(mampsCurrent,setpoint));

    }else{
        previousAngleTotal = angleTotal;
        angleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation());
        angleSensor.AbsoluteAngleRotation(&angleTotal, &angleCurrent, &anglePrevious);
        if(controlMode == DISTANCE){
            mmPosition = getDistanceFromAngle(angleTotal);
            errorDist = setpoint - mmPosition;
            output = int(pid.getOutput(mmPosition,setpoint));
        }else if(controlMode == SPEED){
            mmPerSecond = (getDistanceFromAngle(angleTotal - getDistanceFromAngle(previousAngleTotal))/loopInterval;
            errorDist = setpoint - mmPerSecond;
            output = int(pid.getOutput(mmPerSecond,setpoint));
        }else{
            revolutionPosition = getRevolutionsFromAngle(angleTotal);
            errorDist = setpoint - revolutionPosition;
            output = int(pid.getOutput(revolutionPosition,setpoint));
        }
    }
    if(~disabled){
        motor.runAtPID(output);
    }else{
        motor.stop();
    }
}

void MotorUnit::disableControl(){
    disabled = true;
}

void MotorUnit::enableControl(){
    disabled = false;
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

void   MotorUnit::changePitch(float newPitch){
    _mmPerRevolution = newPitch;
}

float  MotorUnit::getPitch(){
    return _mmPerRevolution;
}

void   MotorUnit::eStop(){
    disableControl();
    motor.stop();
}
