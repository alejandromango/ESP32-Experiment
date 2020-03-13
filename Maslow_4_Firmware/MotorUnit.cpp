#include "MotorUnit.h"

MotorUnit::MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS){
    pid = MiniPID(100000,10,0.0);
    pid.setOutputLimits(-65535,65535);
    motor = DRV8873LED(tlc, forwardPin, backwardPin, readbackPin, senseResistor, cal);
    angleSensor = AS5048A(angleCS);
    angleSensor.init();
}

void    MotorUnit::write(const float& targetPosition){
    _timeLastMoved = millis();
    _pidSetpoint   =  targetPosition/ *_mmPerRotation;
    return;
}

float  MotorUnit::read(){
    //returns the true MotorUnit position

    return (motorGearboxEncoder.encoder.read()/ *_encoderSteps) * *_mmPerRotation;

}

float  MotorUnit::setpoint(){
    return _pidSetpoint * *_mmPerRotation;
}

void   MotorUnit::set(const float& newMotorUnitPosition){

    //reset everything to the new value
    _pidSetpoint  =  newMotorUnitPosition/ *_mmPerRotation;
    motorGearboxEncoder.encoder.write((newMotorUnitPosition * *_encoderSteps)/ *_mmPerRotation);

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
    if(~disabled){
        motor.runAtPID(output);
    }else{
        motor.stop();
    }
}

void   MotorUnit::disablePositionPID(){

    _pidController.SetMode(MANUAL);

}

void   MotorUnit::enablePositionPID(){

    _pidController.SetMode(AUTOMATIC);

}

void   MotorUnit::setPIDValues(float* KpPos, float* KiPos, float* KdPos, float* propWeight, float* KpV, float* KiV, float* KdV, float* propWeightV){
    /*

    Sets the positional PID values for the MotorUnit

    */
    _Kp = KpPos;
    _Ki = KiPos;
    _Kd = KdPos;

    _pidController.SetTunings(_Kp, _Ki, _Kd, propWeight);

    motorGearboxEncoder.setPIDValues(KpV, KiV, KdV, propWeightV);
}

String  MotorUnit::getPIDString(){
    /*

    Get PID tuning values

    */
    String PIDString = "Kp=";
    return PIDString + *_Kp + ",Ki=" + *_Ki + ",Kd=" + *_Kd;
}

void   MotorUnit::setPIDAggressiveness(float aggressiveness){
    /*

    The setPIDAggressiveness() function sets the aggressiveness of the PID controller to
    compensate for a change in the load on the motor.

    */

    motorGearboxEncoder.setPIDAggressiveness(aggressiveness);
}

float  MotorUnit::error(){

    float encoderErr = (motorGearboxEncoder.encoder.read()/ *_encoderSteps) - _pidSetpoint;

    return encoderErr * *_mmPerRotation;
}

void   MotorUnit::changePitch(float *newPitch){
    /*
    Reassign the distance moved per-rotation for the MotorUnit.
    */
    _mmPerRotation = newPitch;
}

float  MotorUnit::getPitch(){
    /*
    Returns the distance moved per-rotation for the MotorUnit.
    */
    return *_mmPerRotation;
}

void   MotorUnit::changeEncoderResolution(float *newResolution){
    /*
    Reassign the encoder resolution for the MotorUnit.
    */
    _encoderSteps = newResolution;

    //push to the gearbox for calculating RPM
    motorGearboxEncoder.setEncoderResolution(*newResolution);

}

int    MotorUnit::detach(){

    motorGearboxEncoder.motor.detach();

    return 1;
}

int    MotorUnit::attach(){
     motorGearboxEncoder.motor.attach();
     sys.writeStepsToEEPROM = true;
     return 1;
}

bool   MotorUnit::attached(){
    /*

    Returns true if the MotorUnit is attached, false if it is not.

    */

    return motorGearboxEncoder.motor.attached();
}

void   MotorUnit::detachIfIdle(){
    /*
    Detaches the MotorUnit, turning off the motor and PID control, if it has been
    stationary for more than MotorUnitDetachTime
    */
    if (millis() - _timeLastMoved > sysSettings.MotorUnitDetachTime){
        detach();
    }

}

void   MotorUnit::endMove(const float& finalTarget){

    _timeLastMoved = millis();
    _pidSetpoint    = finalTarget/ *_mmPerRotation;

}

void   MotorUnit::stop(){
    /*

    Immediately stop the MotorUnit where it is, not where it should be

    */

    _timeLastMoved = millis();
    _pidSetpoint   = read()/ *_mmPerRotation;

}

void   MotorUnit::test(){
    /*
    Test the MotorUnit by directly commanding the motor and observing if the encoder moves
    */

    Serial.print(F("Testing "));
    Serial.print(_MotorUnitName);
    Serial.println(F(" motor:"));

    //print something to prevent the connection from timing out
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));

    int i = 0;
    double encoderPos = motorGearboxEncoder.encoder.read(); //record the position now

    //move the motor
    motorGearboxEncoder.motor.directWrite(255);
    while (i < 1000){
        i++;
        maslowDelay(1);
        if (sys.stop){return;}
    }
    //stop the motor
    motorGearboxEncoder.motor.directWrite(0);

    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() > 500){
        Serial.println(F("Direction 1 - Pass"));
    }
    else{
        Serial.println(F("Direction 1 - Fail"));
    }

    //record the position again
    encoderPos = motorGearboxEncoder.encoder.read();
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));

    //move the motor in the other direction
    i = 0;
    motorGearboxEncoder.motor.directWrite(-255);
    while (i < 1000){
        i++;
        maslowDelay(1);
        if (sys.stop){return;}
    }
    //stop the motor
    motorGearboxEncoder.motor.directWrite(0);

    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() < -500){
        Serial.println(F("Direction 2 - Pass"));
    }
    else{
        Serial.println(F("Direction 2 - Fail"));
    }

    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
}

double  MotorUnit::pidInput(){ return _pidInput * *_mmPerRotation;}
double  MotorUnit::pidOutput(){ return _pidOutput;}
