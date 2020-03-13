#ifndef MotorUnit_h
#define MotorUnit_h

#include <Arduino.h>
#include "TLC59711.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "MiniPID.h" //https://github.com/tekdemo/MiniPID
#include "DRV8873LED.h"
#include "AS5048A.h"

enum mode {REVOLUTIONS, CURRENT, DISTANCE, SPEED};
class MotorUnit{
public:
    MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS);
    void   setSetpoint(float newSetpoint);
    float  getSetpoint();
    float  getError();
    float  getOutput();
    float  getInput();
    void   setControlMode(mode newMode);
    mode   getControlMode();
    float  getRevolutionsFromAngle(float angle);
    float  getDistanceFromAngle(float angle);
    void   setPitch(float newPitch);
    float  getPitch();
    void   setPIDTune(float kP, float kI, float kD);
    void   updatePIDTune();
    void   computePID();
    float  getControllerState();
    void   eStop();
    void   reset();
    void   stop();

private:
    void   _disableControl();
    void   _enableControl();


    MiniPID pid;
    DRV8873LED motor;
    AS5048A angleSensor;
    float _mmPerRevolution = 10;
    float lastInterval = 0.001;
    unsigned long lastUpdate = millis();

    // PID tunings for revolution position control
    float rProportional = 100000;
    float rIntegral = 10;
    float rDerivative = 0.0;

    // PID tunings for mm position control
    float mmProportional = 0;
    float mmIntegral = 0;
    float mmDerivative = 0;

    // PID tunings for speed control
    float vProportional = 0;
    float vIntegral = 0;
    float vDerivative = 0;

    // PID tunings for current control
    float ampProportional = 0;
    float ampIntegral = 0;
    float ampDerivative = 0;

    bool disabled = false;

    int output = 0;
    float currentState = 0.0;
    float setpoint = 0.0;
    float errorDist = 0.0;

    float angleTotal = 0.0;
    float previousAngleTotal = 0.0;
    float revolutionPosition = 0.0;
    float mmPosition = 0.0;
    float mmPerSecond = 0.0;
    float angleCurrent  = 0.0;
    float anglePrevious = 0.0;

    float mampsCurrent  = 0.0;
    mode controlMode = REVOLUTIONS;
};

#endif
