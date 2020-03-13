#ifndef MotorUnit_h
#define MotorUnit_h

#include <Arduino.h>
#include "TLC59711.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "MiniPID.h" //https://github.com/tekdemo/MiniPID
#include "DRV8873LED.h"
#include "AS5048A.h"

class MotorUnit{
public:
    enum mode {REVOLUTIONS, CURRENT, DISTANCE, SPEED};
    MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS);
    void   write(const float& targetPosition);
    float  read();
    void   set(const float& newMotorUnitPosition);
    void   setSteps(const long& steps);
    int    updatePositionFromEncoder();
    int    detach();
    int    attach();
    void   detachIfIdle();
    void   endMove(const float& finalTarget);
    void   stop();
    float  target();
    float  error();
    float  setpoint();
    void   computePID();
    void   disablePositionPID();
    void   enablePositionPID();
    void   setPIDAggressiveness(float aggressiveness);
    void   test();
    void   changePitch(float* newPitch);
    float  getPitch();
    void   changeEncoderResolution(float* newResolution);
    bool   attached();
    MotorGearboxEncoder    motorGearboxEncoder;
    void   setPIDValues(float* Kp, float* Ki, float* Kd, float* propWeight, float* KpV, float* KiV, float* KdV, float* propWeightV);
    String     getPIDString();
    double     pidInput();
    double     pidOutput();
    long  steps();
    float getRevolutionsFromAngle(float angle);
    float getDistanceFromAngle(float angle);

private:
    MiniPID pid;
    DRV8873LED motor;
    AS5048A angleSensor;
    int        _PWMread(int pin);
    void       _writeFloat(const unsigned int& addr, const float& x);
    float      _readFloat(const unsigned int& addr);
    unsigned long   _timeLastMoved;
    volatile double _pidSetpoint;
    volatile double _pidInput;
    volatile double _pidOutput;
    float      *_Kp, *_Ki, *_Kd;
    PID        _pidController;
    float      _mmPerRevolution;
    float      *_encoderSteps;
    bool       _disableMotorUnitForTesting = false;
    char       _MotorUnitName;

    bool disabled = false;

    int output = 0;
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
