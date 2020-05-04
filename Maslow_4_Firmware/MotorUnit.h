#ifndef MotorUnit_h
#define MotorUnit_h
#include "memory"
// #include "grbl.h"

#include <Arduino.h>
#include "TLC59711.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "MiniPID.h" //https://github.com/tekdemo/MiniPID
#include "DRV8873LED.h"
#include "AS5048A.h"

enum pid_mode {REVOLUTIONS, CURRENT, DISTANCE, SPEED, MAX = SPEED};
class MotorUnit{

public:
    MotorUnit(TLC59711 *tlc,
               uint8_t forwardPin,
               uint8_t backwardPin,
               adc1_channel_t readbackPin,
               double senseResistor,
               esp_adc_cal_characteristics_t *cal,
               byte angleCS,
               double mmPerRev,
               double desiredAccuracy);
    std::unique_ptr<MiniPID> pid;
    std::unique_ptr<DRV8873LED> motor;
    std::unique_ptr<AS5048A> angleSensor;
    void   setSetpoint(double newSetpoint);
    double  getSetpoint();
    void   step(bool step, bool direction, double mm_per_step);
    double  getError();
    int    getOutput();
    double  getInput();
    bool   getRegulationState();
    void   setControlMode(pid_mode newMode);
    pid_mode   getControlMode();
    double  getRevolutionsFromAngle(double angle);
    double  getDistanceFromAngle(double angle);
    void   setPitch(double newPitch);
    double  getPitch();
    void   setPIDTune(double kP, double kI, double kD);
    void   updatePIDTune();
    void   computePID();
    double  getP();
    double  getI();
    double  getD();
    void    updateControllerState();
    double  getControllerState();
    void   eStop();
    void   reset();
    void   stop();

private:
    void   _disableControl();
    void   _enableControl();

    double _mmPerRevolution = 1;
    double lastInterval = 0.001;
    unsigned long lastUpdate = millis();

    // PID tunings for revolution position control
    double rProportional = 100000;
    double rIntegral = 10;
    double rDerivative = 0.0;

    // PID tunings for mm position control
    double mmProportional = 300000;// 600000; Commented values for 24v 100rpm gearmotor.
    double mmIntegral = 200;// 20;            Active values for 6v 30 RPM  gearmotor
    double mmDerivative = 0;// 200000;

    // PID tunings for speed control
    double vProportional = 0;
    double vIntegral = 0;
    double vDerivative = 0;

    // PID tunings for current control
    double ampProportional = 0;
    double ampIntegral = 0;
    double ampDerivative = 0;

    // active PID tunings
    double activeP = 0;
    double activeI = 0;
    double activeD = 0;

    bool disabled = false;
    bool inRegulation = false;

    double accuracy = 0.05; // Accuracy in mm to set in regulation flag

    int output = 0;
    double currentState = 0.0;
    double setpoint = 0.0;
    double errorDist = 0.0;

    double angleTotal = 0.0;
    double previousAngleTotal = 0.0;
    double revolutionPosition = 0.0;
    double mmPosition = 0.0;
    double mmPerSecond = 0.0;
    double angleCurrent  = 0.0;
    double anglePrevious = 0.0;

    double mampsCurrent  = 0.0;
    pid_mode controlMode = DISTANCE;

};

#endif
