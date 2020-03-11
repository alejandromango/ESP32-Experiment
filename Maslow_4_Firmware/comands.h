float setPoint1 = 0;
float setPoint2 = 0;
float setPoint3 = 0;
float setPoint4 = 0;
float setPoint5 = 0;
// ESP32MotorControl MotorControl = ESP32MotorControl();

void parseCommand(String command, String value){
    Serial.println("Parsing Command:");
    Serial.println(command);

    if(command == "setpoint1"){
        Serial.println("Error: ");
        Serial.println(setPoint1 - value.toFloat());
        setPoint1 = value.toFloat();
    }
    if(command == "setpoint2"){
        Serial.println("Error: ");
        Serial.println(setPoint2 - value.toFloat());
        setPoint2 = value.toFloat();
    }
    if(command == "setpoint3"){
        Serial.println("Error: ");
        Serial.println(setPoint3 - value.toFloat());
        setPoint3 = value.toFloat();
    }
    if(command == "setpoint4"){
        Serial.println("Error: ");
        Serial.println(setPoint4 - value.toFloat());
        setPoint4 = value.toFloat();
    }
    if(command == "setpoint5"){
        Serial.println("Error: ");
        Serial.println(setPoint5 - value.toFloat());
        setPoint5 = value.toFloat();
    }
}

void

// void setupMotor(){
    // const int freq = 1000;
    // const int ledChannel = 0;
    // const int resolution = 8;

    // ledcSetup(ledChannel, freq, resolution);

    // attach the channel to the GPIO to be controlled
    // ledcAttachPin(MotorIn1, ledChannel);
    // ledcAttachPin(MotorIn2, ledChannel);
// }

// void motorStop(){
//     MotorControl.motorsStop();
// }

// void motorForwards(){
    // digitalWrite(MotorIn1, LOW);
    // digitalWrite(MotorIn2, HIGH);
// }

// void motorReverse(){
    // digitalWrite(MotorIn1, HIGH);
    // digitalWrite(MotorIn2, LOW);
// }

// void motorSpeed(int speed){
//     //Serial.println(speed);
//     if(speed < 0){
//         // Serial.println("Less than zero");
//         speed = constrain(abs(speed), 0, 255);
//         MotorControl.motorReverse(0, speed);
//     }
//     else{
//         // Serial.println("More than zero");
//         speed = constrain(abs(speed), 0, 255);
//         MotorControl.motorForward(0, speed);
//     }
// }

