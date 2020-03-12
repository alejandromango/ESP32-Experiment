float setPoint1 = 0;
float setPoint2 = 0;
float setPoint3 = 0;
float setPoint4 = 0;
float setPoint5 = 0;

float proportional = 100000;
float integral = 10;
float derivative = 0.0;
bool pidFlag = false;

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
    if(command == "setproportional"){
        proportional = value.toFloat();
        pidFlag = true;
    }
    if(command == "setintegral"){
        integral = value.toFloat();
        pidFlag = true;
    }
    if(command == "setderivative"){
        derivative = value.toFloat();
        pidFlag = true;
    }
}
