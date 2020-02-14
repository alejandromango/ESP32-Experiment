#include <WiFi.h>
#include <FS.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h" //https://github.com/me-no-dev/ESPAsyncWebServer
#include "AS5048A.h"
#include "html.h"
#include "ESP32MotorControl.h" 	// https://github.com/JoaoLopesF/ESP32MotorControl
#include "MiniPID.h" //https://github.com/tekdemo/MiniPID
#include "TLC59711.h"
#include "DRV8873LED.h"
#include "comands.h"

const char* ssid = "Leek Soup";
const char* password = "Cranberry Pie";

AsyncWebServer server(80);

AS5048A angleSensor(17);

float RotationAngle = 0.0;
float AngleCurrent  = 0.0;
float AnglePrevious = 0.0;
float errorDist = 0.0;

const int MotorIn1 = 12;
const int MotorIn2 = 14;

MiniPID pid = MiniPID(1000,.1,0);


#define NUM_TLC59711 1
#define tlcData   5
#define tlcClock  21
TLC59711 tlc = TLC59711(NUM_TLC59711, tlcClock, tlcData);
DRV8873LED motor1 = DRV8873LED(&tlc, 3, 2);
DRV8873LED motor2 = DRV8873LED(&tlc, 1, 0);
DRV8873LED motor3 = DRV8873LED(&tlc, 4, 5);
DRV8873LED motor4 = DRV8873LED(&tlc, 7, 6);
DRV8873LED motor5 = DRV8873LED(&tlc, 9, 8);

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  angleSensor.init();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", HTML);
  });

  server.on("/settarget", HTTP_POST, [](AsyncWebServerRequest *request){

    int paramsNr = request->params();
    Serial.println(paramsNr);

    for(int i=0;i<paramsNr;i++){

        AsyncWebParameter* p = request->getParam(i);
        parseCommand(p->name(), p->value());
    }

    request->redirect("/");
  });

  server.on("/position", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(RotationAngle/360, 5).c_str());
  });

  server.on("/target", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(setPoint, 5).c_str());
  });

  server.on("/errorDist", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(errorDist, 5).c_str());
  });


  server.begin();

  MotorControl.attachMotor(MotorIn1, MotorIn2);
  motorStop();

  pid.setOutputLimits(-255,255);

  tlc.begin();
  tlc.write();

}

void loop(){

    //Find the position
    // AngleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation());
    // angleSensor.AbsoluteAngleRotation(&RotationAngle, &AngleCurrent, &AnglePrevious);
    // errorDist = setPoint - (RotationAngle/360.0);

    // //Set the speed of the motor
    // motorSpeed(int(pid.getOutput(RotationAngle/360.0,setPoint)));

    // //Print it out
    // //Serial.println(int(pid.getOutput(RotationAngle/360.0,setPoint)));
    delay(100);
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    motor5.stop();
    // for(int i = 0; i<12; i++){
    //   tlc.setPWM(i, 65535-i);
    // }
    // tlc.write();
    delay(100);
    motor1.highZ();
    motor2.highZ();
    motor3.highZ();
    motor4.highZ();
    motor5.highZ();
    // for(int i = 0; i<12; i++){
    //   tlc.setPWM(i, 0+i);
    // }
    // tlc.write();


}
