#include <WiFi.h>
#include <FS.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h" //https://github.com/me-no-dev/ESPAsyncWebServer
#include "html.h"
#include "comands.h"
#include "Ticker.h"
#include "maslow_v2.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

// #define BOARD_BRINGUP

const char* ssid = "Leek Soup";
const char* password = "Cranberry Pie";

AsyncWebServer server(80);

unsigned long ourTime = millis();

machine mill;

Ticker motorTimer = Ticker();

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

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

  server.on("/position1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor1->getInput(), 5).c_str());
  });
  server.on("/position2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor2->getInput(), 5).c_str());
  });
  server.on("/position3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor3->getInput(), 5).c_str());
  });
  server.on("/position4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor4->getInput(), 5).c_str());
  });
  server.on("/position5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor5->getInput(), 5).c_str());
  });

  server.on("/target1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor1->getSetpoint(), 5).c_str());
  });
  server.on("/target2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor2->getSetpoint(), 5).c_str());
  });
  server.on("/target3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor3->getSetpoint(), 5).c_str());
  });
  server.on("/target4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor4->getSetpoint(), 5).c_str());
  });
  server.on("/target5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor5->getSetpoint(), 5).c_str());
  });

  server.on("/errorDist1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor1->getError(), 5).c_str());
  });
  server.on("/errorDist2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor2->getError(), 5).c_str());
  });
  server.on("/errorDist3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor3->getError(), 5).c_str());
  });
  server.on("/errorDist4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor4->getError(), 5).c_str());
  });
  server.on("/errorDist5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(mill.motor5->getError(), 5).c_str());
  });

  server.on("/proportional", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(proportional, 5).c_str());
  });
  server.on("/integral", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(integral, 5).c_str());
  });
  server.on("/derivative", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(derivative, 5).c_str());
  });

  server.begin();

  mill.machine_init();
#ifndef BOARD_BRINGUP
  motorTimer.attach_ms(100, onTimer); //Gets error when faster than ~100ms cycle
#endif
  Serial.println("Setup complete");

}

void onTimer(){
  mill.compute_pid();
}

void loop(){
#ifndef BOARD_BRINGUP
  delay(1);
  if(setpointFlag){
    mill.update_setpoints(setPoint1, setPoint2, setPoint3, setPoint4, setPoint5);
    setpointFlag = false;
  }
  if(pidFlag){
    mill.update_pid_tunes(proportional, integral, derivative);
    pidFlag = false;
  }
  if(modeFlag){
    mill.update_control_mode(updatedMode);
    proportional = mill.motor1->getP();
    integral = mill.motor1->getI();
    derivative = mill.motor1->getD();
    modeFlag = false;
  }
#else
  Serial.println("Pins high:");
  motor1.motor->highZ();
  motor2.motor->highZ();
  motor3.motor->highZ();
  motor4.motor->highZ();
  motor5.motor->highZ();
  delay(5000);
  Serial.println("Pins low:");
  motor1.motor->stop();
  motor2.motor->stop();
  motor3.motor->stop();
  motor4.motor->stop();
  motor5.motor->stop();
  delay(5000);
#endif
}
