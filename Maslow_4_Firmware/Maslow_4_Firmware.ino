#include <WiFi.h>
#include <FS.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h" //https://github.com/me-no-dev/ESPAsyncWebServer
#include "AS5048A.h"
#include "html.h"
#include "MiniPID.h" //https://github.com/tekdemo/MiniPID
#include "TLC59711.h"
#include "DRV8873LED.h"
#include "comands.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"

const char* ssid = "Leek Soup";
const char* password = "Cranberry Pie";

AsyncWebServer server(80);

float RotationAngle1 = 0.0;
float AngleCurrent1  = 0.0;
float AmpsCurrent1  = 0.0;
float AnglePrevious1 = 0.0;
float errorDist1 = 0.0;

float RotationAngle2 = 0.0;
float AngleCurrent2  = 0.0;
float AmpsCurrent2  = 0.0;
float AnglePrevious2 = 0.0;
float errorDist2 = 0.0;

float RotationAngle3 = 0.0;
float AngleCurrent3  = 0.0;
float AmpsCurrent3  = 0.0;
float AnglePrevious3 = 0.0;
float errorDist3 = 0.0;

float RotationAngle4 = 0.0;
float AngleCurrent4  = 0.0;
float AmpsCurrent4  = 0.0;
float AnglePrevious4 = 0.0;
float errorDist4 = 0.0;

float RotationAngle5 = 0.0;
float AngleCurrent5  = 0.0;
float AmpsCurrent5  = 0.0;
float AnglePrevious5 = 0.0;
float errorDist5 = 0.0;

MiniPID pid1 = MiniPID(10000,.1,0);
MiniPID pid2 = MiniPID(10000,.1,0);
MiniPID pid3 = MiniPID(10000,.1,0);
MiniPID pid4 = MiniPID(10000,.1,0);
MiniPID pid5 = MiniPID(10000,.1,0);

unsigned long ourTime = millis();

esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
esp_err_t config_err_1 = adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_2 = adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_3 = adc1_config_channel_atten(ADC1_GPIO36_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_4 = adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_5 = adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_2_5);
#define NUM_TLC59711 1
#define tlcData   5
#define tlcClock  21
TLC59711 tlc = TLC59711(NUM_TLC59711, tlcClock, tlcData);
DRV8873LED motor1 = DRV8873LED(&tlc, 1, 0, ADC1_GPIO33_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor2 = DRV8873LED(&tlc, 3, 2, ADC1_GPIO34_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor3 = DRV8873LED(&tlc, 5, 4, ADC1_GPIO36_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor4 = DRV8873LED(&tlc, 7, 6, ADC1_GPIO32_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor5 = DRV8873LED(&tlc, 9, 8, ADC1_GPIO35_CHANNEL, 10000.0, adc_1_characterisitics);

AS5048A angleSensor1(17);
AS5048A angleSensor2(3);
AS5048A angleSensor3(22);
AS5048A angleSensor4(25);
AS5048A angleSensor5(13);

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  angleSensor1.init();
  angleSensor2.init();
  angleSensor3.init();
  angleSensor4.init();
  angleSensor5.init();

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
    request->send_P(200, "text/plain", String(RotationAngle1/360, 5).c_str());
  });
  server.on("/position2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(RotationAngle2/360, 5).c_str());
  });
  server.on("/position3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(RotationAngle3/360, 5).c_str());
  });
  server.on("/position4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(RotationAngle4/360, 5).c_str());
  });
  server.on("/position5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(RotationAngle5/360, 5).c_str());
  });

  server.on("/target1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(setPoint1, 5).c_str());
  });
  server.on("/target2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(setPoint2, 5).c_str());
  });
  server.on("/target3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(setPoint3, 5).c_str());
  });
  server.on("/target4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(setPoint4, 5).c_str());
  });
  server.on("/target5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(setPoint5, 5).c_str());
  });

  server.on("/errorDist1", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(errorDist1, 5).c_str());
  });
  server.on("/errorDist2", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(errorDist2, 5).c_str());
  });
  server.on("/errorDist3", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(errorDist3, 5).c_str());
  });
  server.on("/errorDist4", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(errorDist4, 5).c_str());
  });
  server.on("/errorDist5", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(errorDist5, 5).c_str());
  });


  server.begin();

  pid1.setOutputLimits(-65535,65535);
  pid2.setOutputLimits(-65535,65535);
  pid3.setOutputLimits(-65535,65535);
  pid4.setOutputLimits(-65535,65535);
  pid5.setOutputLimits(-65535,65535);

  tlc.begin();
  tlc.write();
  Serial.println("Setup complete");

}

int motorSpeedValue = 65535;

void loop(){

    //Find the position
    AngleCurrent1 = angleSensor1.RotationRawToAngle(angleSensor1.getRawRotation());
    angleSensor1.AbsoluteAngleRotation(&RotationAngle1, &AngleCurrent1, &AnglePrevious1);
    errorDist1 = setPoint1 - (RotationAngle1/360);
    motor1.runAtPID(int(pid1.getOutput(RotationAngle1/360,setPoint1)));
    AngleCurrent2 = angleSensor2.RotationRawToAngle(angleSensor2.getRawRotation());
    angleSensor2.AbsoluteAngleRotation(&RotationAngle2, &AngleCurrent2, &AnglePrevious2);
    errorDist2 = setPoint2 - (RotationAngle2/360);
    motor2.runAtPID(int(pid2.getOutput(RotationAngle2/360,setPoint2)));
    AngleCurrent3 = angleSensor3.RotationRawToAngle(angleSensor3.getRawRotation());
    angleSensor3.AbsoluteAngleRotation(&RotationAngle3, &AngleCurrent3, &AnglePrevious3);
    errorDist3 = setPoint3 - (RotationAngle3/360);
    motor3.runAtPID(int(pid3.getOutput(RotationAngle3/360,setPoint3)));
    AngleCurrent4 = angleSensor4.RotationRawToAngle(angleSensor4.getRawRotation());
    angleSensor4.AbsoluteAngleRotation(&RotationAngle4, &AngleCurrent4, &AnglePrevious4);
    errorDist4 = setPoint4 - (RotationAngle4/360);
    motor4.runAtPID(int(pid4.getOutput(RotationAngle4/360,setPoint4)));
    AngleCurrent5 = angleSensor5.RotationRawToAngle(angleSensor5.getRawRotation());
    angleSensor5.AbsoluteAngleRotation(&RotationAngle5, &AngleCurrent5, &AnglePrevious5);
    errorDist5 = setPoint5 - (RotationAngle5/360);
    motor5.runAtPID(int(pid5.getOutput(RotationAngle5/360,setPoint5)));
    // // //Set the speed of the motor
    // motor3.runAtPID(int(pid.getOutput(RotationAngle,setPoint)));

    // AmpsCurrent = motor3.readCurrent();
    // errorDist = setPoint - AmpsCurrent;

    // //Set the speed of the motor
    // motor3.runAtPID(int(pid.getOutput(AmpsCurrent,setPoint)));
    // if(millis() - ourTime > 1000){
    //   Serial.print("Forward motor speed: ");
    //   Serial.printf("%g \n", int(pid.getOutput(AmpsCurrent,setPoint)));
    //   Serial.print("Motor angle: ");
    //   Serial.printf("%g \n", AngleCurrent);
    //   ourTime = millis();
    // }
    //Motor test
    // motor3.forward(50000);
    // delay(2000);
    // Serial.print("Forward motor current: ");
    // Serial.printf("%g \n", motor3.readCurrent());
    // delay(1000);
    // motor3.fullBackward();
    // delay(2000);
    // Serial.print("Backward motor current: ");
    // Serial.printf("%g \n", motor3.readCurrent());
    // delay(1000);


    // TLC test
    // Serial.println("HighZ all (5V, 5V)");
    // motor1.highZ();
    // motor2.highZ();
    // motor3.highZ();
    // motor4.highZ();
    // motor5.highZ();
    // delay(5000);
    // Serial.println("Stopping (0, 0), all");
    // motor1.stop();
    // motor2.stop();
    // motor3.stop();
    // motor4.stop();
    // motor5.stop();
    // delay(5000);



}
