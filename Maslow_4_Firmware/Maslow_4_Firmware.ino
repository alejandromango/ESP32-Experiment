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

#include "driver/adc.h"
#include "esp_adc_cal.h"

const char* ssid = "Leek Soup";
const char* password = "Cranberry Pie";

AsyncWebServer server(80);

AS5048A angleSensor(22);

float RotationAngle = 0.0;
float AngleCurrent  = 0.0;
float AnglePrevious = 0.0;
float errorDist = 0.0;

const int MotorIn1 = 12;
const int MotorIn2 = 14;

MiniPID pid = MiniPID(1000,.1,0);

unsigned long ourTime;

esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
esp_err_t config_err_1 = adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_2 = adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_3 = adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_4 = adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_2_5);
esp_err_t config_err_5 = adc1_config_channel_atten(ADC1_GPIO36_CHANNEL, ADC_ATTEN_DB_2_5);
#define NUM_TLC59711 1
#define tlcData   5
#define tlcClock  21
TLC59711 tlc = TLC59711(NUM_TLC59711, tlcClock, tlcData);
DRV8873LED motor1 = DRV8873LED(&tlc, 3, 2, ADC1_GPIO32_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor2 = DRV8873LED(&tlc, 1, 0, ADC1_GPIO33_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor3 = DRV8873LED(&tlc, 4, 5, ADC1_GPIO34_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor4 = DRV8873LED(&tlc, 7, 6, ADC1_GPIO35_CHANNEL, 10000.0, adc_1_characterisitics);
DRV8873LED motor5 = DRV8873LED(&tlc, 9, 8, ADC1_GPIO36_CHANNEL, 10000.0, adc_1_characterisitics);

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
  Serial.println("Setup complete");

}

int motorSpeedValue = 65535;

void loop(){

    //Find the position
    AngleCurrent = angleSensor.RotationRawToAngle(angleSensor.getRawRotation());
    angleSensor.AbsoluteAngleRotation(&RotationAngle, &AngleCurrent, &AnglePrevious);
    errorDist = setPoint - (RotationAngle/360.0);

    // //Set the speed of the motor
    // motorSpeed(int(pid.getOutput(RotationAngle/360.0,setPoint)));

    // //Print it out
    Serial.println(AngleCurrent);
    Serial.println("Motor Forward");
    ourTime = millis();
    motor3.forward(motorSpeedValue);
    Serial.print("Motor speed: ");
    Serial.println(motorSpeedValue);
    while (millis() - ourTime < 15000) {
      delay(2000);
      Serial.print("Forward motor current: ");
      Serial.printf("%g \n", motor3.readCurrent());
      delay(1000);
      // motorSpeedValue =  motorSpeedValue - 5000;
    }

    motorSpeedValue = 65535;


}
