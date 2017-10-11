#include <Wire.h>
#include <gpio.h>

// WiFi
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>

// Sensor Includes
#include <Adafruit_BMP085.h>
#include <DHT.h>

// Misc Variables
#define OK_LED_PIN 14
#define ERROR_LED_PIN 12
#define REQUEST_LED_PIN 15

#define ENABLE_BATTERY_READING true
#define BATTERY_PIN A0
#define BATTERY_VOLTAGE_ENABLE_PIN 10

// Enable sleep, sleep length in microseconds 20e6 is 20 seconds
#define ENABLE_DEEP_SLEEP true
#define DEEP_SLEEP_LENGTH 60e6

#define NO_SLEEP_READING_DELAY 5000

int okLedState = HIGH;

// If battery is off then transmit vcc
#if ENABLE_BATTERY_READING == false
ADC_MODE(ADC_VCC);
#endif

// WiFi Variables
#include "./login.h"

// Sensor Variables
#define DHTPIN 13
#define DHTTYPE DHT22
DHT dhtSensor(DHTPIN, DHTTYPE);

Adafruit_BMP085 bmpSensor;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);

  // Wait for serial to initalize
  while (!Serial) {}

  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(REQUEST_LED_PIN, OUTPUT);

  #if ENABLE_BATTERY_READING == true
  pinMode(BATTERY_VOLTAGE_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, LOW);
  
  pinMode(BATTERY_PIN, INPUT);
  #endif

  digitalWrite(OK_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);
  digitalWrite(REQUEST_LED_PIN, LOW);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    okLedState = !okLedState;
    digitalWrite(OK_LED_PIN, okLedState);
    Serial.print(".");
  }
  
  okLedState = HIGH;
  digitalWrite(OK_LED_PIN, okLedState);
  digitalWrite(ERROR_LED_PIN, LOW);

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  bmpSensor.begin();

  // Need to wait 2 seconds for dhtSensor to initalize
  Serial.println("Reading...");
  delay(2000);

}

void loop() {
  char outputUrl[800] = "";
  char buffer[15];

  float seaLevelPressure = 1017.2*100;

  float bmpTemp = bmpSensor.readTemperature();
  float bmpPressure = bmpSensor.readPressure();
  float bmpAltitude = bmpSensor.readAltitude(seaLevelPressure);

  float dhtTemp = dhtSensor.readTemperature();
  float dhtHumidity = dhtSensor.readHumidity();
  float dhtHeatIndex = dhtSensor.computeHeatIndex(dhtTemp, dhtHumidity);

  #if ENABLE_BATTERY_READING == true
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, HIGH);
  delay(10); // Wait for the circuit to switch
  
  analogRead(BATTERY_PIN);
  delay(2); // For ADC to stabalize

  float batteryPinReading = analogRead(BATTERY_PIN);
  
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, LOW);

  int batteryPercent;
  int batMinVoltage = 3700;
  int batMaxVoltage = 4400;
  

  //float maxReading = 997.81;

  //Serial.print("New Reading: ");
  //Serial.println(batteryPinReading/maxReading);
  
  float dividerRatio = 4.56;

  int refVoltage = 4550;
  float fullRangeMult = 1023.0 / 997.81; // Range / MaxMv out of 1V for input signal maxMv being for ref voltage
  
  float adcReading = (batteryPinReading * fullRangeMult);
  float batteryVoltage = adcReading * dividerRatio;

  if (batteryVoltage <= batMinVoltage) {
    batteryPercent = 0;
  } else if (batteryVoltage >= batMaxVoltage) {
    batteryPercent = 100;
  } else {
    batteryPercent = (batteryVoltage - batMinVoltage) * 100 / (batMaxVoltage - batMinVoltage);
  }

  Serial.print("Pin Reading: ");
  Serial.println(batteryPinReading);

  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.print(" ");
  Serial.print(batteryPercent);
  Serial.println("%");
  
  #else
  float batteryVoltage = ESP.getVcc()/1024.0;
  #endif

  digitalWrite(REQUEST_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, LOW);

  strcat(outputUrl, "/report?");
  strcat(outputUrl, "temp=");

  Serial.println(dhtTemp);

  if (dhtTemp) {
    strcat(outputUrl, String((bmpTemp+dhtTemp)/2).c_str());
  } else {
    strcat(outputUrl, String(bmpTemp).c_str());
  }
  strcat(outputUrl, ",humidity=");
  strcat(outputUrl, String(dhtHumidity).c_str());
  strcat(outputUrl, ",heatIndex=");
  strcat(outputUrl, String(dhtHeatIndex).c_str());
  strcat(outputUrl, ",pressure=");
  strcat(outputUrl, String(bmpPressure).c_str());
  strcat(outputUrl, ",altitude=");
  strcat(outputUrl, String(bmpAltitude).c_str());
  strcat(outputUrl, ",bat=");
  strcat(outputUrl, String(batteryVoltage).c_str());
  //strcat(outputUrl, ",pin=");
  //strcat(outputUrl, String(batteryPinReading).c_str());

  Serial.println(outputUrl);
  
  HTTPClient http;
  http.begin("192.168.1.166", 8080, outputUrl); //HTTP
  int httpCode = http.GET();

  if(httpCode > 0) {
    if(httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
    } else {
      Serial.println(httpCode);
    }
  } else {
    Serial.println(httpCode);
    digitalWrite(ERROR_LED_PIN, HIGH);
  }

  http.end();

  digitalWrite(REQUEST_LED_PIN, LOW);

  #if ENABLE_DEEP_SLEEP
  ESP.deepSleep(DEEP_SLEEP_LENGTH);
  #else
  delay(NO_SLEEP_READING_DELAY);
  #endif
}
