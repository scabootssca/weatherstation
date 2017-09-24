#include <Wire.h>

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

int okLedState = HIGH;

// WiFi Variables
#include "./login.h"

// Sensor Variables
#define DHTPIN 13
#define DHTTYPE DHT22
DHT dhtSensor(DHTPIN, DHTTYPE);

Adafruit_BMP085 bmpSensor;

void setup() {
  Serial.begin(115200);

  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(REQUEST_LED_PIN, OUTPUT);

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

  digitalWrite(REQUEST_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, LOW);

  strcat(outputUrl, "/?");
  strcat(outputUrl, "temp=");
  strcat(outputUrl, String((bmpTemp+dhtTemp)/2).c_str());
  strcat(outputUrl, ",pressure=");
  strcat(outputUrl, String(bmpPressure).c_str());
  strcat(outputUrl, ",altitude=");
  strcat(outputUrl, String(bmpAltitude).c_str());
  strcat(outputUrl, ",humidity=");
  strcat(outputUrl, String(dhtHumidity).c_str());
  strcat(outputUrl, ",heatIndex=");
  strcat(outputUrl, String(dhtHeatIndex).c_str());

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

  delay(5000);

}
