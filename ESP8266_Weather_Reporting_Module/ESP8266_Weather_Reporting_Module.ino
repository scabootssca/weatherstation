#include <Wire.h>

// WiFi
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>

// Sensor Includes
#include <Adafruit_BMP085.h>
#include <DHT.h>

// Misc Variables
const int okLed = 14;
const int errorLed = 12;
const int requestLed = 15;

int okLedState = HIGH;

// WiFi Variables
const char* ssid = "MyCharterWiFi03-2G";
const char* password = "royalbreeze440";

// Sensor Variables
#define DHTPIN 13
#define DHTTYPE DHT22
DHT dhtSensor(DHTPIN, DHTTYPE);

static bool usingBMPSensor;
Adafruit_BMP085 bmpSensor;
// ---------

void setup() {
  Serial.begin(115200);

  pinMode(okLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(requestLed, OUTPUT);

  digitalWrite(okLed, HIGH);
  digitalWrite(errorLed, HIGH);
  digitalWrite(requestLed, LOW);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    okLedState = !okLedState;
    digitalWrite(okLed, okLedState);
    Serial.print(".");
  }
  okLedState = HIGH;
  digitalWrite(okLed, okLedState);
  digitalWrite(errorLed, LOW);

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  usingBMPSensor = bmpSensor.begin();
  if (!usingBMPSensor) {
    Serial.println("Could not find a valid BMP sensor");
  }

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

  digitalWrite(requestLed, HIGH);
  digitalWrite(errorLed, LOW);

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
    digitalWrite(errorLed, HIGH);
  }

  http.end();

  digitalWrite(requestLed, LOW);
  
  delay(5000);

}
