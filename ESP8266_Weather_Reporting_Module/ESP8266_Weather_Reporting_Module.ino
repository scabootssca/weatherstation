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
#define REQUEST_LED_PIN 2

#define SERVER_IP_ADDRESS "192.168.1.160"

#define ENABLE_BATTERY_READING true
#define BATTERY_PIN A0
#define BATTERY_VOLTAGE_ENABLE_PIN 10

// Enable sleep, sleep length in microseconds 20e6 is 20 seconds
#define ENABLE_DEEP_SLEEP false
#define DEEP_SLEEP_LENGTH 60e6

// 60 Seconds for delay with deep sleep disabled
#define NO_SLEEP_READING_DELAY 60000

#define LED_PWM_VALUE 300
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

#define ANEMOMETOR_PIN 15
#define ANEMOMETOR_SAMPLE_LENGTH 30000 // In ms
unsigned int anemometorSampleCount = 0;
unsigned long lastAnemometorSamplePrint = 0;
unsigned long lastAnemometorInterrupt = 0;

Adafruit_BMP085 bmpSensor;

void handleAnemometorInterrupt() {
  if (millis()-lastAnemometorInterrupt > 50) {
    lastAnemometorInterrupt = millis();
    anemometorSampleCount++;
  }
}

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

  /* Start the anemometor counting here so we have a larger result length */
  pinMode(ANEMOMETOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETOR_PIN), handleAnemometorInterrupt, RISING);

  anemometorSampleCount = 0;
  lastAnemometorSamplePrint = millis();
  
  analogWrite(OK_LED_PIN, LED_PWM_VALUE);
  analogWrite(ERROR_LED_PIN, LED_PWM_VALUE);
  analogWrite(REQUEST_LED_PIN, 0);

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    okLedState = !okLedState;
    analogWrite(OK_LED_PIN, okLedState ? LED_PWM_VALUE : 0);
    Serial.print(".");
  }
  
  okLedState = HIGH;
  analogWrite(OK_LED_PIN, okLedState ? LED_PWM_VALUE : 0);
  analogWrite(ERROR_LED_PIN, 0);

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  bmpSensor.begin(BMP085_ULTRALOWPOWER);

  // Need to wait 2 seconds for dhtSensor to initalize
  Serial.println("Reading...");
  delay(2000);

}

void loop() {
  char outputUrl[800] = "";
  char buffer[15];

  /* ANEMOMETOR BLOCK */
  /********************/
  // Delay longer if it hasn't been 30 seconds
  // Samples will count using interrupts
  long msToDelayAnemometor = ANEMOMETOR_SAMPLE_LENGTH-(millis()-lastAnemometorSamplePrint);
  if (msToDelayAnemometor > 0) {
    Serial.print("Sampling anemometor for an additional ");
    Serial.print(msToDelayAnemometor/1000);
    Serial.println(" seconds.");
    
    delay(msToDelayAnemometor);
  }
  
  int millisNow = millis();
  int numTimelyAnemometorSamples = anemometorSampleCount;
  int anemometorRpm = numTimelyAnemometorSamples*(60.0/((millisNow-lastAnemometorSamplePrint)/1000.0));

  Serial.print(numTimelyAnemometorSamples);
  Serial.print(" Samples in last ");
  Serial.print((millisNow-lastAnemometorSamplePrint)/1000.0);
  Serial.print(" second ");
  Serial.print(anemometorRpm);
  Serial.println(" RPM");

  anemometorSampleCount -= numTimelyAnemometorSamples;
  lastAnemometorSamplePrint = millisNow;

  /* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  /******************************************/
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

  /* OUTPUT BLOCK */
  analogWrite(REQUEST_LED_PIN, LED_PWM_VALUE);
  analogWrite(ERROR_LED_PIN, 0);

  strcat(outputUrl, "/report.php?");
  strcat(outputUrl, "temp=");

  if (dhtTemp) {
    strcat(outputUrl, String((bmpTemp+dhtTemp)/2).c_str());
  } else {
    strcat(outputUrl, String(bmpTemp).c_str());
  }
  strcat(outputUrl, "&humidity=");
  strcat(outputUrl, String(dhtHumidity).c_str());
  strcat(outputUrl, "&heatIndex=");
  strcat(outputUrl, String(dhtHeatIndex).c_str());
  strcat(outputUrl, "&pressure=");
  strcat(outputUrl, String(bmpPressure).c_str());
  strcat(outputUrl, "&altitude=");
  strcat(outputUrl, String(bmpAltitude).c_str());
  strcat(outputUrl, "&bat=");
  strcat(outputUrl, String(batteryVoltage).c_str());
  strcat(outputUrl, "&windSpeed=");
  strcat(outputUrl, String(anemometorRpm).c_str());

  int port = 80;
  char ip[] = SERVER_IP_ADDRESS;

  Serial.print(ip);
  Serial.print(":");
  Serial.print(port);
  Serial.println(outputUrl);
  
  HTTPClient http;
  http.begin(ip, port, outputUrl); //HTTP
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
    analogWrite(ERROR_LED_PIN, LED_PWM_VALUE);
  }

  http.end();
  
  analogWrite(REQUEST_LED_PIN, 0);

  #if ENABLE_DEEP_SLEEP
  ESP.deepSleep(DEEP_SLEEP_LENGTH);
  #else
  delay(NO_SLEEP_READING_DELAY);
  #endif
}
