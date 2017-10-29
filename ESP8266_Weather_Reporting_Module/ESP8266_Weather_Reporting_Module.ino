#include <Wire.h>
#include <gpio.h>

// WiFi
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// WiFi Variables
#include "./login.h"

// Sensor Includes
#include "./heatIndex.h"
#include "./SparkFun_RHT03.h"

#include <Adafruit_BMP085.h>

#define DEBUG 1

#define SERVER_IP_ADDRESS "192.168.1.160"
#define LED_PWM_VALUE 300

#define ENABLE_BATTERY_READING false

#define ENABLE_DEEP_SLEEP false // Enable sleep, sleep length in microseconds 20e6 is 20 seconds
#define DEEP_SLEEP_LENGTH 60e6
#define NO_SLEEP_READING_DELAY 60000 // 60 Seconds for delay with deep sleep disabled

int okLedState = HIGH;
unsigned long lastSenseTime = 9999999;

// Pin Assignments
#define OK_LED_PIN D8
#define ERROR_LED_PIN D6

#define BATTERY_PIN A0
#define BATTERY_VOLTAGE_ENABLE_PIN 10

// If battery is off then transmit vcc
#if ENABLE_BATTERY_READING == false
ADC_MODE(ADC_VCC);
#endif

// Sensor Variables
#define DHT_PIN D7
RHT03 dhtSensor;

#define ANEMOMETOR_PIN D5
#define ANEMOMETOR_SAMPLE_LENGTH 30000 // In ms
volatile unsigned int anemometorSampleCount = 0;
unsigned long lastAnemometorSamplePrint = 0;
volatile unsigned long lastAnemometorInterrupt = 0;

Adafruit_BMP085 bmpSensor;

void handleAnemometorInterrupt() {
  if (millis()-lastAnemometorInterrupt > 50) {
    lastAnemometorInterrupt = millis();
    anemometorSampleCount++;
  }
}

void connectToWiFi(char* ssid, char* passwd) {
//  WiFi.forceSleepWake();
//  wifi_set_sleep_type(MODEM_SLEEP_T);
//  if ((WiFi.status() != WL_CONNECTED) {
//    WiFi.mode(WIFI_STA);
//    WiFi.begin(ssid, password);
//  }
  

  
}

void delayForWiFi() {
  
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    okLedState = !okLedState;
    analogWrite(OK_LED_PIN, okLedState ? LED_PWM_VALUE : 0);
    Serial.print(".");
  }

  okLedState = LOW;
  analogWrite(OK_LED_PIN, 0);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);

  // Wait for serial to initalize
  while (!Serial) {}

  // LEDs
  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  analogWrite(OK_LED_PIN, LED_PWM_VALUE);
  analogWrite(ERROR_LED_PIN, LED_PWM_VALUE);

  // For battery instead of vcc reading
  #if ENABLE_BATTERY_READING == true
  pinMode(BATTERY_VOLTAGE_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, LOW);
  
  pinMode(BATTERY_PIN, INPUT);
  #endif
  
  //******************************
  //* Start Anemometor Counting
  //******************************
  pinMode(ANEMOMETOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETOR_PIN), handleAnemometorInterrupt, RISING);
  anemometorSampleCount = 0;
  lastAnemometorSamplePrint = millis();

  //******************************
  //* Start BMP and DHT sensors
  //******************************
  bmpSensor.begin(BMP085_ULTRALOWPOWER);
  dhtSensor.begin(DHT_PIN);

  //*************************
  //* Start Wifi
  //*************************
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  delayForWiFi();
  
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  analogWrite(ERROR_LED_PIN, 0);
}

void loop() {
  if (millis()-lastSenseTime < NO_SLEEP_READING_DELAY){
    return;
  }

  lastSenseTime = millis();
  
  #ifdef DEBUG
  Serial.println("Starting Sensing");
  #endif
  
  // First reset any error lights from last runs
  analogWrite(ERROR_LED_PIN, 0);
  analogWrite(OK_LED_PIN, LED_PWM_VALUE);

  //******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  #ifdef DEBUG
  Serial.println("Sensing BMP");
  #endif
  float seaLevelPressure = 1017.2*100;

  float bmpTemp = bmpSensor.readTemperature();
  float bmpPressure = bmpSensor.readPressure();
  float bmpAltitude = bmpSensor.readAltitude(seaLevelPressure);

  #ifdef DEBUG
  Serial.println("Sensing DHT22");
  #endif
  delay(RHT_READ_INTERVAL_MS);
  int updateResult = dhtSensor.update();

  if (updateResult != 1) {
    Serial.println("Problem reading DHT22");
  }
  
  float dhtTemp = dhtSensor.tempC();
  float dhtHumidity = dhtSensor.humidity();
  float dhtHeatIndex = computeHeatIndex(dhtTemp, dhtHumidity, false);

  #ifdef DEBUG
  Serial.print("DHT Temp: ");
  Serial.println(dhtTemp);

  Serial.println("Sensing Anemometer");
  #endif
  //******************************************/
  //* ANEMOMETOR BLOCK
  //******************************************/
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

  // Rpm is *2 because there's 2 samples per revolution
  // It needs to stay as rpm so the server can calibrate itself?
  // Unless we output kph then we can convert to mph and have the calibration data stored on the esp8266 in this code here?
  // int calibrationCoeficient = 0.313; // Maybe something like this?
  // What if it's not a linear correlation?
  // round(samples*.5*minutes)
  int anemometorRpm = floor((numTimelyAnemometorSamples * 0.5 * (60000.0 / (millisNow-lastAnemometorSamplePrint))) + 0.5);

  #ifdef DEBUG
  Serial.print(numTimelyAnemometorSamples);
  Serial.print(" Samples in last ");
  Serial.print((millisNow-lastAnemometorSamplePrint)/1000.0);
  Serial.print(" second ");
  Serial.print(anemometorRpm);
  Serial.println(" RPM");
  #endif

  anemometorSampleCount -= numTimelyAnemometorSamples;
  lastAnemometorSamplePrint = millisNow;

  //******************************************/
  //* Battery block 
  //******************************************/
  #if ENABLE_BATTERY_READING == true
  #ifdef DEBUG
  Serial.println("Sensing Battery");
  #endif
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
  float batteryVoltage = ESP.getVcc(); // Cause we want mV
  #endif

  //****************/
  //* OUTPUT BLOCK */
  //****************/
  analogWrite(ERROR_LED_PIN, 0);

  int port = 80;
  char ip[] = SERVER_IP_ADDRESS;
  
  char outputUrl[800] = "";
  char buffer[15];

  strcat(outputUrl, "/report.php?");
  strcat(outputUrl, "temp=");

  if (dhtTemp == NAN) {
    Serial.println("DHT temp is NAN");
    strcat(outputUrl, String(bmpTemp).c_str());
  } else {
    
    strcat(outputUrl, String((bmpTemp+dhtTemp)/2).c_str());
    
    if (dhtHumidity == NAN) {
      Serial.println("DHT humidity is NAN");
    } else {
      strcat(outputUrl, "&humidity=");
      strcat(outputUrl, String(dhtHumidity).c_str());
      strcat(outputUrl, "&heatIndex=");
      strcat(outputUrl, String(dhtHeatIndex).c_str());
    }
  }
  strcat(outputUrl, "&pressure=");
  strcat(outputUrl, String(bmpPressure).c_str());
  strcat(outputUrl, "&altitude=");
  strcat(outputUrl, String(bmpAltitude).c_str());
  strcat(outputUrl, "&bat=");
  strcat(outputUrl, String(batteryVoltage).c_str());
  strcat(outputUrl, "&windSpeed=");
  strcat(outputUrl, String(anemometorRpm).c_str());

  // Secret key for security (>_O)
  strcat(outputUrl, "&key=frieggandham");

 
  Serial.print(ip);
  Serial.print(":");
  Serial.print(port);
  Serial.println(outputUrl);

  //WiFi.forceSleepWake();
  //delayForWiFi();

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
    Serial.print("Http Error: ");
    Serial.println(httpCode);
    analogWrite(ERROR_LED_PIN, LED_PWM_VALUE);
  }

  http.end();

  analogWrite(OK_LED_PIN, 0);

  #ifdef DEBUG
  Serial.println("Sleeping Now");
  #endif

  #if ENABLE_DEEP_SLEEP
  ESP.deepSleep(DEEP_SLEEP_LENGTH);
  #else
  //WiFi.forceSleepBegin();
  //delay(NO_SLEEP_READING_DELAY);
  #endif
}
