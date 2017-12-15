/*
Copyright 2017 Jason Ryan Richards

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include <Wire.h>
#include <gpio.h>
#include <SPI.h>
#include "RTClib.h"

// WiFi
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// Required for LIGHT_SLEEP_T delay mode
extern "C" {
#include "user_interface.h"
}

// WiFi Variables
#include "./login.h"

// Sensor Includes
#include "./helpers.h"

#include "./Adafruit_MCP23008.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define DEBUG 1

#if DEBUG
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// Config
#define ENABLE_DEEP_SLEEP false // Enable sleep, sleep length in microseconds 20e6 is 20 seconds
#define DEEP_SLEEP_LENGTH 60e6
#define NO_SLEEP_READING_DELAY 60000 // 60 Seconds for delay with deep sleep disabled

#define MAX_READING_STORAGE 240 // How many readings to store in memory
#define SUBMIT_MIN_READINGS 10 // How many to have before attempting to submit
#define MAX_FAILED_SUBMITS 5

#define SERVER_IP_ADDRESS "192.168.1.160"
#define GMT_OFFSET -6

// Pin Assignments
#define OK_LED_PIN 1
#define ERROR_LED_PIN 0

#define SOLAR_ENABLE_PIN 4

#define BAT_ENABLE_PIN 3

#define INTERRUPT_PIN D2
#define ADC_CS_PIN 7

ADC_MODE(ADC_VCC);

// Globals
WeatherReading storedReadings[MAX_READING_STORAGE];
int readingIndex = 0;
short int serverDownCounter = 0;

int okLedState = HIGH;
unsigned long lastSenseTime = 9999999;

// Real Time Clock
RTC_DS1307 RTC;
// IO Expander
Adafruit_MCP23008 mcp;

// Sensors
/**********/
// BME 280
#define SEALEVELPRESSURE 1017.2*100
Adafruit_BME280 bmeSensor;
bool bmeConnected = true;

// Anemometer
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 10.2677201193868
#define ANEMOMETER_PIN 2
#define ANEMOMETER_SAMPLE_LENGTH NO_SLEEP_READING_DELAY//30000 // In ms
volatile unsigned int anemometerSampleCount = 0;
unsigned long lastAnemometerSamplePrint = 0;

// Wind Vane
//#define WIND_VANE_ENABLE_PIN
#define WIND_VANE_PIN 1

bool connectToWiFi() {
	// If we're connected then return
	if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN("Already Connected");
		return true;
	}

  DEBUG_PRINTLN("Not Connected");

  //******************************
  //* Init WiFi variables
  //******************************
  IPAddress ip(192, 168, 1, 180);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.forceSleepWake();
  delay(1);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.config(ip, gateway, subnet);

  noInterrupts();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  interrupts();

  //wifi_set_sleep_type(LIGHT_SLEEP_T);

  unsigned int totalWaitTime = 0;

  while (WiFi.status() != WL_CONNECTED) {
    waitMs(500);
    totalWaitTime += 500;
    okLedState = !okLedState;
    mcp.digitalWrite(OK_LED_PIN, okLedState ? HIGH : LOW);
    DEBUG_PRINT(".");

    // Wait 10 secs and give up
    if (totalWaitTime > 30000) {
      DEBUG_PRINTLN("WiFi error connecting.");
      return false;
    }
  }

  okLedState = LOW;
  mcp.digitalWrite(OK_LED_PIN, LOW);

	DEBUG_PRINTLN("");
	DEBUG_PRINT("IP address: ");
	DEBUG_PRINTLN(WiFi.localIP());
  //DEBUG_PRINT("Strength: ");
  //DEBUG_PRINT(getRSSI(WIFI_SSID));
  //DEBUG_PRINTLN(" db");

  return true;
}

void disconnectFromWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    //DEBUG_PRINTLN("Tried to disconnect while already disconnected");
    return;
  }

  WiFi.disconnect(true);
  WiFi.forceSleepBegin();
  delay(1);
}

bool submit_reading(WeatherReading currentReading, bool disconnectAfterSubmission=true, bool updateClock=false) {
  bool success = false;

  char outputUrl[300] = "";

  strcat(outputUrl, "/report.php?");

  if (!isnan(currentReading.temperature)) {
    strcat(outputUrl, "temp=");
    strcat(outputUrl, String(currentReading.temperature).c_str());
  }

  if (!isnan(currentReading.humidity)) {
      strcat(outputUrl, "&humidity=");
      strcat(outputUrl, String(currentReading.humidity).c_str());

      // If we have temperature and humidity then calculate and submit the heat index also
      if (!isnan(currentReading.temperature)) {
        strcat(outputUrl, "&heatIndex=");
        strcat(outputUrl, String(computeHeatIndex(currentReading.temperature, currentReading.humidity, false)).c_str());
      }
  }

  if (!isnan(currentReading.pressure)) {
    strcat(outputUrl, "&pressure=");
    strcat(outputUrl, String(currentReading.pressure).c_str());
  }

  strcat(outputUrl, "&bat=");
  strcat(outputUrl, String(currentReading.battery).c_str());
  strcat(outputUrl, "&windSpeed=");
  strcat(outputUrl, String(currentReading.windSpeed).c_str());
  strcat(outputUrl, "&windDirection=");
  strcat(outputUrl, String(currentReading.windDirection).c_str());

  strcat(outputUrl, "&timestamp=");
  strcat(outputUrl, String(currentReading.timestamp).c_str());


  // Secret key for security (>_O)
  strcat(outputUrl, "&key=f6f9b0b8348a85843e951723a3060719f55985fd"); // frie!ggandham!!%2{[ sha1sum

  int port = 80;
  char ip[] = SERVER_IP_ADDRESS;

  // Connect if needed
  if (connectToWiFi()) {
    HTTPClient http;

    Serial.print(ip);
    Serial.print(":");
    Serial.print(port);
    Serial.println(outputUrl);

    noInterrupts();
    http.begin(ip, port, outputUrl); //HTTP
    interrupts();

    int httpCode = http.GET();

    if(httpCode > 0) {
      if(httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        //Serial.println(payload);

        if (payload.startsWith("S")) {
          if (updateClock) {
            // Get the new time
            strtok((char*)payload.c_str(), "\n");
            char *newTime = strtok(NULL, "\n");
            int intTime = atoi(newTime);

            Serial.print("Success! Also, setting clock to ");
            Serial.println(newTime);

            RTC.adjust(DateTime(intTime));
          } else {
            Serial.print("Success!");
          }

          success = true;
        } else {
          Serial.println("Failure: Invalid payload");
          Serial.println(payload);
        }
      } else {
        Serial.print("Http Error: ");
        Serial.println(httpCode);
      }
    } else {
      Serial.print("Http Error: ");
      Serial.println(httpCode);
    }

    mcp.digitalWrite(ERROR_LED_PIN, success ? LOW : HIGH);

    noInterrupts();
    http.end();

//    if (disconnectAfterSubmission) {
//      disconnectFromWiFi();
//    }

    interrupts();
  }

  return success;
}

void submit_stored_readings() {
  /*
  /* Really need a multiple submit in here, maybe a post with temp[1],temp[2] or temp_1 whatever
  */
  int failedSubmissionIndexes[MAX_READING_STORAGE];
  int failedIndex = 0;

  short numConsecutiveFails = 0;

  // Turn this off too
  mcp.digitalWrite(OK_LED_PIN, LOW);

  // Go through the readings and submit any that are there
  for (int i=0; i<MAX_READING_STORAGE; i++) {
    // Turn the light off for this now
    mcp.digitalWrite(ERROR_LED_PIN, LOW);

    // If we've reached an unpopulated reading then we've reached the end and stop
    if (storedReadings[i].populated == false) {
      break;
    }

    DEBUG_PRINTLN("");
    DEBUG_PRINT(">> Submitting index: ");
    DEBUG_PRINTLN(i);

    bool success = submit_reading(storedReadings[i], false, !i);

    if (!success) {
      DEBUG_PRINTLN("Failed!, Caching for later.");
      failedSubmissionIndexes[failedIndex++] = i;
      numConsecutiveFails++;
    } else {
      numConsecutiveFails = 0;
    }

    // If the index is also the number of fails
    // And if we've failed too many times in a row then quit
    // The server must be down
    if (i == numConsecutiveFails-1 && numConsecutiveFails > MAX_FAILED_SUBMITS) {
      DEBUG_PRINT("Aborting submission, the server must be down. Counter is: ");
      serverDownCounter = 4; // Includes 0 so 4 is don't try for 4 times and try on the 5th
      DEBUG_PRINTLN(serverDownCounter);

      return;
    }

    // Do background things between submissions
    //yield();
    delay(50); // Maybe this will let all go through? if we even still have them
  }

  // Iterate through the reading indexes and if it's an index less than the failed count means it's a failed one
  // Get the associated index from the readings and copy its data over
  for (int i=0; i<MAX_READING_STORAGE; i++) {
    // If it is within the count of failed indexes we need to put storedReadings[failIndex[i]] at storedReadings[i]
    if (i < failedIndex) {
      storedReadings[i] = storedReadings[failedSubmissionIndexes[i]];
    // Else flag it as not populated
    } else {
      storedReadings[i].populated = false;
    }
  }

  // Set the current index to the index of where the failed readings stopped
  readingIndex = failedIndex;

  // If we've failed on all then set the index back to zero
  if (readingIndex == MAX_READING_STORAGE) {
    readingIndex = 0;
  }

  DEBUG_PRINTLN();
  DEBUG_PRINT("New index: ");
  DEBUG_PRINTLN(readingIndex);

  disconnectFromWiFi();
}

uint16_t readSPIADC(int channel=0) {
  // 1200000 is a good speed if we lower the pin impedance (<= ~10K ohm)
  // 250000 because 5RC for the sample capacitor is 23K input impedance and 20pf cap
  // So 5RC is 3.3us with RC constant of 660ns
  // The capacitor needs to charge in at least one clock cycle so at 250k each cycle is
  // 4us and should give enough time for the cap to charge fully

  int spiHz = 500000;//1200000;//250000; /* Seems it doesn't it wants to take longer, so a slower clock */
  SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  mcp.digitalWrite(ADC_CS_PIN, LOW);

  delay(1);

  // Channel 0 by default
  uint8_t cmd = 0b01101000;

  // Channel 1 will do a different command
  if (channel) {
    cmd = 0b01111000;
  }

  // Do one read to allow a lock and stabalize
  byte msb = SPI.transfer(cmd);
  byte lsb = SPI.transfer(0);

  uint16_t result = ((msb << 8) | lsb) & 0b0000001111111111;

  // DEBUG_PRINT("Reading ADC Channel ");
  // DEBUG_PRINT(channel);
  // DEBUG_PRINT(": ");
  // DEBUG_PRINTLN(result);

  mcp.digitalWrite(ADC_CS_PIN, HIGH);
  SPI.endTransaction();

  return result;
}

float readBatteryVoltage() {
  // Disable the solar panel to not interfere with the readings
  mcp.digitalWrite(SOLAR_ENABLE_PIN, LOW);
  delay(10);

  // Switch on the voltage divider for the battery and charge the cap
  mcp.digitalWrite(BAT_ENABLE_PIN, HIGH);
  delay(10); // Wait a bit

  // Get a reading to keep
  int reading = readSPIADC(0);

  // Vin voltage
  int refV = 3308;//3230;
  float dividerRatio = 1.333333334; // R2/R1 3.008/1.002   //1.3037809648;
  float batteryVoltage = (reading * dividerRatio) * (refV / 1023.0);

  // Turn the divider back off
  mcp.digitalWrite(BAT_ENABLE_PIN, LOW);
  delay(1);

  // Turn the solar panel back on
  mcp.digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  // Output
  DEBUG_PRINT("Battery Voltage: ");
  DEBUG_PRINTLN(batteryVoltage);

  return batteryVoltage;
}

int readWindVane() {
  float windVaneReading = readSPIADC(WIND_VANE_PIN);
  int windDegrees = map(windVaneReading, 2, 991, 0, 360);

  DEBUG_PRINT("Wind Pin Reading: ");
  DEBUG_PRINTLN(windVaneReading);
  DEBUG_PRINT("Wind Direction: ");
  DEBUG_PRINT(windDegrees);
  DEBUG_PRINTLN(" degrees");

  return windDegrees;
}

void do_mcp_isr() {
  uint8_t activeInterrupts = mcp.readInterrupts();

  if (activeInterrupts>>ANEMOMETER_PIN & 1 == 1) {
    anemometerSampleCount++;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);

  // Wait for serial to initalize
  while (!Serial) {}

  // Use D3 as SDA and D1 as SCL (GPIO0, GPIO5) respectivley
  Wire.begin(D3, D1);

  // mcp23008 begin
  mcp.begin();      // use default address 0

  // Set the interrupt pin to ACTIVE HIGH
  mcp.setInterruptOutPinMode(MCP23008_INT_OUT_HIGH);

  // Attach our handler to the interrupt trigger pin with an interrupt
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), do_mcp_isr, RISING);

  // SPI init
  SPI.begin();
  mcp.pinMode(ADC_CS_PIN, OUTPUT);

  // LEDs
  mcp.pinMode(OK_LED_PIN, OUTPUT);
  mcp.pinMode(ERROR_LED_PIN, OUTPUT);
  mcp.digitalWrite(OK_LED_PIN, HIGH);
  mcp.digitalWrite(ERROR_LED_PIN, HIGH);


  // Clock
  RTC.begin();

  // Check to see if the RTC is keeping time.  If it is, load the time from your computer.
  if (!RTC.isrunning()) {
    DEBUG_PRINTLN("RTC is NOT running!");
    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  //******************************
  //* WiFi Enable light_sleep
  //******************************
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  //wifi_set_sleep_type(LIGHT_SLEEP_T);

  //******************************
  //* Enable the solar panel
  //******************************
  mcp.pinMode(SOLAR_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  //******************************
  //* Init battery reading if enabled instead of vcc reading
  //******************************
  mcp.pinMode(BAT_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(BAT_ENABLE_PIN, LOW);

  //******************************
  //* Init Wind Vane
  //******************************
  //pinMode(WIND_VANE_ENABLE_PIN, OUTPUT);
  //digitalWrite(WIND_VANE_ENABLE_PIN, HIGH);

  //******************************
  //* Start Anemometer Counting
  //******************************
  // Enable interrupt for pin 2
  mcp.pinMode(ANEMOMETER_PIN, INPUT);
  mcp.enableInterrupt(ANEMOMETER_PIN, RISING);

  anemometerSampleCount = 0;
  lastAnemometerSamplePrint = millis();

  //******************************
  //* Start BME sensor
  //******************************
  bmeConnected = bmeSensor.begin(0x76);
  delay(100); // Wait for sensor to boot (initalize)

  mcp.digitalWrite(OK_LED_PIN, LOW);
  mcp.digitalWrite(ERROR_LED_PIN, LOW);
}

void loop() {
  if (millis()-lastSenseTime < NO_SLEEP_READING_DELAY) {
    return;
  }

  lastSenseTime = millis();

  DEBUG_PRINT("Starting Sensing on Sample Index: ");
  DEBUG_PRINTLN(readingIndex);

  // First reset any error lights from last runs then turn on the reading led
  mcp.digitalWrite(ERROR_LED_PIN, LOW);
  mcp.digitalWrite(OK_LED_PIN, HIGH);

  //******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  DEBUG_PRINTLN("Sensing BME");

  float bmeTemp = NAN;
  float bmePressure = NAN;
  float bmeAltitude = NAN;
  float bmeHumidity = NAN;

  if (bmeConnected) {
    bmeTemp = bmeSensor.readTemperature();
    bmePressure = bmeSensor.readPressure();
    bmeAltitude = bmeSensor.readAltitude(SEALEVELPRESSURE);
    bmeHumidity = bmeSensor.readHumidity();

    DEBUG_PRINT("BME Temp: ");
    DEBUG_PRINT(bmeTemp);
    DEBUG_PRINT(" Pressure: ");
    DEBUG_PRINT(bmePressure);
    DEBUG_PRINT(" Humidity: ");
    DEBUG_PRINTLN(bmeHumidity);
  } else {
    DEBUG_PRINTLN("BME Disconnected");
  }

  //******************************************/
  //* Battery block
  //******************************************/
  float batteryVoltage = readBatteryVoltage();

  //******************************************/
  //* WIND VANE BLOCK
  //******************************************/
  int windDegrees = readWindVane();

  //******************************************/
  //* ANEMOMETER BLOCK
  //******************************************/
  DEBUG_PRINTLN("Sensing Anemometer");

  // Delay longer if it hasn't been 30 seconds
  // Samples will count using interrupts
  long msToDelayAnemometer = ANEMOMETER_SAMPLE_LENGTH-(millis()-lastAnemometerSamplePrint);

  if (msToDelayAnemometer > 0) {
    DEBUG_PRINT("Sampling anemometer for an additional ");
    DEBUG_PRINT(msToDelayAnemometer/1000);
    DEBUG_PRINTLN(" seconds.");

    waitMs(msToDelayAnemometer);
  }

  int millisNow = millis();
  int numTimelyAnemometerSamples = anemometerSampleCount;

  // Rpm is *.5 because there's 2 samples per revolution
  // It needs to stay as rpm so the server can calibrate itself?
  // Unless we output kph then we can convert to mph and have the calibration data stored on the esp8266 in this code here?
  // int calibrationCoeficient = 0.313; // Maybe something like this?
  // What if it's not a linear correlation?
  // round(samples*.5*minutes)
  int anemometerRpm = floor((numTimelyAnemometerSamples * 0.5 * (60000.0 / (millisNow-lastAnemometerSamplePrint))) + 0.5);

  DEBUG_PRINT(numTimelyAnemometerSamples);
  DEBUG_PRINT(" Samples in last ");
  DEBUG_PRINT((millisNow-lastAnemometerSamplePrint)/1000.0);
  DEBUG_PRINT(" second ");
  DEBUG_PRINT(anemometerRpm);
  DEBUG_PRINTLN(" RPM");

  anemometerSampleCount -= numTimelyAnemometerSamples;
  lastAnemometerSamplePrint = millisNow;

  //*****************/
  //* STORAGE BLOCK */
  //*****************/
  // Get the block storing the reading we want to modify
  WeatherReading *currentReading = &storedReadings[readingIndex++];

  DateTime now = RTC.now();
  currentReading->timestamp = now.unixtime();

  // For displaying with timezone offset set to -6 CST by default for now maybe needs a define
  now = DateTime(now + TimeSpan(60*60*GMT_OFFSET));

  DEBUG_PRINT("Timestamp: ");
  DEBUG_PRINT(currentReading->timestamp, DEC);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(now.month(), DEC);
  DEBUG_PRINT('/');
  DEBUG_PRINT(now.day(), DEC);
  DEBUG_PRINT('/');
  DEBUG_PRINT(now.year(), DEC);
  DEBUG_PRINT(' ');
  DEBUG_PRINT(now.hour(), DEC);
  DEBUG_PRINT(':');
  DEBUG_PRINT(now.minute(), DEC);
  DEBUG_PRINT(':');
  DEBUG_PRINT(now.second(), DEC);
  DEBUG_PRINTLN();

  currentReading->temperature = bmeTemp;
  currentReading->humidity = bmeHumidity;
  currentReading->pressure = bmePressure;
  currentReading->battery = batteryVoltage;
  currentReading->windSpeed = anemometerRpm;
  currentReading->windDirection = windDegrees;

  currentReading->populated = true;

  // If we've reached the last reading then goto the beginning again
  if (readingIndex >= MAX_READING_STORAGE) {
    readingIndex = 0;
  }


  //****************/
  //* OUTPUT BLOCK */
  //****************/
  // Count the attemts to wait at least serverDownCounter attempts before submitting again
  if (serverDownCounter > 0) {
    serverDownCounter--;
    DEBUG_PRINT("Server down counter is now: ");
    DEBUG_PRINTLN(serverDownCounter);
  // All good then submit
  } else if (readingIndex == 0 || readingIndex >= SUBMIT_MIN_READINGS) {
    DEBUG_PRINT("Starting Submitting Readings");
    submit_stored_readings();
  }


  ///////
  // Done now
  mcp.digitalWrite(OK_LED_PIN, LOW);
  mcp.digitalWrite(ERROR_LED_PIN, LOW);

  DEBUG_PRINT("Free Heap: ");
  DEBUG_PRINTLN(ESP.getFreeHeap(), DEC);
  DEBUG_PRINTLN("Sleeping Now");
  DEBUG_PRINTLN();

  #if ENABLE_DEEP_SLEEP
  ESP.deepSleep(DEEP_SLEEP_LENGTH);
  #else
  waitMs(NO_SLEEP_READING_DELAY);
  #endif
}
