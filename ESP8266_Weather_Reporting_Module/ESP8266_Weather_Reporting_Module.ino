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

// WiFi and Network Variables and GMT offset
#include "./config.h"

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

#define READING_INTERVAL 300000 // 300000 is 5 minutes in ms
#define SAMPLE_INTERVAL 2000
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL // How many samples to average for a reading 300000/2000 = 30 for one every 2 seconds

#define MAX_READING_STORAGE 576 // How many readings to store in memory; 288 with 300000 ms samples (5 min) is 24 hours; 576 = 48 hours
#define READING_SUBMIT_INTERVAL 1 // How many to have before attempting to submit

#define MAX_FAILED_SUBMITS 3

#define N_ADC_SAMPLES 1

#define SAMPLE_INDICATOR_LED_ENABLED false // Will flash each sample

// Pin Assignments
#define OK_LED_PIN 1
#define ERROR_LED_PIN 0

#define SOLAR_ENABLE_PIN 4

#define BAT_ENABLE_PIN 3

#define INTERRUPT_PIN D2
#define ADC_CS_PIN 7

ADC_MODE(ADC_VCC);

// Globals
WeatherReading sampleReadings[SAMPLES_PER_READING];
int sampleIndex = 0;

WeatherReading storedReadings[MAX_READING_STORAGE];
int readingIndex = 0;
short int serverDownCounter = 0;


int okLedState = HIGH;
unsigned long lastSampleMillis = 9999999;

// Real Time Clock
RTC_DS1307 RTC;
// IO Expander
Adafruit_MCP23008 mcp;

// Sensors
/**********/
// BME 280
Adafruit_BME280 bmeSensor;
bool bmeConnected = true;

// Anemometer
// rpm/coef = wind speed in mph
// 10.2677201193868 = samples per 1mph
// Reciperical of it is 0.09739260404185239
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239
#define ANEMOMETER_PIN 2
unsigned long lastAnemometerReadingMillis = 0;

volatile unsigned int anemometerPulseCount = 0;

volatile unsigned long anemometerSampleSum = 0;
volatile unsigned int anemometerSampleCount = 0;

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
  //IPAddress ip(MY_IP);
  //IPAddress gateway(GATEWAY_IP);
  //IPAddress subnet(SUBNET_MASK);

  WiFi.forceSleepWake();
  delay(1);

  //WiFi.persistent(false);
  //WiFi.mode(WIFI_STA);

  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);

  //WiFi.config(ip, gateway, subnet);

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

  noInterrupts();
  bool disconnected = WiFi.disconnect();
  WiFi.forceSleepBegin();
  delay(1);
  interrupts();

  DEBUG_PRINT("Disconnected from wifi: ");
  DEBUG_PRINTLN(disconnected);
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

    Serial.println("Success with http.begin");

    noInterrupts();
    int httpCode = http.GET();
    interrupts();

    Serial.print("Got http code: ");
    Serial.println(httpCode);

    if(httpCode > 0) {
      if(httpCode == HTTP_CODE_OK) {
        String payload = http.getString();

        Serial.print("Got payload: ");
        Serial.println(payload);

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

  // Go through the readings and submit any that are there
  for (int i=0; i<MAX_READING_STORAGE; i++) {
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
			mcp.digitalWrite(ERROR_LED_PIN, HIGH);
      failedSubmissionIndexes[failedIndex++] = i;
      numConsecutiveFails++;
    } else {
      numConsecutiveFails = 0;
			mcp.digitalWrite(OK_LED_PIN, HIGH);
    }

    // If the index is also the number of fails
    // And if we've failed too many times in a row then quit
    // The server must be down
    if (i == numConsecutiveFails-1 && numConsecutiveFails > MAX_FAILED_SUBMITS) {
      DEBUG_PRINT("Aborting submission, the server must be down. Counter is: ");
      serverDownCounter = READING_SUBMIT_INTERVAL; // Includes 0 so 4 is don't try for 4 times and try on the 5th
      DEBUG_PRINTLN(serverDownCounter);

      disconnectFromWiFi();
      return;
    }

    // Don't want to overload (Need multi submit really)
    delay(50);

		// Turn the light off again
		mcp.digitalWrite(ERROR_LED_PIN, LOW);
		mcp.digitalWrite(OK_LED_PIN, LOW);
  }

  // Iterate through the reading indexes and if it's an index less than the failed count means it's a failed one
  // Get the associated index from the readings and copy its data over
  /* I think this is just changing the reference maybe ? */
  for (int i=0; i<MAX_READING_STORAGE; i++) {
    // If it is within the count of failed indexes we need to put storedReadings[failIndex[i]] at storedReadings[i]
    if (i < failedIndex) {
      //copyWeatherReading(source, dest);
      //storedReadings[i] = storedReadings[failedSubmissionIndexes[i]];
      //^Was like that before but after the server being down for a while when it finally came back up the esp only submitted the last few readings or random ones
      // No idea why should work according to tests on my desk. Maybe was passing reference so they got overwritten?
      copyWeatherReading(storedReadings[failedSubmissionIndexes[i]], storedReadings[i]);
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

  int spiHz = 900000;//1200000;//250000; /* Seems it doesn't it wants to take longer, so a slower clock */
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



float readSPIADCVoltage(int channel=0, float ratio=1.0) {
	float refMv = 0.0;
	float reading = 0.0;

	for (int i=0; i<N_ADC_SAMPLES; i++) {
		refMv += ESP.getVcc() * 1.1725;
		reading += readSPIADC(channel);

		// DEBUG_PRINT("Converging ref: ");
		// DEBUG_PRINTLN(refMv/(i+1));
		// DEBUG_PRINT("Converging vbat: ");
		// DEBUG_PRINTLN(reading/(i+1));

		delay(1);
	}


  refMv /= N_ADC_SAMPLES;//ESP.getVcc() * 1.1725; // Cause internally is connected to ground apparently 1.1?
  reading /= N_ADC_SAMPLES;//readSPIADC(channel);
  float resultMv = (refMv / 1023.0) * (reading * ratio);

	DEBUG_PRINT("ADC channel: ");
	DEBUG_PRINT(channel);
  DEBUG_PRINT(" refMv: ");
  DEBUG_PRINT(refMv);
  DEBUG_PRINT(" reading: ");
  DEBUG_PRINT(reading);
  DEBUG_PRINT(" resultMv: ");
  DEBUG_PRINTLN(resultMv);

  return resultMv;
}

float readBatteryVoltage() {
  float dividerRatio = 1.33511348465;//1.333333334; // R2/R1 3.008/1.002

  // Disable the solar panel to not interfere with the readings
  mcp.digitalWrite(SOLAR_ENABLE_PIN, LOW);
  delay(1);

  // Switch on the voltage divider for the battery and charge the cap
  mcp.digitalWrite(BAT_ENABLE_PIN, HIGH);
  delay(1); // Wait a bit

  // Voltage afterwards
  float batteryVoltage = readSPIADCVoltage(0, dividerRatio);

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

float readAnemometer() {
	DEBUG_PRINTLN("Sensing Anemometer");

  int millisNow = millis();

  // Get how many samples we had
  noInterrupts();
  int numTimelyAnemometerSamples = anemometerPulseCount;
  anemometerPulseCount = 0;
  interrupts();

	// mph = revolutions * measurementDuration * calibrationData
	float anemometerMph = (numTimelyAnemometerSamples * 0.5) * (60000.0 / (millisNow-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;

  DEBUG_PRINT(numTimelyAnemometerSamples);
  DEBUG_PRINT(" Samples in last ");
  DEBUG_PRINT((millisNow-lastAnemometerReadingMillis)/1000.0);
  DEBUG_PRINT(" second ");
  DEBUG_PRINT(anemometerMph);
  DEBUG_PRINTLN(" MPH");

  lastAnemometerReadingMillis = millisNow;

	return anemometerMph;
}

void ICACHE_RAM_ATTR do_mcp_isr() {
  if (mcp.interruptOn(ANEMOMETER_PIN)) {
    anemometerPulseCount++;
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

	// Turn them on for init
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
	IPAddress ip(MY_IP);
  IPAddress gateway(GATEWAY_IP);
  IPAddress subnet(SUBNET_MASK);

	WiFi.persistent(false); // Don't store info to flash
	WiFi.config(ip, gateway, subnet);
	WiFi.mode(WIFI_STA); // Station mode not AP

	WiFi.forceSleepBegin();
  //WiFi.mode(WIFI_OFF);
  //WiFi.forceSleepBegin();
  //delay(1);
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

  anemometerPulseCount = 0;
  lastAnemometerReadingMillis = millis();

  //******************************
  //* Start BME sensor
  //******************************
  bmeConnected = bmeSensor.begin(0x76);

	// Then set it to forced so we sleep
	if (bmeConnected) {
		bmeSensor.setSampling(
			bmeSensor.MODE_FORCED,
			bmeSensor.SAMPLING_X16,
			bmeSensor.SAMPLING_X16,
			bmeSensor.SAMPLING_X16,
			bmeSensor.FILTER_OFF,
			bmeSensor.STANDBY_MS_0_5
		);
	} else {
		Serial.println("BME280 sensor is not detected at i2caddr 0x76; check wiring.");
	}

	// Init is done turn them back off
  mcp.digitalWrite(OK_LED_PIN, LOW);
  mcp.digitalWrite(ERROR_LED_PIN, LOW);
}

bool takeSample() {
	DEBUG_PRINTLN();
	DEBUG_PRINT("Taking Sample on Index: ");
	DEBUG_PRINT(sampleIndex);
	DEBUG_PRINT("/");
	DEBUG_PRINTLN(SAMPLES_PER_READING-1);

	//******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  //DEBUG_PRINTLN("Sensing BME");

  float bmeTemp = NAN;
  float bmePressure = NAN;
  float bmeHumidity = NAN;

  if (bmeConnected) {

		for (int i=0; i<5; i++) {
			Serial.println("Sampling BME280");

			// Need this cause we'll be sleeping all the other time
			bmeSensor.takeForcedMeasurement();

			bmeTemp = bmeSensor.readTemperature();
			bmePressure = bmeSensor.readPressure();
			bmeHumidity = bmeSensor.readHumidity();

			if (!isnan(bmeTemp) && !isnan(bmePressure) && !isnan(bmeHumidity)) {
				break;
			}
		}
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
  float anemometerMph = readAnemometer();

  //*****************/
  //* STORAGE BLOCK */
  //*****************/
  // Get the block storing the reading we want to modify
  WeatherReading *currentReading = &sampleReadings[sampleIndex++];

  //DateTime now = RTC.now();
  //currentReading->timestamp = now.unixtime();
  currentReading->temperature = bmeTemp;
  currentReading->humidity = bmeHumidity;
  currentReading->pressure = bmePressure;
  currentReading->battery = batteryVoltage;
  currentReading->windSpeed = anemometerMph;
  currentReading->windDirection = windDegrees;

	//printWeatherReading(*currentReading);

	// If we've reached the last reading then goto the beginning again
	if (sampleIndex >= SAMPLES_PER_READING) {
		sampleIndex = 0;
	}
}

void storeAveragedSamples() {
	DEBUG_PRINTLN();
	DEBUG_PRINT("Averaging samples as reading: ");
	DEBUG_PRINTLN(readingIndex);

	WeatherReading *currentReading = &storedReadings[readingIndex++];

	// Save the time
	DateTime now = RTC.now();
	currentReading->timestamp = now.unixtime();

	// Average all the samples
	for (int i=0; i<SAMPLES_PER_READING; i++) {
		currentReading->temperature += sampleReadings[i].temperature;
		currentReading->humidity += sampleReadings[i].humidity;
		currentReading->pressure += sampleReadings[i].pressure;
		currentReading->battery += sampleReadings[i].battery;
		currentReading->windSpeed += sampleReadings[i].windSpeed;
		currentReading->windDirection += sampleReadings[i].windDirection;
	}

	currentReading->temperature /= SAMPLES_PER_READING;
	currentReading->humidity /= SAMPLES_PER_READING;
	currentReading->pressure /= SAMPLES_PER_READING;
	currentReading->battery /= SAMPLES_PER_READING;
	currentReading->windSpeed /= SAMPLES_PER_READING;
	currentReading->windDirection /= SAMPLES_PER_READING;

	currentReading->populated = true;

	printWeatherReading(*currentReading);

	// If we've reached the last reading then goto the beginning again
	if (readingIndex >= MAX_READING_STORAGE) {
		readingIndex = 0;
	}
}

void loop() {
  if (millis()-lastSampleMillis < SAMPLE_INTERVAL) {
		delay(1);
    return;
  }

	#if SAMPLE_INDICATOR_LED_ENABLED
	mcp.digitalWrite(OK_LED_PIN, HIGH);
	#endif

	// Take a weather reading sample
	lastSampleMillis = millis();
	takeSample();

	// If we're at the first sample then we've taken all of ours
	if (sampleIndex == 0) {
		storeAveragedSamples();

	  //****************/
	  //* OUTPUT BLOCK */
	  //****************/
	  // Count the attemts to wait at least serverDownCounter attempts before submitting again
	  if (serverDownCounter > 0) {
	    serverDownCounter--;
	    DEBUG_PRINT("Server down counter is now: ");
	    DEBUG_PRINTLN(serverDownCounter);
	  // All good then submit every READING_SUBMIT_INTERVAL
		}

		if (!serverDownCounter && readingIndex >= READING_SUBMIT_INTERVAL) {
	    DEBUG_PRINT("Starting Submitting Readings");
	    submit_stored_readings();
	  }
	}

	#if SAMPLE_INDICATOR_LED_ENABLED
	mcp.digitalWrite(OK_LED_PIN, LOW);
	#endif

	// It'll be negative if we've waited long enough or millis rolls over
	int sleepLength = SAMPLE_INTERVAL-(millis()-lastSampleMillis);

	DEBUG_PRINT("Free Heap: ");
	DEBUG_PRINTLN(ESP.getFreeHeap(), DEC);
	DEBUG_PRINT("Sleeping For: ");
	DEBUG_PRINT(sleepLength);
	DEBUG_PRINTLN(" ms ");

	if (sleepLength > 0) {
	  #if ENABLE_DEEP_SLEEP
	  ESP.deepSleep(sleepLength*1000);
	  #else
	  waitMs(sleepLength);
	  #endif
	}
}
