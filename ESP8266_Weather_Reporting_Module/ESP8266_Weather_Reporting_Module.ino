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

#define DEBUG 2

#if DEBUG
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// WiFi
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include <StreamString.h>

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
//
// #define USE_ADAFRUIT_BME280 1
// #if USE_ADAFRUIT_BME280
#include <Adafruit_BME280.h>
// #else
// #include "libraries/BME280_driver/bme280.h"
// #endif

#define SEND_RESULT_STREAM 0

#define CALIBRATION_MODE 0
#if CALIBRATION_MODE
int CALIB_windVaneMax = 0;
int CALIB_windVaneMin = 1024;
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

#define SOLAR_ENABLE_PIN 3

#define BAT_ENABLE_PIN 4
#define BAT_ADC_PIN 7

#define INTERRUPT_PIN D2
#define ADC_CS_PIN 7

#define ADC_REF_PIN 0

// Anemometer
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph
#define ANEMOMETER_PIN 2

// Wind Vane
#define WIND_VANE_PIN 6
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

ADC_MODE(ADC_VCC);

// Globals
WeatherReadingDouble *sampleAccumulator = new WeatherReadingDouble;
int numSamples = 0;

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

unsigned long lastAnemometerReadingMillis = 0;
volatile unsigned int anemometerPulseCount = 0;
volatile unsigned long anemometerSampleSum = 0;
volatile unsigned int anemometerSampleCount = 0;

bool connectToWiFi() {
	// If we're connected then return
	if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN("Already Connected");
		return true;
	}

  //******************************
  //* Init WiFi
  //******************************
  WiFi.forceSleepWake();
  delay(1);

  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);

	WiFi.mode(WIFI_STA); // Station mode not AP

  noInterrupts();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  interrupts();

	// Wait for a connection
  unsigned int totalWaitTime = 0;

  while (WiFi.status() != WL_CONNECTED) {
    waitMs(500);
    totalWaitTime += 500;
    okLedState = !okLedState;
    mcp.digitalWrite(OK_LED_PIN, okLedState ? HIGH : LOW);
    DEBUG_PRINT(".");

    // Wait 10 secs and give up
    if (totalWaitTime > 30000) {
			okLedState = LOW;
			mcp.digitalWrite(OK_LED_PIN, LOW);

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
    return;
  }

  noInterrupts(); // Seems WiFi.disconnect also calls ETS_UART_INTR_DISABLE maybe this can be below it
  bool disconnected = WiFi.disconnect(true);
	interrupts();

	delay(1);

	noInterrupts();
  WiFi.forceSleepBegin(); // This'll call mode(WIFI_OFF)
  interrupts();

	delay(1);

  DEBUG_PRINT("Disconnected from wifi: ");
  DEBUG_PRINTLN(disconnected);
}

// #define SUBMISSION_URL "http://192.168.1.160"
//
// class ResultStream: public StreamString {
// private:
// 	int sampleIndex = 0;
//
// public:
// 	bool append_reading() {
// 		Serial.print("Current length: ");
// 		Serial.println(length());
//
// 		if (length()) {
// 			return true;
// 		}
//
// 		if (sampleIndex < MAX_READING_STORAGE) {
// 			if (storedReadings[sampleIndex].populated == false) {
// 				Serial.print("Unpopulated at index: ");
// 				Serial.println(sampleIndex);
// 				return false;
// 			}
//
// 			char buffer[20];
// 			sprintf(buffer, "%i,", storedReadings[sampleIndex].timestamp);
//
// 			Serial.print("Next buffer: ");
// 			Serial.println(String(buffer));
//
// 			sampleIndex++;
//
// 			concat(buffer);//this += buffer; //write((const uint8_t *)buffer, strlen(buffer));
// 			return true;
// 		}
// 		return false;
// 	}
//
// 	int available() {
// 		Serial.println("Checking available");
//
// 		// if (!append_reading() && !length()) {
// 		// 	return -1;
// 		// }
//
// 		return length();
// 	}
// };

#if SEND_RESULT_STREAM
void submit_readings() {
	if (connectToWiFi()) {
		WiFiClient client;
		char buffer[300] = {};

		//const char* host="http://192.168.1.160/";
		//String PostData = "title=foo&body=bar&userId=1";

		if (!client.connect(IPAddress(192, 168, 1, 160), 80)) {
			Serial.println("Connection Failed");
			return;
		}

		client.println("POST /submit_new.php HTTP/1.1");
		client.println("Host: 192.168.1.160");
		client.println("Accept: */*");
		client.println("Connection: close");
		//
		//client.println("Cache-Control: no-cache");
		//client.println("Content-Type: application/x-www-form-urlencoded");
		//client.println("Content-Length: 0");
		client.println();

		// Go through the readings and submit any that are there
		for (int i=0; i<MAX_READING_STORAGE; i++) {
			// If we've reached an unpopulated reading then we've reached the end and stop
			if (storedReadings[i].populated == false) {
				break;
			}

			// Need strcpy to init array at beginning for each reading
			strcpy(buffer, "timestamp=");
			strcat(buffer, String(storedReadings[i].timestamp).c_str());

			if (!isnan(storedReadings[i].temperature)) {
				strcat(buffer, "&temp=");
				strcat(buffer, String(storedReadings[i].temperature).c_str());
			}

			if (!isnan(storedReadings[i].humidity)) {
					strcat(buffer, "&humidity=");
					strcat(buffer, String(storedReadings[i].humidity).c_str());

					// If we have temperature and humidity then calculate and submit the heat index also
					if (!isnan(storedReadings[i].temperature)) {
						strcat(buffer, "&heatIndex=");
						strcat(buffer, String(computeHeatIndex(storedReadings[i].temperature, storedReadings[i].humidity, false)).c_str());
					}
			}

			if (!isnan(storedReadings[i].pressure)) {
				strcat(buffer, "&pressure=");
				strcat(buffer, String(storedReadings[i].pressure).c_str());
			}

			strcat(buffer, "&bat=");
			strcat(buffer, String(storedReadings[i].battery).c_str());
			strcat(buffer, "&windSpeed=");
			strcat(buffer, String(storedReadings[i].windSpeed).c_str());
			strcat(buffer, "&windDirection=");
			strcat(buffer, String(storedReadings[i].windDirection).c_str());


			Serial.print("Buffer: ");
			Serial.println(buffer);

			client.println(buffer);
		}

		// Output the response
		Serial.println("Result: ");

		while (client.available()) {
	    String line = client.readStringUntil('\r');
	    Serial.print(line);
		}

		Serial.println("End result");
	}
	// Serial.println("\n>>>> Submitting mass readings");
  //
	// HTTPClient httpClient;
  //
	// httpClient.begin("192.168.1.160", 80, "/report.php");//SERVER_IP_ADDRESS, 80, "/submit_new.php");
	// //http.setReuse();
	// //http.sendHeader('POST');
  //
	// Serial.println("Began httpClient");
  //
	// // Connect if needed
  // if (connectToWiFi()) {
	// 	// Serial.println("Sending Request!");
  //   //
	// 	// ResultStream dataStream;
	// 	// dataStream.concat("BALLS");
	// 	// int result = httpClient.sendRequest("POST", &dataStream, 0);
  //   //
	// 	// Serial.print("Http Code: ");
	// 	// Serial.println(result);
	// 	httpClient.sendHeader("POST");
  //
	// 	WiFiClient& httpStream = httpClient.getStream();
  //
	// 	char buffer[] = "Balls";
  //
	// 	//HTTP_TCP_BUFFER_SIZE
	// 	int bytesWrite = httpStream.write((const uint8_t *) buffer, strlen(buffer));
	// 	Serial.print("Wrote bytes: ");
	// 	Serial.println(bytesWrite);
  //
  //
	// 	// String payload = httpClient.getString();
  //   //
	// 	// Serial.println("Got payload: ");
	// 	// Serial.println(payload);
	//}
}
#endif


bool submit_reading(WeatherReading currentReading, bool disconnectAfterSubmission=true, bool updateClock=false) {
  bool success = false;

  char outputUrl[300] = "";

  strcat(outputUrl, "/report.php?");

  if (!isnan(currentReading.temperature)) {
    strcat(outputUrl, "temp=");
    strcat(outputUrl, String(currentReading.temperature).c_str());
    //
		// strcat(outputUrl, "&batPercent=");
	  // strcat(outputUrl, String(getBatteryPercent(currentReading.battery, currentReading.temperature)).c_str());
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

        Serial.println("Got payload: ");
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
			mcp.digitalWrite(ERROR_LED_PIN, HIGH);
      DEBUG_PRINTLN("Failed!, Caching for later.");
      failedSubmissionIndexes[failedIndex++] = i;
      numConsecutiveFails++;
    } else {
			mcp.digitalWrite(OK_LED_PIN, HIGH);
      numConsecutiveFails = 0;
    }

    // If the index is also the number of fails
    // And if we've failed too many times in a row then quit
    // The server must be down
    if (i == numConsecutiveFails-1 && numConsecutiveFails > MAX_FAILED_SUBMITS) {
			// Turn lamps off
			mcp.digitalWrite(OK_LED_PIN, LOW);
			mcp.digitalWrite(ERROR_LED_PIN, LOW);

      DEBUG_PRINT("Aborting submission, the server must be down. Counter is: ");
      serverDownCounter = READING_SUBMIT_INTERVAL; // Includes 0 so 4 is don't try for 4 times and try on the 5th
      DEBUG_PRINTLN(serverDownCounter);
      disconnectFromWiFi();
      return;
    }

    // Don't want to overload (Need multi submit really)
    delay(50);

		// Turn lamps off
		mcp.digitalWrite(OK_LED_PIN, LOW);
		mcp.digitalWrite(ERROR_LED_PIN, LOW);
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
    // Else flag it as not populated/zero it
    } else {
			zeroWeatherReading(&storedReadings[i]);
      //storedReadings[i].populated = false;
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
	// Only 8 channels
	if (channel > 7) {
		return 0;
	}

  int spiHz = 500000; /* Seems it doesn't it wants to take longer, so a slower clock */
  SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  mcp.digitalWrite(ADC_CS_PIN, LOW);

  delay(1);

	uint8_t cmd = 0b10000000 | channel<<4;

  // Read the sample from the adc
	SPI.transfer(0b00000001);
  byte msb = SPI.transfer(cmd);
  byte lsb = SPI.transfer(0);

  uint16_t result = ((msb << 8) | lsb) & 0b0000001111111111;

  mcp.digitalWrite(ADC_CS_PIN, HIGH);
  SPI.endTransaction();

  return result;
}

float readSPIADCVoltage(int channel=0, float ratio=1.0, int offset=0) {
	int referencePinMv = 2500;
	int refPinReading = readSPIADC(ADC_REF_PIN);
	float vccMv = (1023.0 / refPinReading) * referencePinMv;

	int channelPinReading = readSPIADC(channel);
  float pinMv = (vccMv / 1023.0) * channelPinReading;

	float readingMv = (pinMv * ratio) + offset;

	DEBUG_PRINT("ADC channel: ");
	DEBUG_PRINT(channel);
	DEBUG_PRINT(" refValue: ");
	DEBUG_PRINT(refPinReading);
  DEBUG_PRINT(" pinValue: ");
  DEBUG_PRINT(channelPinReading);
	DEBUG_PRINT(" vccMv: ");
	DEBUG_PRINT(vccMv);
	DEBUG_PRINT(" pinMv: ");
	DEBUG_PRINT(pinMv);
  DEBUG_PRINT(" readingMv: ");
  DEBUG_PRINTLN(readingMv);

  return readingMv;
}

float readBatteryVoltage() {
	//1.33511348465; // R2/R1 3.008/1.002
  float dividerRatio = 1.33511348465;//1.372997; // <-- As measured
	// Includes drop from FET and battery comtroller circuit

  // Disable the solar panel to not interfere with the readings
  mcp.digitalWrite(SOLAR_ENABLE_PIN, LOW);
  delay(10);

  // Switch on the voltage divider for the battery and charge the cap and stabilize reference voltage
  mcp.digitalWrite(BAT_ENABLE_PIN, HIGH);
  delay(100); // Wait a bit

  // Voltage afterwards
  float batteryVoltage = readSPIADCVoltage(BAT_ADC_PIN, dividerRatio);

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
  int windVaneReading = readSPIADC(WIND_VANE_PIN);
  int windDegrees = map(windVaneReading, WIND_VANE_MIN, WIND_VANE_MAX, 0, 360);

	#if DEBUG >= 3
  DEBUG_PRINT("Wind Pin Reading: ");
  DEBUG_PRINTLN(windVaneReading);
  DEBUG_PRINT("Wind Direction: ");
  DEBUG_PRINT(windDegrees);
  DEBUG_PRINTLN(" degrees");
	#endif

  return windDegrees;
}

float readAnemometer() {
  int millisNow = millis();

  // Get how many samples we had
  noInterrupts();
  int numTimelyAnemometerSamples = anemometerPulseCount;
  anemometerPulseCount = 0;
  interrupts();

	// mph = revolutions * measurementDuration * calibrationData
	float anemometerMph = (numTimelyAnemometerSamples * 0.5) * (60000.0 / (millisNow-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;

	#if DEBUG >= 2
	DEBUG_PRINT("Anemometer: ");
  DEBUG_PRINT(numTimelyAnemometerSamples);
  DEBUG_PRINT(" Samples in last ");
  DEBUG_PRINT((millisNow-lastAnemometerReadingMillis)/1000.0);
  DEBUG_PRINT(" second ");
  DEBUG_PRINT(anemometerMph);
  DEBUG_PRINTLN(" MPH");
	#endif

  lastAnemometerReadingMillis = millisNow;

	return anemometerMph;
}

void ICACHE_RAM_ATTR do_mcp_isr() {
  if (mcp.interruptOn(ANEMOMETER_PIN)) {
    anemometerPulseCount++;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(500);

	// Maybe store this ?
	//ESP.getResetReason();

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

	WiFi.forceSleepBegin(); // This'll call WiFi.mode(WIFI_OFF);
	delay(1);
  //wifi_set_sleep_type(LIGHT_SLEEP_T);

  //******************************
  //* Enable the solar panel
  //******************************
  mcp.pinMode(SOLAR_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  //******************************
  //* Battery divider enable pin
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
	DEBUG_PRINT("Taking Sample Num: ");
	DEBUG_PRINT(numSamples);
	DEBUG_PRINT("/");
	DEBUG_PRINTLN(SAMPLES_PER_READING-1);

	//******************************************/
	//* Battery block
	//******************************************/
	float batteryVoltage = readBatteryVoltage();

	//******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  float bmeTemp = NAN;
  float bmePressure = NAN;
  float bmeHumidity = NAN;

  if (bmeConnected) {
		for (int i=0; i<5; i++) {
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
  //* WIND VANE BLOCK
  //******************************************/
  int windDegrees = readWindVane();

  //******************************************/
  //* ANEMOMETER BLOCK
  //******************************************/
  float anemometerMph = readAnemometer();

	//float batteryPercent = getBatteryPercent(batteryVoltage, bmeTemp);
	//Serial.printf("Battery Percent: %s\n", String(batteryPercent).c_str());

  //*****************/
  //* STORAGE BLOCK */
  //*****************/
	numSamples++;
	//DateTime now = RTC.now();
	//sampleAccumulator->timestamp = now.unixtime();
	sampleAccumulator->temperature += bmeTemp;
	sampleAccumulator->humidity += bmeHumidity;
	sampleAccumulator->pressure += bmePressure;
	sampleAccumulator->battery += batteryVoltage;
	sampleAccumulator->windSpeed += anemometerMph;
	sampleAccumulator->windDirection += windDegrees;

	#if DEBUG
	WeatherReading sampleReading;

	DateTime now = RTC.now();
	sampleReading.timestamp = now.unixtime();
	sampleReading.temperature = bmeTemp;
	sampleReading.humidity = bmeHumidity;
	sampleReading.pressure = bmePressure;
	sampleReading.battery = batteryVoltage;
	sampleReading.windSpeed = anemometerMph;
	sampleReading.windDirection = windDegrees;

	printWeatherReading(sampleReading);
	#endif

	// If we've reached the last reading then goto the beginning again
	if (numSamples >= SAMPLES_PER_READING) {
		numSamples = 0;
	}
}

void storeAveragedSamples() {
	// Output
	DEBUG_PRINTLN();
	DEBUG_PRINT("Averaging samples as reading: ");
	DEBUG_PRINTLN(readingIndex);

	WeatherReading *currentReading = &storedReadings[readingIndex++];

	// Save the time
	DateTime now = RTC.now();
	currentReading->timestamp = now.unixtime();

	// Divide the accumulator by the num samples for the average reading
	currentReading->temperature = sampleAccumulator->temperature/float(SAMPLES_PER_READING);
	currentReading->humidity = sampleAccumulator->humidity/float(SAMPLES_PER_READING);
	currentReading->pressure = sampleAccumulator->pressure/float(SAMPLES_PER_READING);
	currentReading->battery = sampleAccumulator->battery/float(SAMPLES_PER_READING);
	currentReading->windSpeed = sampleAccumulator->windSpeed/float(SAMPLES_PER_READING);
	currentReading->windDirection = sampleAccumulator->windDirection/float(SAMPLES_PER_READING);
	currentReading->populated = true;

	zeroWeatherReading(sampleAccumulator);

	printWeatherReading(*currentReading);

	// If we've reached the last reading then goto the beginning again
	if (readingIndex >= MAX_READING_STORAGE) {
		readingIndex = 0;
	}
}

void loop() {
	#if CALIBRATION_MODE
	int windVaneReading = readSPIADC(WIND_VANE_PIN);

	if (windVaneReading > CALIB_windVaneMax) {
		CALIB_windVaneMax = windVaneReading;
	}

	if (windVaneReading < CALIB_windVaneMin) {
		CALIB_windVaneMin = windVaneReading;
	}

	if (millis()-lastSampleMillis > 1000) {
			Serial.print("Wind Vane Max: ");
			Serial.println(CALIB_windVaneMax);
			Serial.print("Wind Vane Min: ");
			Serial.println(CALIB_windVaneMin);
			lastSampleMillis = millis();

	}

	return;
	#endif

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

	#if SEND_RESULT_STREAM
	storeAveragedSamples();
	submit_readings();
	#else
	// If we're at the first sample then we've taken all of ours
	if (numSamples == 0) {
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
	#endif

	// Turn the light off again
	mcp.digitalWrite(ERROR_LED_PIN, LOW);
	mcp.digitalWrite(OK_LED_PIN, LOW);

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
