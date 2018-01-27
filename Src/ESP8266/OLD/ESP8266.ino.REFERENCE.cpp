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
#define DEBUG 3

#if DEBUG
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// Includes
#include <Wire.h>
#include <SPI.h>

#include <ESP8266WiFi.h>

#include "./config.h"
#include "./helpers.h"

#include "RTClib.h"
#include "./Adafruit_MCP23008.h"
#include <Adafruit_BME280.h>

// Esp Pin Assignments
#define SW_SERIAL_TX_PIN D1
#define SW_SERIAL_RX_PIN D2
#define SDA_PIN D3 // GPIO0 /*D1 // GPIO5*/
#define SCL_PIN D4 // GPIO2
#define SPI_CLOCK_PIN D5
#define SPI_MOSI_PIN D6
#define SPI_MISO_PIN D7

// IO Expander Pin Assignments
#define ERROR_LED_PIN 0
#define OK_LED_PIN 1
#define REFV_ENABLE_PIN 2
#define SOLAR_ENABLE_PIN 3
#define BAT_DIV_ENABLE_PIN 4
//#define REFV_ENABLE_PIN 5
#define ATTINY_DATA_REQUEST_PIN 6
#define ADC_CS_PIN 7

// ADC Pin Assignments
#define ADC_REF_PIN 0
#define WIND_VANE_PIN 6
#define BAT_ADC_PIN 7

// Flags
  // Reading settings
#define READING_INTERVAL 300000 // 300000 is 5 minutes in ms
#define SAMPLE_INTERVAL 2000
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL // How many samples to average for a reading 300000/2000 = 30 for one every 2 seconds
#define BATTERY_SAMPLE_MODULO 20 // Every X samples check the battery

#define MAX_READING_STORAGE 288//576 // How many readings to store in memory; 288 with 300000 ms samples (5 min) is 24 hours; 576 = 48 hours
#define READING_SUBMIT_INTERVAL 1 // How many to have before attempting to submit
#define MAX_FAILED_SUBMITS 3

#define SAMPLE_INDICATOR_LED_ENABLED true // Will flash each sample

// Instrument settings
// Anemometer
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph

// Wind Vane
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

// Globals
unsigned long lastSampleMillis =0;

// Init structures
RTC_DS1307 RTC; // Real Time Clock
Adafruit_MCP23008 mcp; // IO Expander
Adafruit_BME280 bmeSensor; // Termometer, Hygrometer, Barometer

// // Serial connection to ATtiny
// #include <SoftwareSerial.h>
// SoftwareSerial ATtinySerial(SW_SERIAL_RX_PIN, -1);

void setup() {
  Serial.begin(19200);

	// SoftwareSerial
	//ATtinySerial.begin(9600);
}








void loop() {
	while (ATtinySerial.available()) {
		Serial.write(ATtinySerial.read());
	}

	if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
		lastSampleMillis = millis();

		// Send it to attiny for storage
		ATtinySerial.println("Frogs");

		Serial.println("Sleeping Now.");
	}
}


// WiFi
#include <ESP8266WiFi.h>

// Required for LIGHT_SLEEP_T delay mode
extern "C" {
#include "user_interface.h"
}

// WiFi and Network Variables and GMT offset
#include "./config.h"

// Helpers
#include "./helpers.h"

// Sensor Includes
#include "RTClib.h"

#include "./Adafruit_MCP23008.h"
#include <Adafruit_Sensor.h>

#if BME_SENSOR == SENS_BME280
#include <Adafruit_BME280.h>
#endif

#if BME_SENSOR == SENS_BMP180
#include <Adafruit_BMP085.h>
#endif

// Wifi Variables
IPAddress wifi_ip(MY_IP);
IPAddress wifi_gateway(GATEWAY_IP);
IPAddress wifi_subnet(SUBNET_MASK);

#define DEEP_SLEEP_MODE 0
#define COMPANION_MODE 1
#define CALIBRATION_MODE 0

#if CALIBRATION_MODE
int CALIB_windVaneMax = 0;
int CALIB_windVaneMin = 1024;
#endif

// Config
#define READING_INTERVAL 300000 // 300000 is 5 minutes in ms
#define SAMPLE_INTERVAL 2000
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL // How many samples to average for a reading 300000/2000 = 30 for one every 2 seconds
#define BATTERY_SAMPLE_MODULO 20 // Every X samples check the battery

#define MAX_READING_STORAGE 288//576 // How many readings to store in memory; 288 with 300000 ms samples (5 min) is 24 hours; 576 = 48 hours
#define READING_SUBMIT_INTERVAL 1 // How many to have before attempting to submit
#define MAX_FAILED_SUBMITS 3

#define SAMPLE_INDICATOR_LED_ENABLED true // Will flash each sample

// Esp Pin Assignments
#define SW_SERIAL_TX_PIN D1
#define SW_SERIAL_RX_PIN D2
#define SDA_PIN D3 // GPIO0 /*D1 // GPIO5*/
#define SCL_PIN D4 // GPIO2
#define SPI_CLOCK_PIN D5
#define SPI_MOSI_PIN D6
#define SPI_MISO_PIN D7

// IO Expander Pin Assignments
#define ERROR_LED_PIN 0
#define OK_LED_PIN 1
#define REFV_ENABLE_PIN 2
#define SOLAR_ENABLE_PIN 3
#define BAT_DIV_ENABLE_PIN 4
//#define REFV_ENABLE_PIN 5
#define ATTINY_DATA_REQUEST_PIN 6
#define ADC_CS_PIN 7

// ADC Pin Assignments
#define ADC_REF_PIN 0
#define WIND_VANE_PIN 6
#define BAT_ADC_PIN 7

// Serial connection to ATtiny
#include <SoftwareSerial.h>
SoftwareSerial ATtinySerial(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN);
bool ATtinyNewline = true;

char attinyReplyBuffer[11]; // 1 + 4 + 1 + 4 + 1 == (~ Int1 ! Int2 \n)
short attinyReplyBufferIndex = 0;
bool attinyReplyBuffering = false;

/*********************
* Instrument Settings
*********************/
// Anemometer
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph

// Wind Vane
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

// Globals
unsigned long totalReadings = 0;
unsigned long powerOnTime;

WeatherReadingAccumulator *sampleAccumulator = new WeatherReadingAccumulator;
int numSamples = 0;

WeatherReading storedReadings[MAX_READING_STORAGE];
int readingIndex = 0;
short int serverDownCounter = 0;


int okLedState = HIGH;
unsigned long lastSampleMillis = 0;

// Real Time Clock
RTC_DS1307 RTC;

// IO Expander
Adafruit_MCP23008 mcp;

// Sensors
/**********/
// BME 280
#if BME_SENSOR == SENS_BME280
Adafruit_BME280 bmeSensor;
#endif

#if BME_SENSOR == SENS_BMP180
Adafruit_BMP085 bmeSensor;
#endif

bool bmeConnected = true;

unsigned long lastAnemometerReadingMillis = 0;
volatile unsigned int anemometerPulseCount = 0;
volatile unsigned long anemometerSampleSum = 0;
volatile unsigned int anemometerSampleCount = 0;

volatile bool hasInterrupt = false;

void print_raw_attiny_buffer() {
	Serial.print("Recieved from ATtiny Raw: ");

	for (int i=0;i<attinyReplyBufferIndex;i++) {
		for (int bit=7;bit>=0;bit--) {
			Serial.print((attinyReplyBuffer[i]>>bit)&1);
		}

		Serial.print(" ");
	}

	Serial.println();

	Serial.print("Nice:  ");
	Serial.println(attinyReplyBuffer);
}

bool recv_attiny_serial() {
	// Print attiny serial messages
	while (ATtinySerial.available() > 0) {
		uint8_t rxByte = ATtinySerial.read();

		if (ATtinyNewline) {
			ATtinyNewline = false;

			if (!attinyReplyBuffering) {
				Serial.write("[ATtiny]");
			}
		}

		// Print it if it's not a reply to us
		if (!attinyReplyBuffering) {
			Serial.write(rxByte);
		}

		if (rxByte == 126) {
			attinyReplyBufferIndex = 0;
			attinyReplyBuffering = true;
		} else if (rxByte == 10) {
			ATtinyNewline = true;
			attinyReplyBuffering = false;
		}

		if (attinyReplyBuffering && attinyReplyBufferIndex < 5) {
			attinyReplyBuffer[attinyReplyBufferIndex++] = rxByte;
		}
	}
}

bool wait_attiny_data(int timeoutMs = 50) {
	attinyReplyBuffering = true;
	int totalDelayMs = 0;

	do {
		recv_attiny_serial();
		delay(1);
		totalDelayMs++;

		if (totalDelayMs >= timeoutMs) {
			break;
		}
	} while (attinyReplyBuffering);

	// If we timed out
	if (totalDelayMs >= timeoutMs) {
		return false;
	}

	return true;
}

bool connectToWiFi() {
	// If we're connected then return
	if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN("Already Connected");
		return true;
	}

  //******************************
  //* Init WiFi
  //******************************
  //noInterrupts();
	WiFi.forceSleepWake();
	//interrupts();
  delay(1);

	//WiFi.persistent(false);

	Serial.print("Connecting to: ");
	Serial.println(WIFI_SSID);

	WiFi.mode(WIFI_STA);

	WiFi.config(wifi_ip, wifi_gateway, wifi_subnet);

	//noInterrupts();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  //interrupts();


	// Wait for a connection
  unsigned int totalWaitTime = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    totalWaitTime += 50;
    okLedState = !okLedState;
    mcp.digitalWrite(OK_LED_PIN, okLedState ? HIGH : LOW);
    DEBUG_PRINT(".");

    // Wait 30 secs and give up
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

  // //noInterrupts(); // Seems WiFi.disconnect also calls ETS_UART_INTR_DISABLE maybe this can be below it
  // bool disconnected = WiFi.disconnect(true);
	// //interrupts();
	//WiFi.mode(WIFI_OFF);

	//noInterrupts();
  WiFi.forceSleepBegin(); // This'll call mode(WIFI_OFF)
  //interrupts();

	delay(1);

  DEBUG_PRINT("Disconnected from wifi: ");
  //DEBUG_PRINTLN(disconnected);
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
		WiFiClient client;

    Serial.print(ip);
    Serial.print(":");
    Serial.print(port);
    Serial.println(outputUrl);

		if (client.connect(ip, port)) {
			Serial.println("Connection Successful!");

			client.print(String("GET ") + outputUrl + " HTTP/1.1\r\n" +
				"Host: " + ip + "\r\n" +
				"Connection: close\r\n\r\n");

				while (client.connected())
				{
				  if (client.available())
				  {
				    String line = client.readStringUntil('\n');
				    Serial.println(line);
				  }
					yield();
				}

				client.stop();

				success = true;
		} else {
			Serial.println("Connection Failed!");
			client.stop();
		}

		// HTTPClient http;
    //
    // //noInterrupts();
    // http.begin(ip, port, outputUrl); //HTTP
    // //interrupts();
    //
    // Serial.println("Success with http.begin");
    //
    // ////noInterrupts();
    // int httpCode = http.GET();
    // ////interrupts();
    //
    // Serial.print("Got http code: ");
    // Serial.println(httpCode);
    //
    // if(httpCode > 0) {
    //   if(httpCode == HTTP_CODE_OK) {
    //     String payload = http.getString();
    //
    //     Serial.println("Got payload: ");
    //     Serial.println(payload);
    //
    //     if (payload.startsWith("S")) {
    //       if (updateClock) {
    //         // Get the new time
    //         strtok((char*)payload.c_str(), "\n");
    //         char *newTime = strtok(NULL, "\n");
    //         int intTime = atoi(newTime);
    //
    //         Serial.print("Success! Also, setting clock to ");
    //         Serial.println(newTime);
    //
    //         RTC.adjust(DateTime(intTime));
    //       } else {
    //         Serial.print("Success!");
    //       }
    //
    //       success = true;
    //     } else {
    //       Serial.println("Failure: Invalid payload");
    //     }
    //   } else {
    //     Serial.print("Http Error: ");
    //     Serial.println(httpCode);
    //   }
    // } else {
    //   Serial.print("Http Error: ");
    //   Serial.println(httpCode);
    // }
    //
    // //noInterrupts();
    // http.end();
    // //interrupts();
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





void ICACHE_RAM_ATTR do_mcp_isr() {
	hasInterrupt = true;
}

void setup() {
  Serial.begin(19200);
  //Serial.setTimeout(500);
	Serial.setDebugOutput(true);

	// Maybe store this ?
	Serial.println("");
	Serial.println("Reset Reason:");
	Serial.println(ESP.getResetReason());
	Serial.println("Reset Info:");
	Serial.println(ESP.getResetInfo());
	Serial.println("Wifi Diagnosis:");
	WiFi.printDiag(Serial);

  // Wait for serial to initalize
  while (!Serial) {}

	struct rst_info *rtc_info = system_get_rst_info();

	Serial.printf("reset reason: %x\n", rtc_info->reason);

	if (rtc_info->reason == REASON_WDT_RST ||
		rtc_info->reason == REASON_EXCEPTION_RST ||
		rtc_info->reason == REASON_SOFT_WDT_RST) {
			if (rtc_info->reason == REASON_EXCEPTION_RST) {
				Serial.printf("Fatal exception (%d):\n", rtc_info->exccause);
			}
			Serial.printf("epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x\n",
				rtc_info->epc1, rtc_info->epc2, rtc_info->epc3, rtc_info->excvaddr, rtc_info->depc);//The address of the last crash is printed, which is used to debug garbled output.
	}

	// SoftwareSerial
	ATtinySerial.begin(300);

	// Wire.begin(SDA, SCL);
	Wire.begin(SDA_PIN, SCL_PIN);
	//Wire.setClockStretchLimit(1600); // Maybe for attiny

  // mcp23008 begin
  mcp.begin();      // use default address 0

	// We'll have this as input until we need a reading so we don't interfere with attiny programming
	mcp.pinMode(ATTINY_DATA_REQUEST_PIN, INPUT);

  // Set the interrupt pin mode
  //mcp.setInterruptOutPinMode(MCP23008_INT_OUT_HIGH);

	// Attach our handler to the interrupt trigger pin with an interrupt
	//attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), do_mcp_isr, RISING);

	//gpio_pin_wakeup_enable(INTERRUPT_PIN, GPIO_PIN_INTR_HILEVEL);

  // SPI init
	// D5 Clock
	// D6 MOSI
	// D7 MISO
  SPI.begin();
  mcp.pinMode(ADC_CS_PIN, OUTPUT);
	mcp.digitalWrite(ADC_CS_PIN, HIGH);

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

	// Store the power on time
	powerOnTime = RTC.now().unixtime();

  //******************************
  //* WiFi
  //******************************
	WiFi.persistent(false); // Don't store info to flash
	//noInterrupts();
	WiFi.forceSleepBegin(); // This'll call WiFi.mode(WIFI_OFF);
	//interrupts();
	//WiFi.mode(WIFI_OFF);
	delay(1);
  //wifi_set_sleep_type(LIGHT_SLEEP_T);

	//******************************
	//* For the reference voltage
	//******************************
	mcp.pinMode(REFV_ENABLE_PIN, OUTPUT);
	//mcp.digitalWrite(REFV_ENABLE_PIN, LOW);
	mcp.digitalWrite(REFV_ENABLE_PIN, HIGH);

  //******************************
  //* Enable the solar panel
  //******************************
  mcp.pinMode(SOLAR_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  //******************************
  //* Battery divider enable pin
  //******************************
  mcp.pinMode(BAT_DIV_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(BAT_DIV_ENABLE_PIN, LOW);

  //******************************
  //* Init Wind Vane
  //******************************
  //pinMode(WIND_VANE_ENABLE_PIN, OUTPUT);
  //digitalWrite(WIND_VANE_ENABLE_PIN, HIGH);

  //******************************
  //* Start Anemometer Counting
  //******************************
  // Enable interrupt for pin 2
  //mcp.pinMode(ANEMOMETER_PIN, INPUT);
  //mcp.enableInterrupt(ANEMOMETER_PIN, RISING);

  anemometerPulseCount = 0;
  lastAnemometerReadingMillis = millis();

  //******************************
  //* Start BME sensor
  //******************************
	#if BME_SENSOR == SENS_BME280
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
	#endif

	#if BME_SENSOR == SENS_BMP180
	bmeConnected = bmeSensor.begin();
	#endif


	// Init is done turn them back off
  mcp.digitalWrite(OK_LED_PIN, LOW);
  mcp.digitalWrite(ERROR_LED_PIN, LOW);

	//digitalWrite(AWAKE_PIN, LOW);
}

bool takeSample() {
	// Get the reading from the ATtiny
	mcp.pinMode(ATTINY_DATA_REQUEST_PIN, OUTPUT);
	mcp.digitalWrite(ATTINY_DATA_REQUEST_PIN, HIGH);
	delay(1);

	lastSampleMillis = millis();

	#if SAMPLE_INDICATOR_LED_ENABLED
	mcp.digitalWrite(OK_LED_PIN, HIGH);
	#endif

	DEBUG_PRINTLN();
	DEBUG_PRINT("Taking Sample Num: ");
	DEBUG_PRINT(numSamples+1);
	DEBUG_PRINT("/");
	DEBUG_PRINT(SAMPLES_PER_READING);
	DEBUG_PRINT(" for reading buffer index: ");
	DEBUG_PRINT(readingIndex);
	DEBUG_PRINT(" of total readings: ");
	DEBUG_PRINTLN(totalReadings);

	// Store the time
	lastSampleMillis = millis();

	//******************************************/
	//* Battery block
	//******************************************/
	// float batteryVoltage = readBatteryVoltage();
	if (numSamples % BATTERY_SAMPLE_MODULO == 0) {
		sampleAccumulator->battery += readBatteryVoltage();
		sampleAccumulator->numBatterySamples += 1;
	}

	//******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  float bmeTemp = NAN;
  float bmePressure = NAN;
  float bmeHumidity = NAN;

  if (bmeConnected) {
		for (int i=0; i<5; i++) {
			#if BME_SENSOR == SENS_BME280
			// Need this cause we'll be sleeping all the other time
			bmeSensor.takeForcedMeasurement();

			bmeTemp = bmeSensor.readTemperature();
			bmePressure = bmeSensor.readPressure();
			bmeHumidity = bmeSensor.readHumidity();

			if (!isnan(bmeTemp) && !isnan(bmePressure) && !isnan(bmeHumidity)) {
				break;
			}
			#endif

			#if BME_SENSOR == SENS_BMP180
			bmeTemp = bmeSensor.readTemperature();
			bmePressure = bmeSensor.readPressure();

			if (!isnan(bmeTemp) && !isnan(bmePressure)) {
				break;
			}
			#endif
		}
  }

  //******************************************/
  //* WIND VANE BLOCK
  //******************************************/
  int windDegrees = readWindVane();

  //******************************************/
  //* ATTINY INTERRUPT READINGS
  //******************************************/
	float anemometerMph = 0;

	bool recvSuccess = wait_attiny_data();
	int numTimelyAnemometerSamples = 0;

	if (recvSuccess) {
		// Get how many samples we had
		numTimelyAnemometerSamples = (int)attinyReplyBuffer[3];
	}

	// mph = revolutions * measurementDuration * calibrationData
	unsigned long millisNow = millis();
	anemometerMph = (numTimelyAnemometerSamples * 0.5) * (60000.0 / (millisNow-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;

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
	//sampleAccumulator->battery += batteryVoltage;
	sampleAccumulator->windSpeed += anemometerMph;
	sampleAccumulator->windDirection += windDegrees;

	#if DEBUG > 2
	WeatherReading sampleReading;

	DateTime now = RTC.now();
	sampleReading.timestamp = now.unixtime();
	sampleReading.temperature = bmeTemp;
	sampleReading.humidity = bmeHumidity;
	sampleReading.pressure = bmePressure;
	sampleReading.battery = sampleAccumulator->battery/float(sampleAccumulator->numBatterySamples);
	sampleReading.windSpeed = anemometerMph;
	sampleReading.windDirection = windDegrees;

	// Serial.print("Num samples: ");
	// Serial.println(sampleAccumulator->numBatterySamples);

	printWeatherReading(sampleReading);
	#endif

	// If we've reached the last reading then goto the beginning again
	if (numSamples >= SAMPLES_PER_READING) {
		numSamples = 0;
	}

	// Turn the light off again
	mcp.digitalWrite(ERROR_LED_PIN, LOW);
	mcp.digitalWrite(OK_LED_PIN, LOW);

	DEBUG_PRINT("Free Heap: ");
	DEBUG_PRINTLN(ESP.getFreeHeap(), DEC);

	mcp.digitalWrite(ATTINY_DATA_REQUEST_PIN, LOW);
	mcp.pinMode(ATTINY_DATA_REQUEST_PIN, INPUT);
}

void storeAveragedSamples() {
	// Output
	DEBUG_PRINTLN();
	DEBUG_PRINT("Averaging samples as reading: ");
	DEBUG_PRINTLN(readingIndex);

	WeatherReading *currentReading = &storedReadings[readingIndex++];
	totalReadings++; // Add one to the total readings

	// Save the time
	DateTime now = RTC.now();
	currentReading->timestamp = now.unixtime();

	// Divide the accumulator by the num samples for the average reading
	currentReading->temperature = sampleAccumulator->temperature/float(SAMPLES_PER_READING);
	currentReading->humidity = sampleAccumulator->humidity/float(SAMPLES_PER_READING);
	currentReading->pressure = sampleAccumulator->pressure/float(SAMPLES_PER_READING);
	if (sampleAccumulator->numBatterySamples) {
		currentReading->battery = sampleAccumulator->battery/float(sampleAccumulator->numBatterySamples);
	}
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


void save_sample_attiny() {
	//char serialized[] = serialize_sample(sampleAccumulator);
}

void loop() {
	/*******************************************************\
	|*
	|* For calibrating the wind vane when changing wires or whatever
	|*
	|*******************************************************/
	// #if CALIBRATION_MODE
	// int windVaneReading = readSPIADC(WIND_VANE_PIN);
  //
	// if (windVaneReading > CALIB_windVaneMax) {
	// 	CALIB_windVaneMax = windVaneReading;
	// }
  //
	// if (windVaneReading < CALIB_windVaneMin) {
	// 	CALIB_windVaneMin = windVaneReading;
	// }
  //
	// if (millis()-lastSampleMillis > 1000) {
	// 		Serial.print("Wind Vane Max: ");
	// 		Serial.println(CALIB_windVaneMax);
	// 		Serial.print("Wind Vane Min: ");
	// 		Serial.println(CALIB_windVaneMin);
	// 		lastSampleMillis = millis();
  //
	// }
  //
	// return;
	// #endif

	/*******************************************************\
	|*
	|* Companion mode with ATtiny
	|*
	|*******************************************************/
	#if COMPANION_MODE
	// Print what it prints through us
	recv_attiny_serial();

	if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
		lastSampleMillis = millis();
		// Take a weather reading sample
		//takeSample();

		// Send it to attiny for storage
		//save_sample_attiny();
		ATtinySerial.println("Frogs");
		ATtinySerial.flush(); // Make sure its transmitted

		DEBUG_PRINTLN("Sleeping Now.");
		//ESP.deepSleep(0);
	}

	#endif

	//ESP.deepSleep(0);
	//delay(5000);

	/*******************************************************\
	|*
	|* Standalone Mode
	|*
	|*******************************************************/
	// #else
	// if (millis()-lastSampleMillis < SAMPLE_INTERVAL) {
	// 	if (hasInterrupt) {
	// 		//noInterrupts();
	// 		hasInterrupt = false;
	// 		//interrupts();
  //
	// 		// if (mcp.interruptOn(ANEMOMETER_PIN)) {
	// 		//   anemometerPulseCount++;
	// 		// }
	// 	}
  //
	// 	delay(10);
	// 	return;
	// }
  //
	// Serial.println("Starting loop");
  //
	// #if SAMPLE_INDICATOR_LED_ENABLED
	// mcp.digitalWrite(OK_LED_PIN, HIGH);
	// #endif
  //
	// // Take a weather reading sample
	// takeSample();
  //
	// // If we're at the first sample then we've taken all of ours
	// if (numSamples == 0) {
	// 	storeAveragedSamples();
  //
	//   //****************/
	//   //* OUTPUT BLOCK */
	//   //****************/
	//   // Count the attemts to wait at least serverDownCounter attempts before submitting again
	//   if (serverDownCounter > 0) {
// 	//     serverDownCounter--;
// 	//     DEBUG_PRINT("Server down counter is now: ");
// 	//     DEBUG_PRINTLN(serverDownCounter);
// 	//   // All good then submit every READING_SUBMIT_INTERVAL
// 	// 	}
//   //
// 	// 	if (!serverDownCounter && readingIndex >= READING_SUBMIT_INTERVAL) {
// 	//     DEBUG_PRINT("Starting Submitting Readings");
// 	//     submit_stored_readings();
// 	//   }
// 	// }
//   //
// 	// // Turn the light off again
// 	// mcp.digitalWrite(ERROR_LED_PIN, LOW);
// 	// mcp.digitalWrite(OK_LED_PIN, LOW);
//   //
// 	// // It'll be negative if we've waited long enough or millis rolls over
// 	// int sleepLength = SAMPLE_INTERVAL-(millis()-lastSampleMillis);
//   //
// 	// DEBUG_PRINT("Free Heap: ");
// 	// DEBUG_PRINTLN(ESP.getFreeHeap(), DEC);
//   //
// 	// if (sleepLength > 0) {
// 	// 	DEBUG_PRINT("Sleeping For: ");
// 	// 	DEBUG_PRINT(sleepLength);
// 	// 	DEBUG_PRINTLN(" ms ");
//   //
// 	//   #if ENABLE_DEEP_SLEEP
// 	//   ESP.deepSleep(sleepLength*1000);
// 	//   #else
// 	// 	//delay(sleepLength);
// 	//   #endif
// 	// } else {
// 	// 	DEBUG_PRINTLN("Taking next reading directly.");
// 	// }
// 	// #endif
// }
