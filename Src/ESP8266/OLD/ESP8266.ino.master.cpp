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
#define CLEAN_START 0 // This will mark SRAM as unpopulated and update rtc to compile time

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
#define SRAM_CS_PIN 5
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
#define SAMPLES_PER_READING 5//READING_INTERVAL/SAMPLE_INTERVAL // How many samples to average for a reading 300000/2000 = 30 for one every 2 seconds
#define BATTERY_SAMPLE_MODULO 20 // Every X samples check the battery

// #define MAX_READING_STORAGE 576 // How many readings to store in memory; 288 with 300000 ms samples (5 min) is 24 hours; 576 = 48 hours
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
unsigned long totalReadings = 0; // How many readings since boot
WeatherReadingAccumulator sampleAccumulator;

uint32_t weatherReadingWriteIndex = 0;
uint32_t weatherReadingReadIndex = 0;

unsigned long lastSampleMillis = 0;
unsigned long lastAnemometerReadingMillis = 0;
bool bmeConnected = false;

// Init structures
RTC_DS1307 RTC; // Real Time Clock
Adafruit_MCP23008 mcp; // IO Expander
Adafruit_BME280 bmeSensor; // Termometer, Hygrometer, Barometer

// Sram, uses MCP as CS so needs to be here
#include "MCP_23A1024.h"

// // Serial connection to ATtiny
#include <SoftwareSerial.h>
SoftwareSerial ATtinySerial(SW_SERIAL_RX_PIN, -1);
bool ATtinyNewline = true;
char attinyReplyBuffer[11]; // 1 + 4 + 1 + 4 + 1 == (~ Int1 ! Int2 \n)
short attinyReplyBufferIndex = 0;
bool attinyReplyBuffering = false;

void setup() {
  Serial.begin(19200);

  // Wait for serial to initalize
  while (!Serial) {}

  Serial.println("\n");
  Serial.println("Initilizing ESP8266");

	// SoftwareSerial
	ATtinySerial.begin(9600);

  Wire.begin(SDA_PIN, SCL_PIN);

  // mcp23008 begin
  mcp.begin();      // use default address 0
  // Indicator LEDS
  mcp.pinMode(OK_LED_PIN, OUTPUT);
  mcp.pinMode(ERROR_LED_PIN, OUTPUT);
  // Turn leds on for init
  mcp.digitalWrite(OK_LED_PIN, HIGH);
  mcp.digitalWrite(ERROR_LED_PIN, HIGH);
  // Don't select SRAM
  mcp.pinMode(SRAM_CS_PIN, OUTPUT);
  mcp.pinMode(SRAM_CS_PIN, HIGH);
  // Don't select ADC
  mcp.pinMode(ADC_CS_PIN, OUTPUT);
	mcp.digitalWrite(ADC_CS_PIN, HIGH);
  // Turn off reference voltage
  mcp.pinMode(REFV_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(REFV_ENABLE_PIN, LOW);
  // Turn off battery divider
  mcp.pinMode(BAT_DIV_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(BAT_DIV_ENABLE_PIN, LOW);
  // Enable solar panel
  mcp.pinMode(SOLAR_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  // For the ADC
  SPI.begin();

  // Clock
  RTC.begin();

  // Check to see if the RTC is keeping time.  If it is, load the time from your computer.
  if (CLEAN_START || !RTC.isrunning()) {
    #if CLEAN_START
    DEBUG_PRINTLN("Syncing RTC to compile time (UTC)");
    #else
    DEBUG_PRINTLN("RTC is NOT running!");
    #endif

    // This will reflect the time that your sketch was compiled
    RTC.adjust(DateTime(DateTime(__DATE__, __TIME__) - TimeSpan(60*60*GMT_OFFSET)));
  }

  // BME280
  bmeConnected = bmeSensor.begin(0x76);

  // Then set it to forced so it sleeps between readings
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

  Serial.print("Max Readings Storable: ");
  Serial.println(SRAM_MAX_READINGS);

  // //int readIndex = 160;
  // WeatherReading currentReading;
  //
  // for (int readIndex=4;readIndex<5;readIndex++) {
  //   sram_read_reading(&currentReading, readIndex);
  //
  //   Serial.print("Weather Reading Index: ");
  //   Serial.println(readIndex);
  //   printWeatherReading(currentReading);
  // }
  // return;

  #if CLEAN_START
  Serial.println("Unpopulated!");

  sram_write(weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);
  sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
  sram_write_accumulator(sampleAccumulator);

  sram_set_populated();
  #else
  Serial.println("Previously Populated");

  sram_read(&weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);
  sram_read(&weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
  //TODO: For some reason this gave bogus values when reset on [Connecting to Wifi: ...

  sram_read_accumulator(&sampleAccumulator);

  if (sampleAccumulator.timestamp == 0) {
    Serial.println("Invalid accumulator; zeroing");
    zeroWeatherReading(&sampleAccumulator);
  }

  // Print the info
  Serial.print("Read index: ");
  Serial.println(weatherReadingReadIndex);
  Serial.print("Write index: ");
  Serial.println(weatherReadingWriteIndex);
  printWeatherReading(sampleAccumulator);
  #endif

  // Turn off init leds
  mcp.digitalWrite(OK_LED_PIN, LOW);
  mcp.digitalWrite(ERROR_LED_PIN, LOW);
}

void loop() {
  // Print what it prints through us
	//recv_attiny_serial();

  if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
    take_sample();
  }

  if (sampleAccumulator.numSamples >= SAMPLES_PER_READING) {
    // Store the sample accumulator as a reading and zero the accumulator
    WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
    zeroWeatherReading(&sampleAccumulator);
    sram_write_accumulator(sampleAccumulator);

    // Count it for debug printout purposes
    totalReadings++;

    // Store the reading in sram
    DEBUG_PRINTLN();
    DEBUG_PRINT("Storing samples as reading: ");
    DEBUG_PRINTLN(weatherReadingWriteIndex);
    sram_write_reading(currentReading, weatherReadingWriteIndex);
    weatherReadingWriteIndex++;

    printWeatherReading(currentReading);

    // For index overflow; back to beginning
    if (weatherReadingWriteIndex > SRAM_MAX_READINGS) {
      weatherReadingWriteIndex = 0;
    }

    sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);

    // Try and submit any readings
    submit_stored_readings();
  }
}

void submit_stored_readings() {
  int failedSubmits = 0;
  WeatherReading currentReading;

  Serial.println("[Starting Submit Stored Readings]");

  if (weatherReadingReadIndex == weatherReadingWriteIndex) {
    Serial.println("[Nothing to submit]");
    return;
  }

  while (failedSubmits < MAX_FAILED_SUBMITS) {
    sram_read_reading(&currentReading, weatherReadingReadIndex);

    Serial.print("Weather Reading Index: ");
    Serial.println(weatherReadingReadIndex);
    printWeatherReading(currentReading);

    noInterrupts();
    bool success = submit_reading(currentReading);
    interrupts();

    // Success then advance the read pointer
    if (success) {
      weatherReadingReadIndex++;

      if (weatherReadingReadIndex >= SRAM_MAX_READINGS) {
        weatherReadingReadIndex = 0;
      }

      // Store it
      sram_write(weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);

      // If we've submitted all then break
      if (weatherReadingReadIndex == weatherReadingWriteIndex) {
        break;
      }
    } else {
      failedSubmits++;

      Serial.print("Failed - Retries Left: ");
      Serial.println(MAX_FAILED_SUBMITS-failedSubmits);
    }
  }

  disconnect_from_wifi();
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

bool submit_reading(WeatherReading currentReading) {
  mcp.digitalWrite(OK_LED_PIN, HIGH);

  DEBUG_PRINTLN();
  DEBUG_PRINT("[Submitting Reading (");
  DEBUG_PRINT(weatherReadingReadIndex);
  DEBUG_PRINTLN(")]");

  bool success = false;
  char outputUrl[300] = "";

  // Make the url
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
  if (connect_to_wifi()) {
		WiFiClient client;
    client.setTimeout(1000);

    Serial.print("[Connecting to  ");
    Serial.print(ip);
    Serial.print(":");
    Serial.print(port);
    Serial.print(" ");

		if (client.connect(ip, port)) {
      Serial.println("Success!]");

      // Send the request
			Serial.println("[Sending Request]");
      Serial.println(outputUrl);

			client.print(String("GET ") + outputUrl + " HTTP/1.1\r\n" +
				"Host: " + ip + "\r\n" +
				"Connection: close\r\n\r\n");

      unsigned long timeout = millis();
      while (client.available() == 0) {
        if (millis()-timeout > 5000) {
          Serial.println("....Timed out [!]");
          client.stop();
          return false;
        }
      }

      Serial.println("\n[Response:]");

		  while (client.available())
		  {
        // It seems that readStringUntil blocks if it doesn't find the char again
		    String line = client.readStringUntil('\n');
		    Serial.println(line);
		  }
			success = true;

		} else {
			Serial.println("Failed]");
		}

    client.stop();
    Serial.println("\n[Closed Client Connection]");
  }

  mcp.digitalWrite(OK_LED_PIN, LOW);
  return success;
}

bool connect_to_wifi() {
  DEBUG_PRINT("[Connecting to Wifi: ");

  // If we're connected then return
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN("Already Connected]");
    return true;
  }


  IPAddress wifi_ip(MY_IP);
  IPAddress wifi_gateway(GATEWAY_IP);
  IPAddress wifi_subnet(SUBNET_MASK);

  WiFi.mode(WIFI_STA);
	WiFi.config(wifi_ip, wifi_gateway, wifi_subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  bool ledState = false;
  int timeout = 30000;
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    timeout -= 100;
    Serial.print('.');
    ledState = !ledState;
    mcp.digitalWrite(OK_LED_PIN, ledState);

    if (timeout <= 0) {
      DEBUG_PRINTLN("Timeout (!)]");
      mcp.digitalWrite(OK_LED_PIN, LOW);
      return false;
    }
  }

  mcp.digitalWrite(OK_LED_PIN, LOW);
  DEBUG_PRINTLN("Success]");
  return true;
}

bool disconnect_from_wifi() {
  DEBUG_PRINT("[Disconnecting From Wifi: ");
  bool result = WiFi.disconnect(true);
  DEBUG_PRINTLN(result?"Success]":"Failed (!)]");
}

void sram_restore_accumulator() {
  WeatherReadingAccumulator tmpAccum;
  sram_read_accumulator(&sampleAccumulator);

  printWeatherReading(get_averaged_accumulator(sampleAccumulator));
}

void sram_store_accumulator() {
  sram_write_accumulator(sampleAccumulator);
}

void take_sample() {
  lastSampleMillis = millis();

  #if SAMPLE_INDICATOR_LED_ENABLED
  mcp.digitalWrite(OK_LED_PIN, HIGH);
  #endif

  // Get the reading from the ATtiny
	mcp.pinMode(ATTINY_DATA_REQUEST_PIN, OUTPUT);
	mcp.digitalWrite(ATTINY_DATA_REQUEST_PIN, HIGH);
	delay(1);

	DEBUG_PRINTLN();
	DEBUG_PRINT("Taking Sample Num: ");
	DEBUG_PRINT(sampleAccumulator.numSamples+1);
	DEBUG_PRINT("/");
	DEBUG_PRINT(SAMPLES_PER_READING);
	DEBUG_PRINT(" Write index: ");
	DEBUG_PRINT(weatherReadingWriteIndex);
  DEBUG_PRINT(" Read index: ");
  DEBUG_PRINT(weatherReadingReadIndex);
	DEBUG_PRINT(" of total readings: ");
	DEBUG_PRINTLN(totalReadings);

  // DEBUG_PRINTLN("LAST SAMPLE:");
  // WeatherReadingAccumulator tmpAccum;
  // sram_read_accumulator(&tmpAccum);
  // WeatherReading tmpReadubg = get_averaged_accumulator(tmpAccum);
  // printWeatherReading(tmpReadubg);
  // DEBUG_PRINTLN("[END STORED READING]");
  // DEBUG_PRINTLN();

  //******************************************/
	//* Timestamp
	//******************************************/
  DateTime now = RTC.now();
  sampleAccumulator.timestamp = now.unixtime();

  //******************************************/
	//* Battery block
	//******************************************/
  if (sampleAccumulator.numSamples % BATTERY_SAMPLE_MODULO == 0) {
    sampleAccumulator.battery += readBatteryVoltage();
    sampleAccumulator.numBatterySamples += 1;
  }

  //******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  if (bmeConnected) {
    float bmeTemp = NAN;
    float bmePressure = NAN;
    float bmeHumidity = NAN;

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

    sampleAccumulator.temperature += bmeTemp;
    sampleAccumulator.humidity += bmeHumidity;
    sampleAccumulator.pressure += bmePressure;
  }

  //******************************************/
  //* WIND VANE BLOCK
  //******************************************/
  sampleAccumulator.windDirection += readWindVane();

  //******************************************/
  //* ATTINY INTERRUPT READINGS
  //******************************************/
  float anemometerMph = 0;
  int numTimelyAnemometerSamples = 0;

  // bool recvSuccess = wait_attiny_data();

  //
  // if (recvSuccess) {
  //   // Get how many samples we had
  //   numTimelyAnemometerSamples = (int)attinyReplyBuffer[3];
  // }
  //
  // // mph = revolutions * measurementDuration * calibrationData
  unsigned long millisNow = millis();
  // anemometerMph = (numTimelyAnemometerSamples * 0.5) * (60000.0 / (millisNow-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;

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

  // Print and stuff
  sampleAccumulator.numSamples++;

  #if DEBUG > 2
	WeatherReading sampleReading = get_averaged_accumulator(sampleAccumulator);
	printWeatherReading(sampleReading);
	#endif

  sram_store_accumulator();

  // zeroWeatherReading(&sampleAccumulator);
  // sram_read_accumulator(&sampleAccumulator);

  #if SAMPLE_INDICATOR_LED_ENABLED
  mcp.digitalWrite(OK_LED_PIN, LOW);
  #endif
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

float readSPIADCVoltage(int channel=0, float ratio=1.0, int offset=0, int numSamples=3) {
	unsigned long refPinReading = 0;
	unsigned long channelPinReading = 0;

	// Get the vcc mV
	//////////////////////////////
	float referencePinMv = 2495.5;//2500.0;

	mcp.digitalWrite(REFV_ENABLE_PIN, LOW);
	delay(10); // For refV to stabilize

	// Select what we want
	readSPIADC(ADC_REF_PIN);

	for (int i=0; i<numSamples; i++) {
		refPinReading += readSPIADC(ADC_REF_PIN);
	}

	refPinReading /= numSamples;

	mcp.digitalWrite(REFV_ENABLE_PIN, HIGH);

	float vccMv = (referencePinMv / refPinReading) * 1023;


	// Get the channel mV
	//////////////////////////////
	readSPIADC(channel);
	for (int i=0; i<numSamples; i++) {
		channelPinReading += readSPIADC(channel);
	}

	channelPinReading /= numSamples;

  float pinMv = (vccMv / 1023.0) * channelPinReading;
	float readingMv = pinMv * ratio;

	DEBUG_PRINT("ADC channel: ");
	DEBUG_PRINT(channel);
	DEBUG_PRINT(" refPin: ");
	DEBUG_PRINT(refPinReading);
  DEBUG_PRINT(" channelPin: ");
  DEBUG_PRINT(channelPinReading);
	DEBUG_PRINT(" vccMv: ");
	DEBUG_PRINT(vccMv);
	DEBUG_PRINT(" pinMv: ");
	DEBUG_PRINT(pinMv);
  DEBUG_PRINT(" readingMv: ");
  DEBUG_PRINT(readingMv);
	DEBUG_PRINT(" resultMv: ");
	DEBUG_PRINTLN(readingMv+offset);

  return readingMv + offset;
}

float readBatteryVoltage() {
	//1.33511348465; // R2/R1 3.008/1.002
  float dividerRatio = 1.335;

  // Disable the solar panel to not interfere with the readings
  mcp.digitalWrite(SOLAR_ENABLE_PIN, LOW);
  delay(10);

  // Switch on the voltage divider for the battery and charge the cap and stabilize reference voltage
  mcp.digitalWrite(BAT_DIV_ENABLE_PIN, HIGH);
  delay(20); // Wait a bit

  // Voltage afterwards
	float batteryVoltage = readSPIADCVoltage(BAT_ADC_PIN, dividerRatio, 15);

  // Turn the divider back off
  mcp.digitalWrite(BAT_DIV_ENABLE_PIN, LOW);
  //delay(1);

  // Turn the solar panel back on
  mcp.digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  // Output
  DEBUG_PRINT("Battery Voltage: ");
  DEBUG_PRINTLN(batteryVoltage);

  return batteryVoltage;
}

int readWindVane() {
	// Get average of 3
	int windVaneReading = (readSPIADC(WIND_VANE_PIN) + readSPIADC(WIND_VANE_PIN) + readSPIADC(WIND_VANE_PIN)) / 3.0;
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