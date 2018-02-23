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
#include <avr/wdt.h>
#define WD_TIMEOUT WDTO_8S

#define CLEAN_START 0 // This will mark SRAM as unpopulated and update rtc to compile time
#define UPLOAD_TIME_OFFSET 0//53

#define SPI_HZ 500000 // 500Khz

// Sample and Reading defines
#define SAMPLE_INTERVAL 2000
#define READING_INTERVAL 300000 //SAMPLE_INTERVAL*5///300000 // 300000 = 5 minutes
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL
#define BATTERY_SAMPLE_MODULO 25
#define MAX_FAILED_SUBMITS 3
#define ESP_REQUEST_TIMEOUT 30 // In seconds


// Wind Vane Calibration
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

// Lux Calibration (Seems our dome blocks about 6.25% of light based off one uncontrolled rough test)
#define LUX_DOME_BLOCKING_COEF 1.0625

// Anemometer Calibration (Linear Association)
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph

// Pins
#define OK_LED_PIN 9     // PB1
#define ERROR_LED_PIN 10 // PB2
#define MOSI_PIN 11      // PB3
#define MISO_PIN 12      // PB4
#define SCK_PIN 13       // PB5

#define WIND_VANE_ADC_PIN A1 // PC1
#define BAT_ADC_PIN A2       // PC2
#define REF_ADC_PIN A3       // PC3+

#define RAIN_BUCKET_PIN 2    // PD2
#define ANEMOMETER_PIN 3     // PD3
#define ESP_RESET_PIN 4      // PD4
#define ESP_TX_PIN 5         // PD5
#define ESP_RX_PIN 6         // PD6
#define SRAM_CS_PIN 7        // PD7


// Mcp23008 pins
#define MCP_ESP_RESULT_PIN 0
#define MCP_ESP_SUCCESS_PIN 1

#define MCP_SOLAR_ENABLE_PIN 5
#define MCP_REFV_ENABLE_PIN 6
#define MCP_BAT_DIV_ENABLE_PIN 7

// Includes
#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <math.h>

#include "config.h"
#include "helpers.h"
#include "weather_readings.h"

#include "RTClib.h"
#include <Adafruit_BME280.h>
#include "MCP_23A1024.h"
#include "Adafruit_MCP23008.h"
#include "BH1750.h"

// Globals
uint32_t bootTime;

bool bmeConnected = false;

unsigned long lastSampleMillis = 0;

volatile unsigned int anemometerPulses = 0;
unsigned long lastAnemometerReadingMillis = 0;

volatile unsigned int rainBucketPulses = 0;
unsigned long lastRainBucketReadingMillis = 0;

WeatherReadingAccumulator sampleAccumulator;

uint8_t weatherReadingWriteIndex = 0;
uint8_t weatherReadingReadIndex = 0;

#define ESP_STATE_SLEEP 0
#define ESP_STATE_IDLE 1
#define ESP_STATE_AWAITING_RESULT 2
#define ESP_STATE_WAKING 3

unsigned short espState = ESP_STATE_SLEEP;

uint32_t startEspWaitTime = 0;

unsigned int failedSubmits = 0;
unsigned int submitTimeoutCountdown = 0;

// Structures/Classes
Adafruit_MCP23008 mcp; // IO Expander
RTC_DS1307 RTC; // Real Time Clock
Adafruit_BME280 bmeSensor; // Termometer, Hygrometer, Barometer
BH1750 luxMeter; // Lux

SoftwareSerial ESPSerial(ESP_RX_PIN, ESP_TX_PIN);

bool ESPNewline = true;
char ESPReplyBuffer[11]; // 1 + 4 + 1 + 4 + 1 == (~ Int1 ! Int2 \n)
short ESPReplyBufferIndex = 0;
bool ESPReplyBuffering = false;

void anemometerISR() {
  anemometerPulses++;
}

void rainBucketISR() {
  rainBucketPulses++;
}

void setup() {
  // Read and reset the mcu status register
  uint8_t mcusrBootValue = MCUSR;
  MCUSR &= ~(0b00001111);

  // Enable watchdog
  wdt_enable(WD_TIMEOUT);

  // Reset esp so we can send it boot msg
  esp_reset();

  // Serial
  Serial.begin(19200);

  DEBUG_PRINT(F("\n\nInitilizing"));

  for (int i=0; i<10; i++) {
    DEBUG_PRINT(F("."));
    delay(5);
  }

  DEBUG_PRINTLN(".");


  // We need to have something to send debug info with the esp to the server
  // ATmega328 Datasheet Section 15.9.1 (MCU Status Register)
  DEBUG_PRINTLN(F("Mcu Register Status: "));
  DEBUG_PRINT(F("WDRF: "));
  DEBUG_PRINTLN((mcusrBootValue>>WDRF)&1);
  DEBUG_PRINT(F("BORF: "));
  DEBUG_PRINTLN((mcusrBootValue>>BORF)&1);
  DEBUG_PRINT(F("EXTRF: "));
  DEBUG_PRINTLN((mcusrBootValue>>EXTRF)&1);
  DEBUG_PRINT(F("PORF: "));
  DEBUG_PRINTLN((mcusrBootValue>>PORF)&1);
  DEBUG_PRINT(F("Watchdog Status: "));
  DEBUG_PRINTLN(WDTCSR, BIN);

  // Esp serial
  ESPSerial.begin(ESP_ATMEGA_BAUD_RATE);
  ESPSerial.setTimeout(500);

  pinMode(ESP_RESET_PIN, INPUT); // Input while were not using it to not interfere with ESP programming

  // Indicator LEDS
  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(OK_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);

  // Battery pins
  pinMode(BAT_ADC_PIN, INPUT);

  // Inter-Chip interfaces
  Wire.begin();
  SPI.begin();
  //TWBR = 72;  // 50 kHz at 8 MHz clock

  // mcp23008 begin
  mcp.begin();      // use default address 0

  mcp.pinMode(MCP_ESP_RESULT_PIN, INPUT);
  mcp.pinMode(MCP_ESP_SUCCESS_PIN, INPUT);

  // Solar panel control pin
  mcp.pinMode(MCP_SOLAR_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(MCP_SOLAR_ENABLE_PIN, HIGH);

  mcp.pinMode(MCP_REFV_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(MCP_REFV_ENABLE_PIN, HIGH);

  mcp.pinMode(MCP_BAT_DIV_ENABLE_PIN, OUTPUT);
  mcp.digitalWrite(MCP_BAT_DIV_ENABLE_PIN, LOW);

  // Pins
  pinMode(ANEMOMETER_PIN, INPUT);
  digitalWrite(ANEMOMETER_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, RISING);

  pinMode(RAIN_BUCKET_PIN, INPUT);
  digitalWrite(RAIN_BUCKET_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(RAIN_BUCKET_PIN), rainBucketISR, RISING);
  sei();

  pinMode(WIND_VANE_ADC_PIN, INPUT);

  // Clock
  RTC.begin();

  // Lux Sensor (BH1750)
  luxMeter.begin(BH1750_ONE_TIME_HIGH_RES_MODE); // One shot then sleep

  // Check to see if the RTC is keeping time.  If it is, load the time from your computer.
  if (CLEAN_START || !RTC.isrunning()) {
    DEBUG_PRINTLN(F("Syncing RTC to compile time (UTC)"));

    // This will reflect the time that your sketch was compiled
    // Sucks cause if it takes forever to upload then its funny
    RTC.adjust(DateTime(DateTime(__DATE__, __TIME__) - TimeSpan(60*60*GMT_OFFSET) + TimeSpan(UPLOAD_TIME_OFFSET)));
  }

  // Store boot time
  bootTime = RTC.now().unixtime();

  // SRAM init
  sram_init(SRAM_CS_PIN); // Will set pin mode and such
  bool restoredSram = false;

  DEBUG_PRINTLN(F("Restore SRAM"));
  delay(1);

  // Try and restore from prevuint32_t weatherReadingWriteIndex = 0;
  if (CLEAN_START == 0 && sram_restore(&sampleAccumulator)) {
    sram_read(&weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
    sram_read(&weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);

    if (weatherReadingWriteIndex > SRAM_MAX_READINGS || weatherReadingReadIndex > SRAM_MAX_READINGS) {
      DEBUG_PRINTLN(F("ERROR: Sram Corrupted"));
      weatherReadingWriteIndex = 0;
      weatherReadingReadIndex = 0;
    } else {
      DEBUG_PRINTLN(F("Ok! Restored from SRAM."));
      restoredSram = true;
    }
  }

  if (!restoredSram) {
    if (CLEAN_START) {
      DEBUG_PRINTLN(F("No SRAM, CLEAN_START"));
    } else {
      DEBUG_PRINTLN(F("ERROR: No state in SRAM."));
    }

    sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
    sram_write(weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);
  }

  DEBUG_PRINT(F("Reading Read Index: "));
  DEBUG_PRINTLN(weatherReadingReadIndex);
  DEBUG_PRINT(F("Reading Write Index: "));
  DEBUG_PRINTLN(weatherReadingWriteIndex);

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
    DEBUG_PRINTLN(F("BME280 @ 0x76 not found; check wiring."));
  }

  DEBUG_PRINTLN(F("Sending Startup GET"));

  String bootMessage = "BOOT,";

  // lsb first so 0
  bootMessage += ((mcusrBootValue>>PORF)&1)?"1":"0";
  bootMessage += ((mcusrBootValue>>EXTRF)&1)?"1":"0";
  bootMessage += ((mcusrBootValue>>BORF)&1)?"1":"0";
  bootMessage += ((mcusrBootValue>>WDRF)&1)?"1":"0";

  Serial.print(F("Bootmsg: "));
  Serial.println(bootMessage);

  // Wait a bit
  wdt_reset();
  delay(500);

  esp_send_debug_request(bootMessage);

  esp_sleep();

  digitalWrite(OK_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);

  DEBUG_PRINTLN(F("Finished Init"));
  DEBUG_PRINTLN();
}
//
// void esp_do_spi() {
//   uint32_t spiHz = 10000000;
//   SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
//   mcp.digitalWrite(MCP_ESP_CS_PIN, LOW);
//   delay(1);
//
//   char data[33];
//   data[32] = 0;
//
//
//   SPI.transfer(0x03);
//   SPI.transfer(0x00);
//   for(uint8_t i=0; i<32; i++) {
//       data[i] = SPI.transfer(0);
//   }
//
//
//   mcp.digitalWrite(MCP_ESP_CS_PIN, HIGH);
//   SPI.endTransaction();
//
//   DEBUG_PRINT(F("Sent SPI got: "));
//   DEBUG_PRINTLN(String(data).c_str());
// }

// void esp_send_debug_request(String message) {
//   uint16_t debugReadPos = sram_read(SRAM_ADDR_DEBUG_READ_INDEX, SRAM_SIZE_DEBUG_READ_INDEX);
//   uint16_t debugWritePos = sram_read(SRAM_ADDR_DEBUG_WRITE_INDEX, SRAM_SIZE_DEBUG_WRITE_INDEX);
//
//   if (debugReadPos != debugWritePos) {
//
//   }

void esp_send_debug_request(String message) {
  espState = ESP_STATE_AWAITING_RESULT;
  startEspWaitTime = get_timestamp();

  send_esp_serial(ESP_MSG_REQUEST, ("/debug.php?"+message).c_str());
  delay(1);
}

void esp_sleep() {
  DEBUG_PRINTLN(F("Putting ESP to sleep now"));
  send_esp_serial(ESP_MSG_SLEEP, "true");
  espState = ESP_STATE_SLEEP;
}

void esp_reset() {
  DEBUG_PRINTLN(F("Resetting ESP"));
  pinMode(ESP_RESET_PIN, OUTPUT);
  digitalWrite(ESP_RESET_PIN, LOW);
  delay(20);
  digitalWrite(ESP_RESET_PIN, HIGH);
  delay(20);
  digitalWrite(ESP_RESET_PIN, LOW);
  pinMode(ESP_RESET_PIN, INPUT);
  delay(10);
  espState = ESP_STATE_IDLE;
}

void loop() {
  wdt_reset();
  recv_esp_serial();

  // If we're waiting for the esp to send the result of a http request
  if (espState == ESP_STATE_AWAITING_RESULT) {
    // If it replied
    if (mcp.digitalRead(MCP_ESP_RESULT_PIN)) {
      espState = ESP_STATE_IDLE;

      bool success = mcp.digitalRead(MCP_ESP_SUCCESS_PIN);
      DEBUG_PRINT(F("Esp sent result: "));
      DEBUG_PRINTLN(success);

      if (success) {
        failedSubmits = 0;
        advance_read_pointer();
      } else {
        failedSubmits++;

        DEBUG_PRINT(F("Failed, Retries Left: "));

        if (failedSubmits >= MAX_FAILED_SUBMITS) {
          DEBUG_PRINTLN(F("Aborting (!)"));
          submitTimeoutCountdown = 1;
        } else {
          DEBUG_PRINTLN(MAX_FAILED_SUBMITS-failedSubmits);
        }
      }

      // If we've submitted all or too many fails then put it back to sleep
      if (submitTimeoutCountdown > 0 || (weatherReadingReadIndex == weatherReadingWriteIndex)) {
        esp_sleep();
      }

    // If the esp timed out
    } else if (get_timestamp()-startEspWaitTime > ESP_REQUEST_TIMEOUT) {
      DEBUG_PRINTLN("Esp reply timeout.");
      // This'll start it again
      esp_reset();
    }
  // ESP_STATE_IDLE, no timeout condition and results waiting for submission
  } else if ((espState == ESP_STATE_IDLE) && (submitTimeoutCountdown <= 0) && (weatherReadingReadIndex != weatherReadingWriteIndex)) {
    submit_reading();
  }

  // Time to sample
  if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
    take_sample();

    // If we're at the sample before last
    if (sampleAccumulator.numSamples == SAMPLES_PER_READING-1) {
      // Wake the esp if it's sleeping
      if (espState == ESP_STATE_SLEEP) {
        esp_reset();
      }
    }
  }

  // If we've enough samples to make a reading
  if (sampleAccumulator.numSamples >= SAMPLES_PER_READING) {
    // Average the accumuator as a reading and store it
    WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
    sram_write_reading(&currentReading, weatherReadingWriteIndex);

    // Then zero the accumulator and store it
    zeroWeatherReading(&sampleAccumulator);
    sram_write_accumulator(&sampleAccumulator);

    #if DEBUG
    Serial.println();
    Serial.print(F("--(WEATHER READING ("));
    Serial.print(weatherReadingWriteIndex);
    Serial.println("))--");
    printWeatherReading(currentReading);
    Serial.println();
    #endif

    // Update the index and save it to sram
    weatherReadingWriteIndex++;

    if (weatherReadingWriteIndex >= SRAM_MAX_READINGS) {
      weatherReadingWriteIndex = 0;
    }

    sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);

    if (submitTimeoutCountdown > 0) {
      submitTimeoutCountdown--;
    }
  }
}

uint32_t get_timestamp() {
  return RTC.now().unixtime();
}

void advance_read_pointer() {
  if (weatherReadingReadIndex == weatherReadingWriteIndex) {
    return;
  }

  weatherReadingReadIndex++;

  if (weatherReadingReadIndex >= SRAM_MAX_READINGS) {
    weatherReadingReadIndex = 0;
  }

  // Store it
  sram_write(weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);
}

String generate_request_url(WeatherReading weatherReading) {
	// Make the url
	String outputUrl = "/report.php?";

	if (bmeConnected) {
	  if (!isnan(weatherReading.temperature)) {
	    outputUrl += "temp=";
	    outputUrl += weatherReading.temperature;
	  }

	  if (!isnan(weatherReading.humidity)) {
	      outputUrl += "&humidity=";
	      outputUrl += weatherReading.humidity;

	      // If we have temperature and humidity then calculate and submit the heat index also
	      if (!isnan(weatherReading.temperature)) {
	        outputUrl += "&heatIndex=";
	        outputUrl += computeHeatIndex(weatherReading.temperature, weatherReading.humidity, false);
	      }
	  }

	  if (!isnan(weatherReading.pressure)) {
	    outputUrl += "&pressure=";
	    outputUrl += weatherReading.pressure;
	  }
	}

  outputUrl += "&bat=";
  outputUrl += weatherReading.batteryMv;
  outputUrl += "&windSpeed=";
  outputUrl += weatherReading.windSpeed;
  outputUrl += "&windDirection=";
  outputUrl += weatherReading.windDirection;
	outputUrl += "&rain=";
	outputUrl += weatherReading.rain;
  outputUrl += "&lux=";
  outputUrl += weatherReading.lux;

  outputUrl += "&timestamp=";
  outputUrl += weatherReading.timestamp;

  // Secret key for security (>_O)
  //outputUrl += "&key=f6f9b0b8348a85843e951723a3060719f55985fd"; // frie!ggandham!!%2{[ sha1sum

	return outputUrl;
}

void submit_reading() {
  wdt_reset();

  DEBUG_PRINTLN();
  DEBUG_PRINT(F("Submiting Reading: "));
  DEBUG_PRINTLN(weatherReadingReadIndex);

  WeatherReading currentReading;
  sram_read_reading(&currentReading, weatherReadingReadIndex);

  if (currentReading.timestamp > get_timestamp()) {
    DEBUG_PRINTLN(F("Corrupt reading: Future timestamp, Skipping."));
    advance_read_pointer();
    return;
  }

  printWeatherReading(currentReading);
  String outputUrl = generate_request_url(currentReading);

  espState = ESP_STATE_AWAITING_RESULT;
  startEspWaitTime = get_timestamp();
  send_esp_serial(ESP_MSG_REQUEST, outputUrl.c_str());
  delay(1);
}

float readADCVoltage(int channel=0, float ratio=1.0, int offset=0, int oversampleBits=1) {
  // Oversample for 10 -> 10+oversampleBits bit adc resolution
  // 4^additionalBits
  int numSamples = (int)pow(4.0, oversampleBits);

	float refPinReading = 0.0;
	float channelPinReading = 0.0;

	// Get the vcc mV
	//////////////////////////////
	int referencePinMv = 2500;

	mcp.digitalWrite(MCP_REFV_ENABLE_PIN, LOW);
	delay(10); // For refV to stabilize

	// Select what we want
	analogRead(REF_ADC_PIN);

	for (int i=0; i<numSamples; i++) {
		refPinReading += analogRead(REF_ADC_PIN);
    yield();
	}

	refPinReading /= numSamples;

	mcp.digitalWrite(MCP_REFV_ENABLE_PIN, HIGH);

	float vccMv = ( referencePinMv / refPinReading ) * 1023;

	// Get the channel mV
	//////////////////////////////
	analogRead(channel);

	for (int i=0; i<numSamples; i++) {
		channelPinReading += analogRead(channel);
    yield();
	}

	channelPinReading /= numSamples;

  float pinMv = (vccMv / 1023.0) * channelPinReading;
	float readingMv = pinMv * ratio;

	DEBUG2_PRINT(F("ADC channel: "));
	DEBUG2_PRINT(channel);
	DEBUG2_PRINT(F(" refPin: "));
	DEBUG2_PRINT(refPinReading);
  DEBUG2_PRINT(F(" channelPin: "));
  DEBUG2_PRINT(channelPinReading);
	DEBUG2_PRINT(F(" vccMv: "));
	DEBUG2_PRINT(vccMv);
	DEBUG2_PRINT(F(" pinMv: "));
	DEBUG2_PRINT(pinMv);
  DEBUG2_PRINT(F(" readingMv: "));
  DEBUG2_PRINT(readingMv);
	DEBUG2_PRINT(F(" resultMv: "));
	DEBUG2_PRINTLN(readingMv+offset);

  return readingMv + offset;
}

void send_esp_serial(char messageType, const char *value) {
  DEBUG_PRINT(F("ESP (sent)"));
  DEBUG_PRINT((int)messageType);
  DEBUG_PRINT((int)strlen(value));
  DEBUG_PRINT(value);
  DEBUG_PRINTLN();

  ESPSerial.write(messageType);
  ESPSerial.write((char)strlen(value));
  ESPSerial.write(value);
}

bool recv_esp_serial() {
	// Print ESP serial messages
	if (ESPSerial.available() > 0) {
    // If we're waiting to see if it's alive
    if (espState == ESP_STATE_WAKING) {
      espState = ESP_STATE_IDLE;
    }

		uint8_t rxByte = ESPSerial.read();

		if (ESPNewline) {
			ESPNewline = false;

			if (!ESPReplyBuffering) {
				DEBUG_PRINT(F("ESP (recv)"));
			}
		}

		// Print it if it's not a reply to us (Just it trying to print something)
		if (!ESPReplyBuffering && rxByte) {
			Serial.print((char)rxByte);
		}

		if (rxByte == 126) {
			ESPReplyBufferIndex = 0;
			ESPReplyBuffering = true;
		} else if (rxByte == 10) {
			ESPNewline = true;
			ESPReplyBuffering = false;
		}

		if (ESPReplyBuffering && ESPReplyBufferIndex < 5) {
			ESPReplyBuffer[ESPReplyBufferIndex++] = rxByte;
		}
	}
}

float read_anemometer() {
  unsigned long currentReadingMillis = millis();

  cli();
  int numTimelyPulses = anemometerPulses;
  anemometerPulses = 0;
  sei();

  DEBUG3_PRINT(F("Num wind pulses: "));
  DEBUG3_PRINTLN(numTimelyPulses);

  float anemometerMph = (numTimelyPulses * 0.5) * (60000.0 / (currentReadingMillis-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;

  lastAnemometerReadingMillis = currentReadingMillis;

  return anemometerMph;
}

int read_rain() {
  rainBucketPulses = 0;
  return 0;
  // //unsigned long currentReadingMillis = millis();
  //
  // cli();
  // int numTimelyPulses = rainBucketPulses;
  // rainBucketPulses = 0;
  // sei();
  //
  // DEBUG3_PRINT(F("Num rain pulses: "));
  // DEBUG3_PRINTLN(numTimelyPulses);
  //
  // return numTimelyPulses;
}

void read_wind_vane(float *destX, float *destY) {
  int windDegrees = map(analogRead(WIND_VANE_ADC_PIN), WIND_VANE_MIN, WIND_VANE_MAX, 0, 360);
  float windRadians = windDegrees * M_PI / 180;

  *destY += sin(windRadians);
  *destX += cos(windRadians);

  // DEBUG3_PRINT(F("Wind Degrees: "));
  // DEBUG3_PRINTLN(windDegrees);
}

float read_battery_voltage(int oversampleBits=3) {
  //1.33511348465; // R2/R1 3.008/1.002
  float dividerRatio = 1.335;

  // Disable the solar panel to not interfere with the readings
  mcp.digitalWrite(MCP_SOLAR_ENABLE_PIN, LOW);
  delay(10);

  // Switch on the voltage divider for the battery and charge the cap and stabilize reference voltage
  mcp.digitalWrite(MCP_BAT_DIV_ENABLE_PIN, HIGH);
  delay(20); // Wait a bit

  // Voltage afterwards
	float batteryVoltage = readADCVoltage(BAT_ADC_PIN, dividerRatio, 15, oversampleBits);

  // Turn the divider back off
  mcp.digitalWrite(MCP_BAT_DIV_ENABLE_PIN, LOW);

  // Turn the solar panel back on
  mcp.digitalWrite(MCP_SOLAR_ENABLE_PIN, HIGH);

  return batteryVoltage;
}

uint16_t read_lux() {
  luxMeter.begin(BH1750_ONE_TIME_HIGH_RES_MODE); // One shot then sleep
  return luxMeter.readLightLevel();
}

void read_tph(float *dest) {
  //******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  if (!bmeConnected) {
    return;
  }

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

  dest[0] = bmeTemp;
  dest[1] = bmePressure;
  dest[2] = bmeHumidity;
}

void take_sample() {
  wdt_reset();

  #if DEBUG
  digitalWrite(OK_LED_PIN, HIGH);
  DEBUG2_PRINTLN();
  DEBUG2_PRINT(F("--(Taking sample: "));
  DEBUG2_PRINT(sampleAccumulator.numSamples+1);
  DEBUG2_PRINT(F("/"));
  DEBUG2_PRINT(SAMPLES_PER_READING);
  DEBUG2_PRINTLN(F(")--"));
  DEBUG2_PRINT(F("For Write Index: "));
  DEBUG2_PRINT(weatherReadingWriteIndex);
  DEBUG2_PRINT(F(" | Read Index: "));
  DEBUG2_PRINTLN(weatherReadingReadIndex);
  DEBUG2_PRINT(F("Free Ram (bytes): "));
  DEBUG2_PRINTLN(getFreeRam());
  DEBUG2_PRINT(F("Uptime: "));
  DEBUG2_PRINT((get_timestamp()-bootTime)/60.0);
  DEBUG2_PRINT(F(" (mins) since "));
  #if DEBUG >= 2
  print_pretty_timestamp(bootTime);
  #endif
  DEBUG2_PRINTLN();
  #endif

  lastSampleMillis = millis();

  sampleAccumulator.timestamp = get_timestamp();

  sampleAccumulator.windSpeed += int32_t(read_anemometer()*100);
  read_wind_vane(&sampleAccumulator.windDirectionX, &sampleAccumulator.windDirectionY);

  sampleAccumulator.rain += read_rain();

  if (sampleAccumulator.numSamples%BATTERY_SAMPLE_MODULO == 0){
    sampleAccumulator.batteryMv += int32_t(read_battery_voltage()*100);
    sampleAccumulator.numBatterySamples++;
  }

  float bmeReadings[3];
  read_tph(bmeReadings);

  sampleAccumulator.temperature += int32_t(bmeReadings[0]*100);
  sampleAccumulator.pressure += int32_t(bmeReadings[1]*100);
  sampleAccumulator.humidity += int32_t(bmeReadings[2]*100);

  sampleAccumulator.lux += read_lux();

  sampleAccumulator.numSamples++;

  sram_write_accumulator(&sampleAccumulator);

  #if DEBUG >= 2
  printWeatherReading(sampleAccumulator);
  DEBUG2_PRINTLN();
  #endif

  #if DEBUG
  digitalWrite(OK_LED_PIN, LOW);
  #endif
}
