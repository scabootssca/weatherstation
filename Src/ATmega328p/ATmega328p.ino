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

// Sample and Reading defines
#define SAMPLE_INTERVAL 2000
#define READING_INTERVAL 300000 //SAMPLE_INTERVAL*5///300000 // 300000 = 5 minutes
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL
#define BATTERY_SAMPLE_MODULO 25
#define MAX_FAILED_SUBMITS 3

// Wind Vane Calibration
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

// Anemometer Calibration (Linear Association)
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph

// Pins
#define ANEMOMETER_PIN 8 // PB0
#define OK_LED_PIN 9     // PB1
#define ERROR_LED_PIN 10 // PB2
#define MOSI_PIN 11      // PB3
#define MISO_PIN 12      // PB4
#define SCK_PIN 13       // PB5

#define BAT_ADC_PIN A0       // PC0
#define REF_ADC_PIN A1       // PC1
#define SOLAR_ENABLE_PIN A2  // PC2
#define WIND_VANE_ADC_PIN A3 // PC3

#define ESP_RESET_PIN 2      // PD2
#define SRAM_CS_PIN 3        // PD3
#define REFV_ENABLE_PIN 4    // PD4
#define ESP_TX_PIN 5         // PD5
#define ESP_RX_PIN 6         // PD6
#define BAT_DIV_ENABLE_PIN 7 // PD7

// Includes
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "config.h"
#include "helpers.h"
#include "weather_readings.h"

#include "RTClib.h"
#include <Adafruit_BME280.h>
#include "MCP_23A1024.h"

// Globals
uint32_t bootTime;

bool bmeConnected = false;

unsigned long lastSampleMillis = 0;
volatile unsigned int anemometerPulses = 0;
unsigned long lastAnemometerReadingMillis = 0;

WeatherReadingAccumulator sampleAccumulator;

uint32_t weatherReadingWriteIndex = 0;
uint32_t weatherReadingReadIndex = 0;

// Structures/Classes
RTC_DS1307 RTC; // Real Time Clock
Adafruit_BME280 bmeSensor; // Termometer, Hygrometer, Barometer
SoftwareSerial ESPSerial(ESP_RX_PIN, ESP_TX_PIN);

bool ESPNewline = true;
char ESPReplyBuffer[11]; // 1 + 4 + 1 + 4 + 1 == (~ Int1 ! Int2 \n)
short ESPReplyBufferIndex = 0;
bool ESPReplyBuffering = false;

void anemometerISR() {
  anemometerPulses++;
}

void setup() {
  // Serial
  Serial.begin(19200);

  Serial.println("\n");
  Serial.println("Initilizing ATmega328p");

  // Esp serial
  ESPSerial.begin(ESP_ATMEGA_BAUD_RATE);

  pinMode(ESP_RESET_PIN, INPUT); // Input while were not using it to not interfere with ESP programming

  // Indicator LEDS
  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(OK_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);

  // Battery pins
  pinMode(BAT_ADC_PIN, INPUT);

  pinMode(BAT_DIV_ENABLE_PIN, OUTPUT);
  digitalWrite(BAT_DIV_ENABLE_PIN, LOW);

  pinMode(REFV_ENABLE_PIN, OUTPUT);
  digitalWrite(REFV_ENABLE_PIN, HIGH);

  // Solar panel control pin
  pinMode(SOLAR_ENABLE_PIN, OUTPUT);
  digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  // Inter-Chip interfaces
  Wire.begin();
  SPI.begin();

  // Pins
  pinMode(ANEMOMETER_PIN, INPUT);
  digitalWrite(ANEMOMETER_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, RISING);
  sei();


  pinMode(WIND_VANE_ADC_PIN, INPUT);

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

  // SRAM init
  sram_init(SRAM_CS_PIN); // Will set pin mode and such

  Serial.println("Attempting to restore from SRAM after 200ms");
  delay(200);

  // Try and restore from prevuint32_t weatherReadingWriteIndex = 0;
  if (!CLEAN_START && sram_restore(&sampleAccumulator)) {
    sram_read(&weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
    sram_read(&weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);

    Serial.println(F("Sucessfully restored state from SRAM."));
  } else {
    if (CLEAN_START) {
      Serial.println(F("Not restoring from SRAM, CLEAN_START"));
    } else {
      Serial.println(F("Previous state not found in SRAM, starting fresh."));
    }

    sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
    sram_write(weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);
  }

  Serial.print("Reading Read Index: ");
  Serial.println(weatherReadingReadIndex);
  Serial.print("Reading Write Index: ");
  Serial.println(weatherReadingWriteIndex);

  // Store boot time
  bootTime = RTC.now().unixtime();

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
    Serial.println(F("BME280 sensor is not detected at i2caddr 0x76; check wiring."));
  }

  Serial.println(F("Finished Initilizing"));
  Serial.println();

  digitalWrite(OK_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
}

void loop() {
  recv_esp_serial();

  if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
    digitalWrite(OK_LED_PIN, HIGH);
    take_sample();

    // Reset the ESP the sample before
    if (sampleAccumulator.numSamples == SAMPLES_PER_READING-1) {
      reset_esp();
    }

    digitalWrite(OK_LED_PIN, LOW);
  }

  if (sampleAccumulator.numSamples >= SAMPLES_PER_READING) {
    // Average the accumuator as a reading and store it
    WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
    sram_write_reading(&currentReading, weatherReadingWriteIndex);

    // Then zero the accumulator and store it
    zeroWeatherReading(&sampleAccumulator);
    sram_write_accumulator(&sampleAccumulator);

    Serial.println();
    Serial.print("--(WEATHER READING (");
    Serial.print(weatherReadingWriteIndex);
    Serial.println("))--");
    printWeatherReading(currentReading);
    Serial.println();

    // Update the index and save it to sram
    weatherReadingWriteIndex++;

    if (weatherReadingWriteIndex >= SRAM_MAX_READINGS) {
      weatherReadingWriteIndex = 0;
    }

    sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);

    submit_stored_readings();
  }
}

uint32_t read_timestamp() {
  return RTC.now().unixtime();
}

void submit_stored_readings() {
  // Have this take the latest reading as an argument and submit that first then any that are stored afterwards
  int failedSubmits = 0;
  uint32_t submitStartTimestamp = read_timestamp();
  WeatherReading currentReading;

  Serial.println(F("[Starting Submit Stored Readings]"));

  if (weatherReadingReadIndex == weatherReadingWriteIndex) {
    Serial.println(F("[Nothing to submit]"));
    return;
  }

  while (failedSubmits < MAX_FAILED_SUBMITS) {
    while (ESPSerial.available()) {
      recv_esp_serial();
    }

    bool advanceReadPointer = false;

    sram_read_reading(&currentReading, weatherReadingReadIndex);

    // Check for read corruption
    if (currentReading.timestamp > submitStartTimestamp) {
      Serial.println("Corrupted reading: Future timestamp, Skipping.");
      advanceReadPointer = true;
    } else {
      Serial.print(F("Weather Reading Index: "));
      Serial.println(weatherReadingReadIndex);
      printWeatherReading(currentReading);

      String outputUrl = generate_request_url(currentReading);
      send_esp_serial(ESP_MSG_REQUEST, outputUrl.c_str());
      delay(1);

      advanceReadPointer = true;
    }

    // If we need to goto the next reading
    if (advanceReadPointer) {
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

      if (failedSubmits >= MAX_FAILED_SUBMITS) {
        Serial.println("Failed - Aborting (!)");
        break;
      }

      Serial.print(F("Failed - Retries Left: "));
      Serial.println(MAX_FAILED_SUBMITS-failedSubmits);
    }
  }

  delay(5);
  send_esp_serial(ESP_MSG_SLEEP, "true");
}

float readADCVoltage(int channel=0, float ratio=1.0, int offset=0, int oversampleBits=1) {
  // Oversample for 10 -> 10+oversampleBits bit adc resolution
  // 4^additionalBits
  int numSamples = (int)pow(4.0, oversampleBits);

	float refPinReading = 0.0;
	float channelPinReading = 0.0;

	// Get the vcc mV
	//////////////////////////////
	float referencePinMv = 2495; //2500.0;

	digitalWrite(REFV_ENABLE_PIN, LOW);
	delay(10); // For refV to stabilize

	// Select what we want
	analogRead(REF_ADC_PIN);

	for (int i=0; i<numSamples; i++) {
		refPinReading += analogRead(REF_ADC_PIN);
    yield();
	}

	refPinReading /= numSamples;

	digitalWrite(REFV_ENABLE_PIN, HIGH);

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

void reset_esp() {
  pinMode(ESP_RESET_PIN, OUTPUT);
  digitalWrite(ESP_RESET_PIN, LOW);
  delay(5);
  digitalWrite(ESP_RESET_PIN, HIGH);
  delay(5);
  digitalWrite(ESP_RESET_PIN, LOW);
  pinMode(ESP_RESET_PIN, INPUT);
  delay(10);
}

void send_esp_serial(char messageType, const char *value) {
  Serial.print("ESP (sent)");
  Serial.print((int)messageType);
  Serial.print((int)strlen(value));
  Serial.print(value);
  Serial.println();

  ESPSerial.write(messageType);
  ESPSerial.write((char)strlen(value));
  ESPSerial.write(value);
}

bool recv_esp_serial() {
	// Print ESP serial messages
	while (ESPSerial.available() > 0) {
		uint8_t rxByte = ESPSerial.read();

		if (ESPNewline) {
			ESPNewline = false;

			if (!ESPReplyBuffering) {
				Serial.write("ESP (recv)");
			}
		}

		// Print it if it's not a reply to us
		if (!ESPReplyBuffering) {
			Serial.write(rxByte);
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
  int numTimelyAnemometerPulses = anemometerPulses;
  anemometerPulses = 0;
  sei();

  float anemometerMph = (numTimelyAnemometerPulses * 0.5) * (60000.0 / (currentReadingMillis-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;

  lastAnemometerReadingMillis = currentReadingMillis;

  return anemometerMph;
}

int read_wind_vane() {
  int windVanePinReading = analogRead(WIND_VANE_ADC_PIN);
  int windDegrees = map(windVanePinReading, WIND_VANE_MIN, WIND_VANE_MAX, 0, 360);

  return windDegrees;
}

float read_battery_voltage(float oversampleBits=3.0) {
  //1.33511348465; // R2/R1 3.008/1.002
  float dividerRatio = 1.335;

  // Disable the solar panel to not interfere with the readings
  digitalWrite(SOLAR_ENABLE_PIN, LOW);
  delay(10);

  // Switch on the voltage divider for the battery and charge the cap and stabilize reference voltage
  digitalWrite(BAT_DIV_ENABLE_PIN, HIGH);
  delay(20); // Wait a bit

  // Voltage afterwards
	float batteryVoltage = readADCVoltage(BAT_ADC_PIN, dividerRatio, 15, oversampleBits);

  // Turn the divider back off
  digitalWrite(BAT_DIV_ENABLE_PIN, LOW);

  // Turn the solar panel back on
  digitalWrite(SOLAR_ENABLE_PIN, HIGH);

  // Output
  DEBUG_PRINT("Battery Voltage: ");
  DEBUG_PRINTLN(batteryVoltage);

  return batteryVoltage;
}

void read_tph(double *dest) {
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
  lastSampleMillis = millis();

  sampleAccumulator.timestamp += read_timestamp();
  sampleAccumulator.windSpeed += read_anemometer();
  sampleAccumulator.windDirection += read_wind_vane();

  if (sampleAccumulator.numSamples%BATTERY_SAMPLE_MODULO == 0){
    sampleAccumulator.battery += read_battery_voltage();
    sampleAccumulator.numBatterySamples++;
  }

  double bmeReadings[3];
  read_tph(bmeReadings);

  sampleAccumulator.temperature += bmeReadings[0];
  sampleAccumulator.pressure += bmeReadings[1];
  sampleAccumulator.humidity += bmeReadings[2];

  sampleAccumulator.numSamples++;

  sram_write_accumulator(&sampleAccumulator);

  #if DEBUG
  Serial.print("Taking sample: ");
  Serial.print(sampleAccumulator.numSamples);
  Serial.print("/");
  Serial.println(SAMPLES_PER_READING);
  Serial.print("Uptime: ");
  Serial.print((read_timestamp()-bootTime)/60.0);
  Serial.print(" (mins) since ");
  print_pretty_timestamp(bootTime);
  Serial.println();

  printWeatherReading(sampleAccumulator);

  Serial.println();
  #endif
}
