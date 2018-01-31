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

// Includes
#include <Wire.h>
#include <SoftwareSerial.h>

#include "config.h"
#include "helpers.h"
#include "weather_readings.h"

#include "RTClib.h"
#include <Adafruit_BME280.h>

// Sample and Reading defines
#define SAMPLE_INTERVAL 2000
#define READING_INTERVAL SAMPLE_INTERVAL*5///300000 // 300000 = 5 minutes
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL

// Wind Vane Calibration
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

// Anemometer Calibration (Linear Association)
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = samples per 1mph


// Pins
#define ANEMOMETER_PIN 2
#define BATTERY_ADC_PIN A0
#define WIND_VANE_ADC_PIN A3

#define ESP_TX_PIN 11
#define ESP_RX_PIN 12
#define ESP_RESET_PIN 13

// Globals
uint32_t bootTime;
unsigned long numWeatherReadings = 0;
unsigned long lastSampleMillis = 0;
bool bmeConnected = false;

volatile unsigned int anemometerPulses = 0;
unsigned long lastAnemometerReadingMillis = 0;

WeatherReadingAccumulator sampleAccumulator;

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

  // Inter-Chip interfaces
  Wire.begin();

  // Pins
  pinMode(ANEMOMETER_PIN, INPUT);
  digitalWrite(ANEMOMETER_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, RISING);
  sei();

  pinMode(BATTERY_ADC_PIN, INPUT);
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
    Serial.println("BME280 sensor is not detected at i2caddr 0x76; check wiring.");
  }
}

void loop() {
  recv_esp_serial();

  if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
    take_sample();

    // Reset the ESP the sample before
    if (sampleAccumulator.numSamples == SAMPLES_PER_READING-1) {
      reset_esp();
    }
  }

  if (sampleAccumulator.numSamples >= SAMPLES_PER_READING) {
    WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
    zeroWeatherReading(&sampleAccumulator);

    numWeatherReadings++;

    Serial.println();
    Serial.print("--(WEATHER READING (");
    Serial.print(numWeatherReadings);
    Serial.println("))--");
    printWeatherReading(currentReading);
    Serial.println();

    while (ESPSerial.available()) {
      recv_esp_serial();
    }

    String outputUrl = generate_request_url(currentReading);
    send_esp_serial(ESP_MSG_REQUEST, outputUrl.c_str());
    delay(5);
    send_esp_serial(ESP_MSG_SLEEP, "true");
  }
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


uint32_t read_timestamp() {
  return RTC.now().unixtime();
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

int read_battery_voltage(float oversampleBits=3.0) {
  // Oversample for 10 -> 13 bit adc resolution so we can get .6 mv Accuracy
  // 4^additionalBits
  float batteryAdcAccumulator = 0;
  int numSamples = (int)pow(4.0, oversampleBits);

  for (int i=0; i<numSamples; i++) {
    batteryAdcAccumulator += analogRead(BATTERY_ADC_PIN);
    yield();
  }

  batteryAdcAccumulator /= numSamples;

  return (batteryAdcAccumulator / 1023.0) * 5000;
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

  sampleAccumulator.battery += read_battery_voltage();
  sampleAccumulator.numBatterySamples++;

  double bmeReadings[3];
  read_tph(bmeReadings);

  sampleAccumulator.temperature += bmeReadings[0];
  sampleAccumulator.pressure += bmeReadings[1];
  sampleAccumulator.humidity += bmeReadings[2];

  sampleAccumulator.numSamples++;

  WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
  Serial.print("Taking sample num: ");
  Serial.print(sampleAccumulator.numSamples);
  Serial.print("/");
  Serial.print(SAMPLES_PER_READING);
  Serial.print(" for reading ");
  Serial.println(numWeatherReadings);
  Serial.print("Uptime: ");
  Serial.print((read_timestamp()-bootTime)/60.0);
  Serial.print(" (mins) since ");
  print_pretty_timestamp(bootTime);
  Serial.println();

  printWeatherReading(currentReading);
  Serial.println();
}
