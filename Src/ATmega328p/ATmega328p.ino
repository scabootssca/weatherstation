/*
Copyright 2017 Jason Ryan Richards

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
SAMPLES:
  * 3 Second: Wind Speed, Wind Direction
  * 10 Second: Temperature, Humidity, Pressure, Global Radiation

READINGS:
  * 1 Min Avg: Temperature, Humidity, Pressure, Global Radiation
  * 2-10 Min Avg: Wind Direction, Wind Speed

*/
#include <avr/wdt.h>
#define WD_TIMEOUT WDTO_8S
#define I2C_TIMEOUT_MS 2000
#define VERSION "Version: 0eba866caa821bdc40ec685b2c2e5a583804624d Mon May 14 15:13:45 2018 -0500"

#define CLEAN_START 0 // This will mark SRAM as unpopulated and update rtc to compile time
#define UPLOAD_TIME_OFFSET 0//53

#define SPI_HZ 500000 // 500Khz

// New timing defines
#define SAMPLE_DELTA_SHORT 3000
#define SAMPLE_DELTA_LONG 10000

// Sample and Reading defines
#define SAMPLE_INTERVAL 2000
#define READING_INTERVAL 300000 //SAMPLE_INTERVAL*5///300000 // 300000 = 5 minutes
#define SAMPLES_PER_READING READING_INTERVAL/SAMPLE_INTERVAL
#define BATTERY_SAMPLE_MODULO 25
#define MAX_FAILED_SUBMITS 3


// Wind Vane Calibration
#define WIND_VANE_MIN 0
#define WIND_VANE_MAX 1023

// Lux Calibration (Seems our dome blocks about 6.25% of light based off one uncontrolled rough test)
#define LUX_DOME_BLOCKING_COEF 1.0625

// Anemometer Calibration (Linear Association)
// rpm/coef = wind speed in mph
#define ANEMOMETER_CALIBRATION_COEF 0.09739260404185239 // 10.2677201193868 = revolutions per 1mph
#define ANEMOMETER_TDELTA_1_MPH 2921.778121255 // (60/10.2677201193868)*0.5 = Tdelta in samples per 1mph (For millis) 2 samples per revolution

// I2c Addresses
//0x68 == RTC
//0x76 == BME280
//0x20 == MCP23008
//0x23 == BH1730 Lux

// Pins
#define OK_LED_PIN 9     // PB1
#define ERROR_LED_PIN 10 // PB2
#define MOSI_PIN 11      // PB3
#define MISO_PIN 12      // PB4
#define SCK_PIN 13       // PB5

#define WIND_VANE_ADC_PIN A1 // PC1
#define REF_ADC_PIN A2       // PC2
#define BAT_ADC_PIN A3       // PC3
#define I2C_SDA_PIN A4       // PC4
#define I2C_SCL_PIN A5       // PC5

#define RAIN_BUCKET_PIN 2    // PD2
#define ANEMOMETER_PIN 3     // PD3
#define ESP_TX_PIN 5         // PD5
#define ESP_RX_PIN 6         // PD6
#define SRAM_CS_PIN 7        // PD7


// Mcp23008 pins
#define MCP_ESP_RESULT_PIN 6
#define MCP_ESP_SUCCESS_PIN 7
#define MCP_ESP_RESET_PIN 5

#define MCP_BAT_DIV_ENABLE_PIN 0 // Biased Low Off, Pull High To Enable
#define MCP_REFV_ENABLE_PIN 1    // Biased High Off, Pull Low To Enable
#define MCP_SOLAR_ENABLE_PIN 2   // Biased High On, Pull Low To Disable

// Includes
#include <limits.h>

#include "I2C.h"
#include <SPI.h>
#include <SoftwareSerial.h>
// #include <math.h>

#include "config.h"
#include "helpers.h"
#include "weather_readings.h"

#include "RTClib.h"
#include "BME280.h"
#include "MCP_23A1024.h"
#include "Adafruit_MCP23008.h"
#include "BH1750.h"



// Globals
uint32_t bootTime;

bool bmeConnected = false;
bool luxConnected = false;

unsigned long lastSampleMillis = 0;

volatile unsigned long lastAnemometerPulseMillis = 0;
volatile unsigned int anemometerPulses = 0;
volatile uint64_t anemometerTDeltaAccumulator = 0;
volatile unsigned long anemometerMinTDelta = ULONG_MAX;

volatile unsigned int rainBucketPulses = 0;
unsigned long lastRainBucketReadingMillis = 0;

WeatherReadingAccumulator sampleAccumulator;

uint16_t weatherReadingWriteIndex = 0;
uint16_t weatherReadingReadIndex = 0;

#define ESP_STATE_SLEEP 0
#define ESP_STATE_IDLE 1
#define ESP_STATE_AWAITING_RESULT 2
#define ESP_STATE_RESETTING 3 // It can get stuck in here at line

unsigned short espState = ESP_STATE_SLEEP;

// All in seconds
#define ESP_RESET_READY_DELAY 2 // In seconds how long to wait after resetting the esp before trying to make it send a request
#define ESP_RESET_TIMEOUT 30 // How long after resetting to wait for a response before trying again
#define ESP_REQUEST_TIMEOUT 30 // In seconds after sending a request to call it failed
#define ESP_REQUEST_DELAY 2 // This will also act as a request rate limiter

uint32_t espResetTime = 0;
uint32_t espRequestTime = 0;

unsigned int failedSubmits = 0;
unsigned int submitTimeoutCountdown = 0;

// Structures/Classes
Adafruit_MCP23008 mcp; // IO Expander
RTC_DS1307 RTC; // Real Time Clock
Adafruit_BME280 bmeSensor; // Termometer, Hygrometer, Barometer
BH1750 luxMeter; // Lux

SoftwareSerial ESPSerial(ESP_RX_PIN, ESP_TX_PIN);

// bool ESPNewline = true;
// char ESPReplyBuffer[11]; // 1 + 4 + 1 + 4 + 1 == (~ Int1 ! Int2 \n)
// short ESPReplyBufferIndex = 0;
// bool ESPReplyBuffering = false;

// void anemometerISR() {
//   anemometerPulses++;
// }

void anemometerISR() {
  // If we've no pulses prior then count this and return
  if (lastAnemometerPulseMillis == 0) {
    //Serial.print("First: ");
    //Serial.println(millis());    lastAnemometerPulseMillis = millis();
    lastAnemometerPulseMillis = millis();
    return;
  }

  unsigned long tDelta = 0;

  // We rolled over
  if (millis() < lastAnemometerPulseMillis) {
    tDelta = (ULONG_MAX-lastAnemometerPulseMillis)+millis();
  } else {
    // Get the tdelta and use it to store the average
    tDelta = millis()-lastAnemometerPulseMillis;
  }

  // 50ms debounce
  // Means ~58.4 MPH is the highest we can measure with this
  if (tDelta < 50) {
    return;
  }

  //Serial.print("tDelta: ");
  //Serial.println(tDelta);

  // Add the delta to the accumulator
  anemometerTDeltaAccumulator += tDelta;

  // For peak instant gust
  if (tDelta < anemometerMinTDelta) {
    anemometerMinTDelta = tDelta;
  }

  // Save this time
  lastAnemometerPulseMillis = millis();
  anemometerPulses++;
}

void rainBucketISR() {
  rainBucketPulses++;
}

void init_bme() {
  DEBUG_PRINTLN("Init BME");

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
    DEBUG_PRINTLN("BME280 @ 0x76 not found.");
  }
}

bool init_i2c() {
  I2c.end();

  // Both to input to check their status
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);

  Serial.print("Reset I2C - SDA: ");
  Serial.print(digitalRead(I2C_SDA_PIN));
  Serial.print(" SCL: ");
  Serial.print(digitalRead(I2C_SCL_PIN));
  Serial.println();

  // Hold sda low while we pulse the clock
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);

  // Reset I2c Bus with 16 clock pulses as per specs
  pinMode(I2C_SCL_PIN, OUTPUT);

  for (int i=0; i<16; i++) {
    digitalWrite(I2C_SCL_PIN, i?HIGH:LOW);
    delay(1);
  }

  // Then a stop by raising scl then sda
  pinMode(I2C_SCL_PIN, INPUT);
  pinMode(I2C_SDA_PIN, INPUT);

  // See if its good
  // If either is low then it may still be stuck
  if (!digitalRead(I2C_SCL_PIN) || !digitalRead(I2C_SDA_PIN)) {
    esp_send_debug_request("I2C_Stuck");
    return false;
  }

  I2c.begin();
  I2c.timeOut(I2C_TIMEOUT_MS);

  return true;
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
  Serial.setTimeout(500);

  DEBUG_PRINT(F("\n\nInitilizing "));
  DEBUG_PRINTLN(F(VERSION));
  DEBUG_PRINT("Compiled: ");
  DEBUG_PRINTLN(__DATE__);

  for (int i=0; i<10; i++) {
    DEBUG_PRINT(F("."));
    delay(5);
  }

  DEBUG_PRINTLN(".");

  // We need to have something to send debug info with the esp to the server
  // ATmega328 Datasheet Section 15.9.1 (MCU Status Register)
  DEBUG_PRINTLN(F("MCUSR Value: "));
  DEBUG_PRINT(F("WDRF: "));
  DEBUG_PRINTLN((mcusrBootValue>>WDRF)&1);
  DEBUG_PRINT(F("BORF: "));
  DEBUG_PRINTLN((mcusrBootValue>>BORF)&1);
  DEBUG_PRINT(F("EXTRF: "));
  DEBUG_PRINTLN((mcusrBootValue>>EXTRF)&1);
  DEBUG_PRINT(F("PORF: "));
  DEBUG_PRINTLN((mcusrBootValue>>PORF)&1);

  // Set reference voltage pin proper
  //Bits 7:6 – REFSn: Reference Selection [n = 1:0]
  // 01 = AV CC with external capacitor at AREF pin
  DEBUG_PRINT("Setting ADMUX: ");
  ADMUX |= 0b01000000;
  Serial.println(ADMUX, BIN);

  // Esp serial
  ESPSerial.begin(ESP_ATMEGA_BAUD_RATE);
  ESPSerial.setTimeout(500);

  DEBUG_PRINT("1."); // Esp Serial DONE

  mcp.pinMode(MCP_ESP_RESET_PIN, INPUT); // Input while were not using it to not interfere with ESP programming

  // Indicator LEDS
  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(OK_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);

  // Battery pins
  pinMode(BAT_ADC_PIN, INPUT);

  DEBUG_PRINT("2."); // ATmega pins DONE

  // Inter-Chip interfaces
  init_i2c();

  SPI.begin();
  //TWBR = 72;  // 50 kHz at 8 MHz clock

  DEBUG_PRINT("3."); // Interfaces started DONE

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

  DEBUG_PRINT("4."); // MCP pins set DONE

  // Interrupts
  pinMode(ANEMOMETER_PIN, INPUT);
  digitalWrite(ANEMOMETER_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), anemometerISR, RISING);

  pinMode(RAIN_BUCKET_PIN, INPUT);
  digitalWrite(RAIN_BUCKET_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(RAIN_BUCKET_PIN), rainBucketISR, RISING);
  sei();

  // Wind vane
  pinMode(WIND_VANE_ADC_PIN, INPUT);

  DEBUG_PRINT("5."); // Interrupts DONE

  // Clock
  RTC.begin();

  DEBUG_PRINT("6."); // RTC DONE

  // Lux Sensor (BH1750)
  luxMeter.begin(BH1750_ONE_TIME_HIGH_RES_MODE); // One shot then sleep
  luxConnected = true;

  DEBUG_PRINTLN("7"); // Lux meter DONE

  // Check to see if the RTC is keeping time.  If it is, load the time from your computer.
  if (CLEAN_START || !RTC.isrunning()) {
    // This will reflect the time that your sketch was compiled
    // Sucks cause if it takes forever to upload then its funny
    DEBUG_PRINTLN(F("Syncing RTC to compile time (local))"));
    RTC.adjust(DateTime(DateTime(__DATE__, __TIME__) - TimeSpan(60*60*GMT_OFFSET) + TimeSpan(UPLOAD_TIME_OFFSET)));
  }

  // Store boot time
  bootTime = RTC.now().unixtime();
  Serial.print("Boot Time: ");
  print_pretty_timestamp(bootTime);
  Serial.println();

  // SRAM init
  DEBUG_PRINTLN(F("Start SRAM"));
  sram_init(SRAM_CS_PIN); // Will set pin mode and such
  delay(1);

  bool restoredSram = false;

  // Try and restore from previous if we're not forced to not
  if (CLEAN_START == 0 && sram_get_populated()) {
    delay(1);

    // Restore the reading indexes and accumulator
    sram_read(&weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
    sram_read(&weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);
    sram_read_accumulator(&sampleAccumulator);

    if (
        weatherReadingWriteIndex > SRAM_MAX_READINGS ||
        weatherReadingReadIndex > SRAM_MAX_READINGS ||
        sampleAccumulator.timestamp == 0) {
      DEBUG_PRINTLN(F("ERROR: Sram Corrupted"));
    } else {
      DEBUG_PRINTLN(F("Ok! Restored from SRAM."));
      restoredSram = true;
    }
  }

  //  If we didn't restore it
  if (!restoredSram) {
    if (CLEAN_START) {
      DEBUG_PRINTLN(F("No SRAM, CLEAN_START"));
    } else {
      DEBUG_PRINTLN(F("ERROR: No state in SRAM."));
    }

    sram_write(weatherReadingWriteIndex, SRAM_ADDR_READINGS_WRITE_INDEX, SRAM_SIZE_READINGS_WRITE_INDEX);
    sram_write(weatherReadingReadIndex, SRAM_ADDR_READINGS_READ_INDEX, SRAM_SIZE_READINGS_READ_INDEX);

    zeroWeatherReading(&sampleAccumulator);
    sram_write_accumulator(&sampleAccumulator);
  }

  DEBUG_PRINT(F("Read Index: "));
  DEBUG_PRINTLN(weatherReadingReadIndex);
  DEBUG_PRINT(F("Write Index: "));
  DEBUG_PRINTLN(weatherReadingWriteIndex);
  Serial.println("Accumulator: ");
  printWeatherReading(sampleAccumulator);

  init_bme();

  // Wait a bit
  delay(500);

  String bootMessage = "BOOT,";

  // lsb first so 0
  bootMessage += ((mcusrBootValue>>PORF)&1)?"1":"0";
  bootMessage += ((mcusrBootValue>>EXTRF)&1)?"1":"0";
  bootMessage += ((mcusrBootValue>>BORF)&1)?"1":"0";
  bootMessage += ((mcusrBootValue>>WDRF)&1)?"1":"0";

  esp_send_debug_request(bootMessage);

  digitalWrite(OK_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);

  DEBUG_PRINTLN(F("Finished Init"));
  DEBUG_PRINTLN();
}

void esp_send_debug_request(String message) {
  espState = ESP_STATE_AWAITING_RESULT;
  espRequestTime = get_timestamp();

  Serial.print(F("Debug Request: "));
  Serial.println(message);

  send_esp_serial(ESP_MSG_REQUEST, ("/debug.php?"+message).c_str());
}

void esp_sleep() {
  DEBUG_PRINTLN(F("Put ESP to sleep now"));
  send_esp_serial(ESP_MSG_SLEEP, "true");
  espState = ESP_STATE_SLEEP;
}

void esp_reset() {
  wdt_reset();

  DEBUG_PRINTLN(F("Resetting ESP"));
  espState = ESP_STATE_RESETTING;
  espResetTime = get_timestamp();

  mcp.pinMode(MCP_ESP_RESET_PIN, OUTPUT);

  mcp.digitalWrite(MCP_ESP_RESET_PIN, LOW);
  delay(20);
  mcp.digitalWrite(MCP_ESP_RESET_PIN, HIGH);
  delay(20);
  mcp.digitalWrite(MCP_ESP_RESET_PIN, LOW);
  delay(20);

  mcp.pinMode(MCP_ESP_RESET_PIN, INPUT);
}

// bool serialCmdBuffering = false;
// char serialCmdBuffer[12]; // 11 char max serial cmd can hold timestamp string + cmd
// uint8_t serialCmdBufferIndex = 0;
//
// void handle_serial_command() {
//   if (!serialCmdBufferIndex) {
//     return;
//   }
//
//   char commandData[10];
//   //strcpy(commandData, serialCmdBuffer+1) = string(serialCmdBuffer).substr(1, serialCmdBufferIndex);
//
//   // New time
//   if (strcmp((const char*)&serialCmdBuffer[0], "t")) {
//     Serial.print("Set RTC To: ");
//     Serial.println(commandData);
//
//     //RTC.adjust(DateTime((uint32_t)strtoul(commandData)));
//   }
// }
//
// void receive_serial_commands() {
//   if (Serial.available()) {
//     const char rxByte = Serial.read();
//
//     if (serialCmdBuffering) {
//       serialCmdBuffer[serialCmdBufferIndex++] = rxByte;
//
//       if (serialCmdBufferIndex > sizeof(serialCmdBuffer)) {
//         serialCmdBuffering = false;
//         serialCmdBuffer[serialCmdBufferIndex++] = '\0';
//         handle_serial_command();
//         serialCmdBufferIndex = 0;
//       }
//     }
//
//     if (strcmp(&rxByte, "!") == 0) {
//       serialCmdBuffering = !serialCmdBuffering;
//     }
//   }
// }

void loop() {
  wdt_reset();

  // If we're waiting for the esp to send the result of a http request
  if (espState == ESP_STATE_AWAITING_RESULT) {
    // If it replied and it's over the reset wait time
    if (get_timestamp()-espRequestTime > ESP_REQUEST_DELAY && mcp.digitalRead(MCP_ESP_RESULT_PIN)) {
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
        DEBUG_PRINTLN(MAX_FAILED_SUBMITS-failedSubmits);

        if (failedSubmits >= MAX_FAILED_SUBMITS) {
          DEBUG_PRINTLN(F("Aborting (!)"));
          submitTimeoutCountdown = 1;
        }
      }

      // If we've submitted all or too many fails then put it back to sleep
      if (submitTimeoutCountdown > 0 || (weatherReadingReadIndex == weatherReadingWriteIndex)) {
        esp_sleep();
      }

    // If the esp timed out
    } else if (get_timestamp()-espRequestTime > ESP_REQUEST_TIMEOUT) {
      DEBUG_PRINTLN("Esp reply timeout.");
      // This'll start it again
      esp_reset();
    }
  // If it was resetting
  } else if (espState == ESP_STATE_RESETTING) {
      // If it's now pulled the pins low signifying it's awake and it's been an appropriate amount of time
      if (get_timestamp()-espResetTime > ESP_RESET_READY_DELAY &&
          mcp.digitalRead(MCP_ESP_RESULT_PIN) == 0 &&
          mcp.digitalRead(MCP_ESP_SUCCESS_PIN) == 0
        ) {
        // Idle now, on next loop we can send it a command
        espState = ESP_STATE_IDLE;
      // If it hasn't responded within the timeout it's probably frozen
      } else if (get_timestamp()-espResetTime > ESP_RESET_TIMEOUT) {
        DEBUG_PRINTLN("Esp reset timeout.");
        // This'll start it again
        esp_reset();
      }
  // ESP_STATE_IDLE
  } else if (espState == ESP_STATE_IDLE) {
    // No timeout condition and if there are results waiting for submission
    if (submitTimeoutCountdown <= 0 && weatherReadingReadIndex != weatherReadingWriteIndex) {
      submit_reading();
    // Means we're not submitting so put it back to sleep
    } else {
      esp_sleep();
    }
  }

  // Time to sample
  if (millis()-lastSampleMillis > SAMPLE_INTERVAL) {
    take_sample();
  }

  // If we've enough samples to make a reading
  if (sampleAccumulator.numSamples >= SAMPLES_PER_READING) {
    // Average the accumuator as a reading and store it
    WeatherReading currentReading = get_averaged_accumulator(sampleAccumulator);
    sram_write_reading(&currentReading, weatherReadingWriteIndex);

    // Need to have a test here that it wrote successfully, maybe reading and comaring or something?

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

      if (submitTimeoutCountdown == 0) {
        failedSubmits = 0;
      }
    }

    // This could potentially submit an invalid bme reading if it wasn't detected
    // for that reading and is now it'll submit invalid readings for the one that came before
    // Maybe have a bitflag set for the data for errors that occured during that reading
    // Try and detect the sensors for the next reading
    if (!bmeConnected) {
      init_bme();
    }

    // Wake the esp if it's sleeping
    if (espState == ESP_STATE_SLEEP) {
      esp_reset();
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
	      outputUrl += "&hum=";
	      outputUrl += weatherReading.humidity;

	      // If we have temperature and humidity then calculate and submit the heat index also
	      if (!isnan(weatherReading.temperature)) {
	        outputUrl += "&hI=";
	        outputUrl += computeHeatIndex(weatherReading.temperature, weatherReading.humidity, false);
	      }
	  }

	  if (!isnan(weatherReading.pressure)) {
	    outputUrl += "&pres=";
	    outputUrl += weatherReading.pressure;
	  }
	}

  outputUrl += "&bat=";
  outputUrl += weatherReading.batteryMv;
  outputUrl += "&wGust=";
  outputUrl += weatherReading.windGust;
  outputUrl += "&wSpd=";
  outputUrl += weatherReading.windSpeed;
  outputUrl += "&wDir=";
  outputUrl += weatherReading.windDirection;
	outputUrl += "&rain=";
	outputUrl += weatherReading.rain;
  outputUrl += "&lux=";
  outputUrl += weatherReading.lux;
  outputUrl += "&time=";
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

  for (int i=0; i<3; i++) {
    sram_read_reading(&currentReading, weatherReadingReadIndex);

    printWeatherReading(currentReading);

    // Maybe have this count and keep skip it if the read fails a few times?
    //possibly a crc or something?
    if (currentReading.timestamp == 0 || currentReading.timestamp > get_timestamp()) {
      DEBUG_PRINT(F("Corrupt reading: "));

      if (i < 2) {
        DEBUG_PRINTLN("Retry");
        continue;
      }

      // Need to have this make the ESP send something like a
      // debug message saying the reading was skipped and sending the data
      // of the reading

      DEBUG_PRINTLN(F("Skipping"));
      advance_read_pointer();
      return;
    }

    // No errors then break
    break;
  }

  String outputUrl = generate_request_url(currentReading);

  espState = ESP_STATE_AWAITING_RESULT;
  espRequestTime = get_timestamp();
  send_esp_serial(ESP_MSG_REQUEST, outputUrl.c_str());
}

int readADC(int channel=0, int oversampleBits=2) {
  Serial.print("ADC Channel ");
  Serial.println(channel);

  analogRead(channel);
  delay(1);

  // Oversample for 10 -> 10+oversampleBits bit adc resolution
  // 4^additionalBits (Expensive)
  short numSamples = (int)pow(4.0, oversampleBits);
  unsigned int adcAccumulator = 0;

  int reading = 0;
  for (int i=0; i<numSamples; i++) {
    adcAccumulator += analogRead(channel);
    delay(1);
  }

  return int(adcAccumulator/(float)numSamples);
}

float readADCVoltage(int channel=0, float ratio=1.0, int offset=0) {
  wdt_reset();

	// Get the vcc mV
	//////////////////////////////
	int referencePinMv = 2465; // Instead of 2500 as measured

	mcp.digitalWrite(MCP_REFV_ENABLE_PIN, LOW);
	delay(1); // For refV to stabilize

	float refPinReading  = readADC(REF_ADC_PIN);

	mcp.digitalWrite(MCP_REFV_ENABLE_PIN, HIGH);

	// Get the channel mV
	//////////////////////////////
	float channelPinReading = readADC(channel);

  float vccMv = (1023.0 / refPinReading) * referencePinMv;
  float pinMv = (vccMv / 1023.0) * channelPinReading;
	float readingMv = pinMv * ratio;

  DEBUG2_PRINT(F("ADC channel: "));
  DEBUG2_PRINT(channel);
  DEBUG2_PRINT(F(" refPin: "));
  DEBUG2_PRINT(refPinReading);
  DEBUG2_PRINT(F(" vccMv: "));
	DEBUG2_PRINT(vccMv);

  DEBUG2_PRINT(F(" channelPin: "));
  DEBUG2_PRINT(channelPinReading);
	DEBUG2_PRINT(F(" pinMv: "));
	DEBUG2_PRINT(pinMv);
  DEBUG2_PRINT(F(" readingMv: "));
  DEBUG2_PRINT(readingMv);
	DEBUG2_PRINT(F(" resultMv: "));
	DEBUG2_PRINTLN(readingMv+offset);

  return readingMv + offset;
}

void send_esp_serial(char messageType, const char *value) {
  wdt_reset();

  DEBUG_PRINT(F("(ATMega->ESP)"));
  DEBUG_PRINT((int)messageType);
  DEBUG_PRINT((int)strlen(value));
  DEBUG_PRINT(value);

  ESPSerial.write(messageType);
  ESPSerial.write((char)strlen(value));
  ESPSerial.write(value);

  DEBUG_PRINTLN(" (done)");
}

// bool recv_esp_serial() {
// 	// Print ESP serial messages
// 	if (ESPSerial.available() > 0) {
// 		uint8_t rxByte = ESPSerial.read();
//
// 		if (ESPNewline) {
// 			ESPNewline = false;
//
// 			// if (!ESPReplyBuffering) {
// 			// 	DEBUG_PRINT(F("ESP (recv)"));
// 			// }
// 		}
//
// 		// // Print it if it's not a reply to us (Just it trying to print something)
// 		// if (!ESPReplyBuffering && rxByte) {
// 		// 	Serial.print((char)rxByte);
// 		// }
//
// 		if (rxByte == 126) {
// 			ESPReplyBufferIndex = 0;
// 			ESPReplyBuffering = true;
// 		} else if (rxByte == 10) {
// 			ESPNewline = true;
// 			ESPReplyBuffering = false;
// 		}
//
// 		if (ESPReplyBuffering && ESPReplyBufferIndex < 5) {
// 			ESPReplyBuffer[ESPReplyBufferIndex++] = rxByte;
// 		}
// 	}
// }

void read_anemometer() {
  // No wind
  if (anemometerPulses == 0) {
    // DEBUG_PRINTLN(F("Avg Wind Tdelta: 0"));
    // DEBUG_PRINTLN(F("Wind MPH: 0"));
    return;
  }

  cli();
  // Get the average Tdelta for the anemometer since our last sample
  float averagePulseTDelta = anemometerTDeltaAccumulator / float(anemometerPulses);
  anemometerPulses = 0;
  anemometerTDeltaAccumulator = 0;
  sei();

  // DEBUG_PRINT(F("Avg Wind Tdelta: "));
  // DEBUG_PRINTLN(averagePulseTDelta);

  float anemometerMph = ANEMOMETER_TDELTA_1_MPH / averagePulseTDelta;

  // DEBUG_PRINT(F("Wind MPH: "));
  // DEBUG_PRINTLN(anemometerMph);

  sampleAccumulator.windSpeed += int32_t(anemometerMph*100);

  // DEBUG_PRINT(F("Wind Accumulator: "));
  // DEBUG_PRINTLN(sampleAccumulator.windSpeed);
  // DEBUG_PRINT(F("AccumAvg: "));
  // DEBUG_PRINTLN(float(sampleAccumulator.windSpeed/sampleAccumulator.numSamples)*.01);
}

void read_anemometer_gust() {
  // No wind
  if (anemometerMinTDelta == ULONG_MAX) {
    return;
  }

  cli();
  float anemometerGustMph = ANEMOMETER_TDELTA_1_MPH / anemometerMinTDelta;
  anemometerMinTDelta = ULONG_MAX;
  sei();

  // DEBUG_PRINT(F("Sample Gust MPH: "));
  // DEBUG_PRINTLN(anemometerGustMph);

  int32_t gustInt32 = int32_t(anemometerGustMph*100);

  if (gustInt32 > sampleAccumulator.windGust) {
    sampleAccumulator.windGust = gustInt32;

    // DEBUG_PRINT(F("NEW GUST RECORD: "));
    // DEBUG_PRINTLN(anemometerGustMph);
  }
}

// float read_anemometer() {
//   unsigned long currentReadingMillis = millis();
//
//   cli();
//   int numTimelyPulses = anemometerPulses;
//   anemometerPulses = 0;
//   sei();
//
//   DEBUG3_PRINT(F("Num wind pulses: "));
//   DEBUG3_PRINTLN(numTimelyPulses);
//
//   float anemometerMph = (numTimelyPulses * 0.5) * (60000.0 / (currentReadingMillis-lastAnemometerReadingMillis)) * ANEMOMETER_CALIBRATION_COEF;
//
//   lastAnemometerReadingMillis = currentReadingMillis;
//
//   return anemometerMph;
// }

int read_rain() {
  cli();
  int numTimelyPulses = rainBucketPulses;
  rainBucketPulses = 0;
  sei();

  return numTimelyPulses;
}

void read_wind_vane(float *destX, float *destY) {
  int windDegrees = map((int)readADC(WIND_VANE_ADC_PIN), WIND_VANE_MIN, WIND_VANE_MAX, 0, 360);
  float windRadians = windDegrees * M_PI / 180;

  *destY += sin(windRadians);
  *destX += cos(windRadians);

  // DEBUG3_PRINT(F("Wind Degrees: "));
  // DEBUG3_PRINTLN(windDegrees);
}

float read_battery_voltage() {
  //1.33511348465; // R2/R1 3.008/1.002
  float dividerRatio = 1.335;

  // Disable the solar panel to not interfere with the readings
  mcp.digitalWrite(MCP_SOLAR_ENABLE_PIN, LOW);
  delay(10);

  // Switch on the voltage divider for the battery and charge the cap and stabilize reference voltage
  mcp.digitalWrite(MCP_BAT_DIV_ENABLE_PIN, HIGH);
  delay(10); // Wait a bit

  // Voltage afterwards
	float batteryVoltage = readADCVoltage(BAT_ADC_PIN, dividerRatio);

  // Turn the divider back off
  mcp.digitalWrite(MCP_BAT_DIV_ENABLE_PIN, LOW);

  // Turn the solar panel back on
  mcp.digitalWrite(MCP_SOLAR_ENABLE_PIN, HIGH);

  return batteryVoltage;
}

uint16_t read_lux() {
  if (!luxConnected) {
    return 0;
  }

  luxMeter.begin(BH1750_ONE_TIME_HIGH_RES_MODE); // One shot then sleep
  return luxMeter.readLightLevel();
}

void read_tph(float *dest) {
  //******************************************/
  //* TEMPERATURE, HUMIDITY, BAROMETER BLOCK */
  //******************************************/
  // Are we sure were getting a reading?
  // also maybe have flags for if the temperature isn't reading correctly?
  // maybe send and alert

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
  DEBUG2_PRINT("Esp State: ");
  DEBUG2_PRINTLN(espState);
  DEBUG2_PRINT(F("Uptime: "));
  DEBUG2_PRINT((get_timestamp()-bootTime)/60.0);
  DEBUG2_PRINT(F(" (mins) since "));
  if (DEBUG >= 2) {
    print_pretty_timestamp(bootTime);
  }
  DEBUG2_PRINTLN();
  #endif

  lastSampleMillis = millis();

  sampleAccumulator.timestamp = get_timestamp();

  // Battery
  if (sampleAccumulator.numSamples%BATTERY_SAMPLE_MODULO == 0){
    sampleAccumulator.batteryMv += int32_t(read_battery_voltage()*100);
    sampleAccumulator.numBatterySamples++;
  }

  // Wind
  read_anemometer();
  read_anemometer_gust();
  read_wind_vane(&sampleAccumulator.windDirectionX, &sampleAccumulator.windDirectionY);

  // Rain
  sampleAccumulator.rain += read_rain();

  // Humidity, Temperature, Pressure
  if (bmeConnected)
  {
    float bmeReadings[3];
    read_tph(bmeReadings);

    sampleAccumulator.temperature += int32_t(bmeReadings[0]*100);
    sampleAccumulator.pressure += int32_t(bmeReadings[1]*100);
    sampleAccumulator.humidity += int32_t(bmeReadings[2]*100);
  }

  // Lux
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
