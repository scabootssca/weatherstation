#ifndef _MCP_23A1024_H_
#define _MCP_23A1024_H_
#include "weather_readings.h"

#define SRAM_DEBUG 0
#define SRAM_MODE_READ 0
#define SRAM_MODE_WRITE 1
#define SRAM_MAX_ADDRESS 0x1FFFF // 131072 (2^17) addresses

/***************************
* SRAM memory layout
***************************/
/*
[0-4]  BLANK
[5-9]    populated
[10-12]  readIndex
[13-19]  BLANK
[20-22]  writeIndex
[23-31]  BLANK
[32-endAccum] readingAccumulator
[endAccum+]   readings
*/
#define SRAM_POPULATED_KEY 0xFAD00BA6

// Hopefully it won't randomly be this
#define SRAM_ADDR_POPULATED 5
#define SRAM_SIZE_POPULATED 4

// Reading Buffer Pointers
#define SRAM_ADDR_READINGS_READ_INDEX 10
#define SRAM_SIZE_READINGS_READ_INDEX 2
#define SRAM_ADDR_READINGS_WRITE_INDEX 20
#define SRAM_SIZE_READINGS_WRITE_INDEX 2

// Ongoing accumulator
#define SRAM_ADDR_ACCUMULATOR 32
#define SRAM_SIZE_ACCUMULATOR sizeof(WeatherReadingAccumulator)

// Readings will directly follow in sram, next page (Each reading will use 1 page)
#define SRAM_ADDR_READINGS SRAM_ADDR_ACCUMULATOR+SRAM_SIZE_ACCUMULATOR
#define SRAM_SIZE_READINGS sizeof(WeatherReading)
#define SRAM_MAX_READINGS int((SRAM_MAX_ADDRESS-SRAM_ADDR_READINGS)/SRAM_SIZE_READINGS)

void print_bin(uint64_t bytes, int numBits, bool newLine=true) {
  // Bytes in the union have the lsb as 0

  if (newLine) {
    Serial.print("Bytes: ");
  }

  for (int bit=0; bit<numBits; bit++) {
    // Space between bytes
    if (bit > 0 && bit%8 == 0) {
      Serial.print(" ");
    }

    Serial.print( ((bytes >> (numBits-1-bit)) & 1) ? "1":"0" );
  }

  if (newLine) {
    Serial.println();
  }
}

void sram_init(int csPin) {
  Serial.println("Initilizing SRAM");

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);
  delay(1);
  digitalWrite(csPin, LOW);
  delay(1);
  digitalWrite(csPin, HIGH);

  Serial.print("SRAM Capacity: ");
  Serial.println(SRAM_MAX_READINGS);
}

bool sram_begin_transaction(bool mode, uint32_t address, int size) {
  // Only 1mbit
  if (address+size > SRAM_MAX_ADDRESS) {
    Serial.println("SRAM transaction cancelled (address overflow)");
    return false;
  }

  #if SRAM_DEBUG
  Serial.println();
  Serial.print("Starting SRAM ");
  Serial.print((mode == SRAM_MODE_READ)?"READ":"WRITE");
  Serial.print(" at addr: ");
  Serial.println(address);
  #endif

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SRAM_CS_PIN, LOW);
  delay(1);

  SPI.transfer((mode == SRAM_MODE_READ)?0b00000011:0b00000010); // Read cmd: Write cmd

  SPI.transfer(uint8_t(address >> 16));
  SPI.transfer(uint8_t(address >> 8));
  SPI.transfer(uint8_t(address));

  return true;
}

void sram_end_transaction() {
  digitalWrite(SRAM_CS_PIN, HIGH);
  SPI.endTransaction();

  #if SRAM_DEBUG
  Serial.println("Finished SRAM transfer");
  #endif
}

uint8_t sram_transfer(uint8_t value) {
  uint8_t result = SPI.transfer(value);

  #if SRAM_DEBUG > 1
  Serial.print("Sending (uint8_t): ");
  print_bin(value, 8, false);
  Serial.println();
  Serial.print("Recieved (uint8_t): ");
  print_bin(result, 16, false);
  Serial.println();
  #endif

  return result;
}

uint16_t sram_transfer(uint16_t value) {
  uint16_t result = SPI.transfer16(value);

  #if SRAM_DEBUG > 1
  Serial.print("Sending (uint16_t): ");
  print_bin(value, 16, false);
  Serial.println();
  Serial.print("Recieved (uint16_t): ");
  print_bin(result, 16, false);
  Serial.println();
  #endif

  return result;
}

uint32_t sram_transfer(uint32_t value) {
  union {
    uint32_t value;

    struct {
      uint16_t bit0;
      uint16_t bit1;
    };
  } send, recv;

  send.value = value;

  recv.bit0 = SPI.transfer16(send.bit0);
  recv.bit1 = SPI.transfer16(send.bit1);

  #if SRAM_DEBUG > 1
  Serial.print("Sending (uint32_t): ");
  print_bin(value, 32, false);
  Serial.println();
  Serial.print("Recieved (uint32_t): ");
  print_bin(recv.value, 32, false);
  Serial.println();
  #endif

  return recv.value;
}

uint64_t sram_transfer(uint64_t value) {
  union {
    uint64_t value;

    struct {
      uint16_t bit0;
      uint16_t bit1;
      uint16_t bit2;
      uint16_t bit3;
    };
  } send, recv;

  send.value = value;

  recv.bit0 = SPI.transfer16(send.bit0);
  recv.bit1 = SPI.transfer16(send.bit1);
  recv.bit2 = SPI.transfer16(send.bit2);
  recv.bit3 = SPI.transfer16(send.bit3);

  #if SRAM_DEBUG > 1
  Serial.print("Sending (uint64_t): ");
  print_bin(value, 64, false);
  Serial.println();
  Serial.print("Recieved (uint64_t): ");
  print_bin(recv.value, 64, false);
  Serial.println();
  #endif

  return recv.value;
}

uint32_t float_to_uint32(float value) {
  union {
    uint32_t intVal;
    float floatVal;
  } conversion;

  conversion.floatVal = value;

  return conversion.intVal;
}

float uint32_to_float(uint32_t value) {
  union {
    uint32_t intVal;
    float floatVal;
  } conversion;

  conversion.intVal = value;

  return conversion.floatVal;
}

void sram_read_accumulator(WeatherReadingAccumulator *sampleAccumulator) {
  sram_begin_transaction(SRAM_MODE_READ, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);

  sampleAccumulator->timestamp = sram_transfer(uint32_t(0));
  sampleAccumulator->temperature = static_cast<int32_t>(sram_transfer(uint32_t(0))); // We want it signed
  sampleAccumulator->humidity = sram_transfer(uint32_t(0));
  sampleAccumulator->pressure = sram_transfer(uint64_t(0));
  sampleAccumulator->batteryMv = sram_transfer(uint32_t(0));
  sampleAccumulator->windSpeed = sram_transfer(uint32_t(0));
  sampleAccumulator->windDirectionX = uint32_to_float(sram_transfer(uint32_t(0)));
  sampleAccumulator->windDirectionY = uint32_to_float(sram_transfer(uint32_t(0)));
  sampleAccumulator->rain = sram_transfer(uint32_t(0));
  sampleAccumulator->lux = sram_transfer(uint64_t(0));

  sampleAccumulator->numSamples = sram_transfer(uint8_t(0));
  sampleAccumulator->numBatterySamples = sram_transfer(uint8_t(0));

  sram_end_transaction();
}

void sram_write_accumulator(WeatherReadingAccumulator *sampleAccumulator) {
  sram_begin_transaction(SRAM_MODE_WRITE, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);

  sram_transfer(sampleAccumulator->timestamp);
  sram_transfer(static_cast<uint32_t>(sampleAccumulator->temperature));
  sram_transfer(sampleAccumulator->humidity);
  sram_transfer(sampleAccumulator->pressure);
  sram_transfer(sampleAccumulator->batteryMv);
  sram_transfer(sampleAccumulator->windSpeed);
  sram_transfer(float_to_uint32(sampleAccumulator->windDirectionX));
  sram_transfer(float_to_uint32(sampleAccumulator->windDirectionY));
  sram_transfer(sampleAccumulator->rain);
  sram_transfer(sampleAccumulator->lux);

  sram_transfer(sampleAccumulator->numSamples);
  sram_transfer(sampleAccumulator->numBatterySamples);

  sram_end_transaction();
}

bool sram_restore(WeatherReadingAccumulator *sampleAccumulator) {
  // See if the populated key was set
  sram_begin_transaction(SRAM_MODE_READ, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
  uint32_t storedValue = sram_transfer(uint32_t(0));
  bool previouslyPopulated = (storedValue-SRAM_POPULATED_KEY) == 0;
  sram_end_transaction();

  Serial.print("Populated: ");
  Serial.println(previouslyPopulated);

  Serial.print("Stored cookie: ");
  print_bin(storedValue, 32, false);
  Serial.println();

  Serial.print("Verify cookie: ");
  print_bin(SRAM_POPULATED_KEY, 32, false);
  Serial.println();

  // Set the populated key
  sram_begin_transaction(SRAM_MODE_WRITE, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
  sram_transfer(uint32_t(SRAM_POPULATED_KEY));
  sram_end_transaction();

  // Read the accumulator here
  if (previouslyPopulated) {
    sram_read_accumulator(sampleAccumulator);

    Serial.println("Restored Accumulator: ");
    printWeatherReading(*sampleAccumulator);

  } else {
    sram_write_accumulator(sampleAccumulator);
  }

  return previouslyPopulated ;
}

void sram_read_reading(WeatherReading *weatherReading, uint32_t readingIndex) {
  uint32_t addrReading = SRAM_ADDR_READINGS + (SRAM_SIZE_READINGS * readingIndex);

  Serial.print("Reading Reading from Addr: ");
  Serial.println(addrReading);

  sram_begin_transaction(SRAM_MODE_READ, addrReading, SRAM_SIZE_READINGS);

  weatherReading->timestamp = sram_transfer(uint32_t(0));
  weatherReading->temperature = uint32_to_float(sram_transfer(uint32_t(0)));
  weatherReading->humidity = uint32_to_float(sram_transfer(uint32_t(0)));
  weatherReading->pressure = uint32_to_float(sram_transfer(uint32_t(0)));
  weatherReading->batteryMv = uint32_to_float(sram_transfer(uint32_t(0)));
  weatherReading->windSpeed = uint32_to_float(sram_transfer(uint32_t(0)));
  weatherReading->windDirection = uint32_to_float(sram_transfer(uint32_t(0)));
  weatherReading->rain = sram_transfer(uint32_t(0));
  weatherReading->lux = uint32_to_float(sram_transfer(uint32_t(0)));

  sram_end_transaction();
}

void sram_write_reading(WeatherReading *weatherReading, uint32_t readingIndex) {
  uint32_t addrReading = SRAM_ADDR_READINGS + (SRAM_SIZE_READINGS * readingIndex);

  Serial.print("Writing Reading to Addr: ");
  Serial.println(addrReading);

  sram_begin_transaction(SRAM_MODE_WRITE, addrReading, SRAM_SIZE_READINGS);

  sram_transfer(weatherReading->timestamp);
  sram_transfer(float_to_uint32(weatherReading->temperature));
  sram_transfer(float_to_uint32(weatherReading->humidity));
  sram_transfer(float_to_uint32(weatherReading->pressure));
  sram_transfer(float_to_uint32(weatherReading->batteryMv));
  sram_transfer(float_to_uint32(weatherReading->windSpeed));
  sram_transfer(float_to_uint32(weatherReading->windDirection));
  sram_transfer(weatherReading->rain);
  sram_transfer(float_to_uint32(weatherReading->lux));

  sram_end_transaction();
}

void sram_read(uint16_t *dest, uint32_t address, unsigned int size) {
  sram_begin_transaction(SRAM_MODE_READ, address, size);
  *dest = sram_transfer(uint16_t(0));
  sram_end_transaction();
}

void sram_write(uint16_t value, uint32_t address, unsigned int size) {
  sram_begin_transaction(SRAM_MODE_WRITE, address, size);
  sram_transfer(value);
  sram_end_transaction();
}
#endif
