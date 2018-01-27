/*
On Esp:
  int: 4
  double: 8
  float: 4
  char: 1
  long: 4
*/
//
#define SRAM_MODE_READ 0
#define SRAM_MODE_WRITE 1
#define SRAM_MAX_ADDRESS 0x1FFFF

/***************************
* SRAM memory layout
***************************/
// Hopefully it won't randomly be this
#define SRAM_ADDR_POPULATED 0
#define SRAM_SIZE_POPULATED 1

// Reading Buffer Pointers
#define SRAM_ADDR_READINGS_READ_INDEX 1
#define SRAM_SIZE_READINGS_READ_INDEX 4
#define SRAM_ADDR_READINGS_WRITE_INDEX 5
#define SRAM_SIZE_READINGS_WRITE_INDEX 4

// Ongoing accumulator
#define SRAM_ADDR_ACCUMULATOR 10
#define SRAM_SIZE_ACCUMULATOR sizeof(WeatherReadingAccumulator)
// Readings will directly follow in sram
#define SRAM_ADDR_READINGS SRAM_ADDR_ACCUMULATOR+SRAM_SIZE_ACCUMULATOR
#define SRAM_SIZE_READINGS sizeof(WeatherReading)
#define SRAM_MAX_READINGS int((SRAM_MAX_ADDRESS-SRAM_ADDR_READINGS)/SRAM_SIZE_READINGS)

bool sram_start(bool mode, int address, int size) {
  // Only 1mbit
  if (address+size > SRAM_MAX_ADDRESS) {
    return false;
  }

  int spiHz = 500000; /* Seems it doesn't it wants to take longer, so a slower clock */
  SPI.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
  mcp.digitalWrite(SRAM_CS_PIN, LOW);
  delay(1);

  SPI.transfer((mode == SRAM_MODE_READ)?0b00000011:0b00000010); // Read cmd: Write cmd

  SPI.transfer((uint8_t)address >> 16);
  SPI.transfer((uint8_t)address >> 8);
  SPI.transfer((uint8_t)address);

  return true;
}

void sram_end() {
  mcp.digitalWrite(SRAM_CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t sram_transfer(uint8_t value = 0) {
  // Serial.print("SRAM Transfer (uint8_t): ");
  // Serial.println(value);
  return SPI.transfer(value);
}

uint32_t sram_transfer(uint32_t value = 0) {
  // Serial.print("SRAM Transfer (uint32_t): ");
  // Serial.println(value);
  uint8_t *outBytes = reinterpret_cast<uint8_t*>(&value);
  uint8_t inBytes[sizeof(uint32_t)];

  SPI.transferBytes(outBytes, inBytes, sizeof(uint32_t));

  return *reinterpret_cast<uint32_t*>(&inBytes);
}

double sram_transfer(double value = 0) {
  // Serial.print("SRAM Transfer (double)(");
  // Serial.print(sizeof(double));
  // Serial.print("): ");
  // Serial.println(value);
  uint8_t *outBytes = reinterpret_cast<uint8_t*>(&value);
  uint8_t inBytes[sizeof(double)];

  // Serial.print("Out: ");
  // for (int i=0; i<sizeof(double);i++){
  //   for (int j=0;j<8;j++) {
  //     Serial.print(((outBytes[i]>>j)&1)?"1":"0");
  //   }
  // }
  // Serial.println();

  SPI.transferBytes(outBytes, inBytes, sizeof(double));

  // Serial.print("In: ");
  // for (int i=0; i<sizeof(double);i++){
  //   for (int j=0;j<8;j++) {
  //     Serial.print(((inBytes[i]>>j)&1)?"1":"0");
  //   }
  // }
  // Serial.println();

  double reply = *reinterpret_cast<double*>(&inBytes);

  // Serial.print("Reply: ");
  // Serial.println(reply);

  return reply;
}

float sram_transfer(float value = 0) {
  // Serial.print("SRAM Transfer (float): ");
  // Serial.println(value);
  uint8_t *outBytes = reinterpret_cast<uint8_t*>(&value);
  uint8_t inBytes[sizeof(float)];

  SPI.transferBytes(outBytes, inBytes, sizeof(float));

  return *reinterpret_cast<float*>(&inBytes);
}

void sram_read(uint32_t *dest, int address, int size) {
  sram_start(SRAM_MODE_READ, address, size);
  *dest = sram_transfer(uint32_t(0));
  sram_end();
}

void sram_write(uint32_t value, int address, int size) {
  sram_start(SRAM_MODE_WRITE, address, size);
  sram_transfer(value);
  sram_end();
}


void sram_write_reading(WeatherReading reading, int readingIndex) {
  int addrReading = SRAM_ADDR_READINGS + (SRAM_SIZE_READINGS * readingIndex);
  Serial.print("Writing WeatherReading to Sram Addr: ");
  Serial.println(addrReading);
  sram_start(SRAM_MODE_WRITE, addrReading, SRAM_SIZE_READINGS);

  sram_transfer(reading.timestamp);
  sram_transfer(reading.temperature);
  sram_transfer(reading.humidity);
  sram_transfer(reading.pressure);
  sram_transfer(reading.battery);
  sram_transfer(reading.windSpeed);
  sram_transfer(reading.windDirection);
  SPI.transfer(reading.populated);

  sram_end();
}

void sram_read_reading(WeatherReading *reading, int readingIndex) {
  int addrReading = SRAM_ADDR_READINGS + (SRAM_SIZE_READINGS * readingIndex);
  Serial.print("Reading WeatherReading from Sram Addr: ");
  Serial.println(addrReading);
  sram_start(SRAM_MODE_READ, addrReading, SRAM_SIZE_READINGS);

  reading->timestamp = sram_transfer(uint32_t(0));
  reading->temperature = sram_transfer(float(0));
  reading->humidity = sram_transfer(float(0));
  reading->pressure = sram_transfer(float(0));
  reading->battery = sram_transfer(float(0));
  reading->windSpeed = sram_transfer(float(0));
  reading->windDirection = sram_transfer(float(0));
  reading->populated = (bool)SPI.transfer(0);

  sram_end();
}


void sram_write_accumulator(WeatherReadingAccumulator accumulator) {
  sram_start(SRAM_MODE_WRITE, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);

  sram_transfer(accumulator.timestamp);
  sram_transfer(accumulator.temperature);
  sram_transfer(accumulator.humidity);
  sram_transfer(accumulator.pressure);
  sram_transfer(accumulator.battery);
  sram_transfer(accumulator.windSpeed);
  sram_transfer(accumulator.windDirection);
  sram_transfer(accumulator.numSamples);
  sram_transfer(accumulator.numBatterySamples);
  SPI.transfer(accumulator.populated);

  sram_end();
}

void sram_read_accumulator(WeatherReadingAccumulator *accumulator) {
  sram_start(SRAM_MODE_READ, SRAM_ADDR_ACCUMULATOR, SRAM_SIZE_ACCUMULATOR);

  accumulator->timestamp = sram_transfer(uint32_t(0));
  accumulator->temperature = sram_transfer(double(0));
  accumulator->humidity = sram_transfer(double(0));
  accumulator->pressure = sram_transfer(double(0));
  accumulator->battery = sram_transfer(double(0));
  accumulator->windSpeed = sram_transfer(double(0));
  accumulator->windDirection = sram_transfer(double(0));
  accumulator->numSamples = sram_transfer(uint32_t(0));
  accumulator->numBatterySamples = sram_transfer(uint32_t(0));
  accumulator->populated = (bool)SPI.transfer(0);

  sram_end();
}

bool sram_get_populated() {
  sram_start(SRAM_MODE_READ, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
  bool result = sram_transfer(uint8_t(0)) == 0b10101010;
  sram_end();

  return result;
}

void sram_set_populated(bool populated = true) {
  sram_start(SRAM_MODE_WRITE, SRAM_ADDR_POPULATED, SRAM_SIZE_POPULATED);
  if (populated) {
    sram_transfer(uint8_t(0b10101010));
  } else {
    sram_transfer(uint8_t(0));
  }
  sram_end();
}
