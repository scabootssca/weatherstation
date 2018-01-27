// void ICACHE_FLASH_ATTR readFromMaster() {
//   // SPI initialization configuration, speed = 0 in slave mode
//   SpiAttr hSpiAttr;
//
// }

#include <Wire.h>
#include "Adafruit_MCP23008.h"

#define SCL_PIN D1
#define SDA_PIN D3

#define ONEWIRE_RX_PIN 5
#define ONEWIRE_TX_PIN 6

#define RED_LED_PIN 0
#define GRN_LED_PIN 1

Adafruit_MCP23008 mcp;

#include "McpComn.h"

bool pinState = true;

unsigned long lastRequestMillis = 0;
unsigned long lastLedChangeMillis = 0;

void setup() {
  Serial.begin(19200);

  Serial.println();
  Serial.println("Initilizing..");

  Wire.begin(SDA_PIN, SCL_PIN);

  mcp.begin();

  mcp.pinMode(RED_LED_PIN, OUTPUT);
  mcp.pinMode(GRN_LED_PIN, OUTPUT);
  mcp.digitalWrite(RED_LED_PIN, LOW);
  mcp.digitalWrite(GRN_LED_PIN, HIGH);

  mcp.pinMode(ONEWIRE_TX_PIN, OUTPUT);
  mcp.pinMode(ONEWIRE_RX_PIN, INPUT);
}


void loop() {
  if (millis()-lastLedChangeMillis > 250) {
    lastLedChangeMillis = millis();
    pinState = !pinState;

    Serial.print(".");

    mcp.digitalWrite(RED_LED_PIN, !pinState);
    mcp.digitalWrite(GRN_LED_PIN, pinState);
  }

  if (millis()-lastRequestMillis > 2000) {
    lastRequestMillis = millis();
    Serial.println("");
    Serial.println("Sending Request");

    // response = [rainPulseCount][anemometerPulseCount]
    uint16_t result = sendRequest();

    // Serial.print("Result: ");
    // Serial.println(result);
    uint8_t anemometerPulseCount = result & 0b0000000011111111; // We only want the lower 8 bits
    uint8_t rainPulseCount = result>>8; // Only the upper 8

    Serial.print("Anemometer Count: ");
    Serial.println(anemometerPulseCount);
    Serial.print("Rain Count: ");
    Serial.println(rainPulseCount);

    //Serial.println("Sleeping now.");
    //ESP.deepSleep(0);
  }
}
