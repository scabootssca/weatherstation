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

Adafruit_MCP23008 mcp;

#include "McpComn.h"

bool pinState = true;

void setup() {
  Serial.begin(19200);

  Wire.begin(SDA_PIN, SCL_PIN);

  mcp.begin();

  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.digitalWrite(0, LOW);
  mcp.digitalWrite(1, HIGH);

  mcp.pinMode(ONEWIRE_TX_PIN, OUTPUT);
  mcp.digitalWrite(ONEWIRE_TX_PIN, HIGH);
  mcp.pinMode(ONEWIRE_RX_PIN, INPUT);
}


void loop() {
  pinState = !pinState;

  Serial.print("Tick ");
  Serial.println(pinState ? "True" : "False");

  mcp.digitalWrite(0, !pinState);
  mcp.digitalWrite(1, pinState);

  int result = sendRequest();

  Serial.print("Response: ");
  Serial.println(result);

  delay(10000);
}
