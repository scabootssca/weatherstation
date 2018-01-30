#include "SoftwareSerial.h"
#include "config.h"
#include "helpers.h"

#define ATMEGA_RX_PIN D1
#define ATMEGA_TX_PIN D2

SoftwareSerial ATmegaSerial(ATMEGA_RX_PIN, ATMEGA_TX_PIN);

char ATmegaSerialBuffer[300];
int ATmegaSerialIndex = 0;

#define STATE_WAIT 0
#define STATE_CMD_RCVD 1
#define STATE_RECIEVING 3
#define STATE_FINISHED 4

int ATmegaSerialState = STATE_WAIT;
char ATmegaSerialCommand = 0;
char ATmegaSerialLength = 0;

void setup()
{
  Serial.begin(19200);
  Serial.setDebugOutput(true);

  ATmegaSerial.begin(2400);
}

void loop() {
  if (ATmegaSerial.available()) {
    uint8_t rxByte = ATmegaSerial.read();

    if (ATmegaSerialState == STATE_WAIT) {
      ATmegaSerialState = STATE_CMD_RCVD;
      ATmegaSerialCommand = rxByte;
      // Serial.print("Got CMD: ");
      // Serial.println((int)ATmegaSerialCommand);
    } else if (ATmegaSerialState == STATE_CMD_RCVD) {
      ATmegaSerialLength = rxByte;

      if (ATmegaSerialLength == 0) {
        ATmegaSerialState = STATE_WAIT;
      } else {
        ATmegaSerialState = STATE_RECIEVING;
      }

      // Serial.print("Got LEN: ");
      // Serial.println((int)ATmegaSerialLength);
    } else if (ATmegaSerialState == STATE_RECIEVING) {
      ATmegaSerialBuffer[ATmegaSerialIndex++] = rxByte;

      if (ATmegaSerialIndex == ATmegaSerialLength) {
        ATmegaSerialState = STATE_FINISHED;
        ATmegaSerialBuffer[ATmegaSerialIndex++] = '\0';
      }
    }
  }

  if (ATmegaSerialState == STATE_FINISHED) {
    ATmegaSerialState = STATE_WAIT;

    if (ATmegaSerialCommand == ESP_MSG_PING) {
      ATmegaSerial.print(String("Pong: ")+ATmegaSerialBuffer);
    } else if (ATmegaSerialCommand == ESP_MSG_REQUEST) {
      send_request(ATmegaSerialBuffer);
    } else {
      ATmegaSerial.print("Recieved: ");
      ATmegaSerial.print(ATmegaSerialCommand, DEC);
      ATmegaSerial.print(" ");
      ATmegaSerial.print(ATmegaSerialLength, DEC);
      ATmegaSerial.print(" ");
      ATmegaSerial.println(ATmegaSerialBuffer);
    }

    ATmegaSerialCommand = 0;
    ATmegaSerialLength = 0;
    ATmegaSerialIndex = 0;
  }
}

void send_request(char *filename) {

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
