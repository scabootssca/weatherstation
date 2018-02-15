#include <ESP8266WiFi.h>
#include "SoftwareSerial.h"
//#include "SPISlave.h"

#include "config.h"
#include "helpers.h"

#define RESULT_PIN D6
#define SUCCESS_PIN D5

#define ATMEGA_RX_PIN D1
#define ATMEGA_TX_PIN D2

SoftwareSerial ATmegaSerial(ATMEGA_RX_PIN, ATMEGA_TX_PIN);

#define ATMEGA_BUFFER_SIZE 300
char ATmegaSerialBuffer[ATMEGA_BUFFER_SIZE];
int ATmegaSerialIndex = 0;

#define STATE_WAIT 0
#define STATE_CMD_RCVD 1
#define STATE_RECIEVING 3
#define STATE_FINISHED 4

int ATmegaSerialState = STATE_WAIT;
char ATmegaSerialCommand = 0;
char ATmegaSerialLength = 0;
//
// void on_spi_data(uint8_t *data, size_t len) {
//   String message = String((char *)data);
//
//   if(message.equals("Hello Slave!")) {
//       SPISlave.setData("Hello Master!");
//   } else if(message.equals("Are you alive?")) {
//       char answer[33];
//       sprintf(answer,"Alive for %u seconds!", millis() / 1000);
//       SPISlave.setData(answer);
//   } else {
//       SPISlave.setData("Say what?");
//   }
//   Serial.printf("Question: %s\n", (char *)data);
// }
//
// void on_spi_data_sent() {
//   Serial.println("Answer Sent");
// }
//
// void on_status(uint32_t data) {
//   Serial.printf("Status: %u\n", data);
//   SPISlave.setStatus(millis()); //set next status
// }
//
// void on_status_sent() {
//   Serial.println("Status Sent");
// }
//
// void setup_spi_slave() {
//   SPISlave.onData(on_spi_data);
//   SPISlave.onDataSent(on_spi_data_sent);
//   SPISlave.onStatus(on_status);
//   SPISlave.onStatusSent(on_status_sent);
//   SPISlave.begin();
//
//   // Set the status register (if the master reads it, it will read this value)
//   SPISlave.setStatus(millis());
//
//   // Sets the data registers. Limited to 32 bytes at a time.
//   // SPISlave.setData(uint8_t * data, size_t len); is also available with the same limitation
//   SPISlave.setData("Ask me a question!");
// }

void setup()
{
  Serial.begin(19200);
  Serial.setDebugOutput(true);

  ATmegaSerial.begin(ESP_ATMEGA_BAUD_RATE);
  // We print this to make the serial sync before the commands are sent
  ATmegaSerial.println("................");

  Serial.println(".......");
  Serial.print("Initilizing ESP8266");

  WiFi.setAutoConnect(false);
  // WiFi.mode(WIFI_OFF);
	// WiFi.persistent(false);

  pinMode(RESULT_PIN, OUTPUT);
  pinMode(SUCCESS_PIN, OUTPUT);

  digitalWrite(RESULT_PIN, LOW);
  digitalWrite(SUCCESS_PIN, LOW);
  //setup_spi_slave();
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

      if (ATmegaSerialIndex >= ATMEGA_BUFFER_SIZE-1) {
        Serial.println("Serial buffer overflow!");
        ATmegaSerialIndex = 0;
      }
    }
  }

  if (ATmegaSerialState == STATE_FINISHED) {
    ATmegaSerialState = STATE_WAIT;

    if (ATmegaSerialCommand == ESP_MSG_PING) {
      ATmegaSerial.println(String("Pong: ")+ATmegaSerialBuffer);
    } else if (ATmegaSerialCommand == ESP_MSG_REQUEST) {
      digitalWrite(RESULT_PIN, LOW);
      digitalWrite(SUCCESS_PIN, LOW);

      if (send_request(ATmegaSerialBuffer)) {
        digitalWrite(SUCCESS_PIN, HIGH);
        Serial.println("Succesfully sent request");
      } else {
        digitalWrite(SUCCESS_PIN, LOW);
        Serial.println("Request failed (!)");
      }

      digitalWrite(RESULT_PIN, HIGH);
    } else if (ATmegaSerialCommand == ESP_MSG_SLEEP) {
      Serial.println("Recieved Sleep Command; Sleeping Now.");
      delay(50);
      ESP.deepSleep(0);
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

bool send_request(char *url) {
  if (!connect_to_wifi()) {
    return false;
  }

  WiFiClient client;
  client.setTimeout(1000);

  Serial.print("[Connecting to ");
  Serial.print(SERVER_IP_ADDRESS);
  Serial.print(" ");

	if (!client.connect(SERVER_IP_ADDRESS, SERVER_PORT)) {
    Serial.println("Failed]");
    return false;
  }

  Serial.println("Success!]");

  // Send the request
	Serial.println("[Sending Request]");
  Serial.print(SERVER_IP_ADDRESS);
  Serial.print(":");
  Serial.print(SERVER_PORT);
  Serial.println(url);

	client.print(String("GET ") + url + " HTTP/1.1\r\n" +
		"Host: " + SERVER_IP_ADDRESS + "\r\n" +
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

  bool success = false;

  while (client.available()) {
    // It seems that readStringUntil blocks if it doesn't find the char again
    String line = client.readStringUntil('\n');

    if (line.startsWith("Success")) {
      success = true;
    }

    Serial.println(line);
  }

  client.stop();
  
  Serial.print("\n[Success: ");
  Serial.print(success?"True":"False");
  Serial.println("]");

  Serial.println("[Closed Client Connection]");

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


  WiFi.hostname("weather_station");
  WiFi.config(wifi_ip, wifi_gateway, wifi_subnet);
  //WiFi.mode(WIFI_STA);

  if (!WiFi.begin(WIFI_SSID, WIFI_PASS)) {
    return false;
  }

  int timeout = 30000;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1);
    timeout--;

    if (timeout%50 == 0) {
      Serial.print('.');
    }

    if (timeout <= 0) {
      DEBUG_PRINTLN("Timeout (!)]");
      return false;
    }
  }

  DEBUG_PRINTLN("Success]");
  return true;
}

bool disconnect_from_wifi() {
  DEBUG_PRINT("[Disconnecting From Wifi: ");
  bool result = WiFi.disconnect(true);
  DEBUG_PRINTLN(result?"Success]":"Failed (!)]");
  return result;
}
