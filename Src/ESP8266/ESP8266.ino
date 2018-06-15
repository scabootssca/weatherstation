#include <ESP8266WiFi.h>
#include "SoftwareSerial.h"
#include <ESP8266WebServer.h>
//#include "SPISlave.h"
#include <Wire.h>
#define I2CAddressESPWifi 8

#include "config.h"
#include "helpers.h"

#define SUCCESS_PIN D1
#define RESULT_PIN D2

#define ATMEGA_RX_PIN D5
#define ATMEGA_TX_PIN D6

SoftwareSerial ATmegaSerial(ATMEGA_RX_PIN, ATMEGA_TX_PIN);

#define ATMEGA_BUFFER_SIZE 300
char ATmegaSerialBuffer[ATMEGA_BUFFER_SIZE];
int ATmegaSerialIndex = 0;

#define STATE_WAIT 0
#define STATE_CMD_RCVD 1
#define STATE_RECIEVING 3
#define STATE_FINISHED 4

#define REQUEST_TIMEOUT_MS 45000
#define CONNECT_TIMEOUT_MS 30000
#define WIFI_CLIENT_TIMEOUT_MS 30000

int ATmegaSerialState = STATE_WAIT;
char ATmegaSerialCommand = 0;
char ATmegaSerialLength = 0;

#define DEBUG_PIN D7
bool debugMode = false;
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

struct NetworkInfo {
  char ssid[100];
  int rssi;
  bool open;
};

unsigned short numDetectedNetworks = 0;
NetworkInfo *detectedNetworks;
const char *hostedSSID = "WeatherStation_AP";
IPAddress apIP(192, 168, 1, 1);
ESP8266WebServer server(80);

void scan_networks() {
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  if (numDetectedNetworks) {
    free(detectedNetworks);
  }

  // WiFi.scanNetworks will return the number of networks found
  numDetectedNetworks = WiFi.scanNetworks();
  Serial.println("scan done");


  Serial.print(numDetectedNetworks);
  Serial.println(" networks found");

  if (numDetectedNetworks) {
      detectedNetworks = (NetworkInfo*)malloc(sizeof(NetworkInfo)*numDetectedNetworks);

      for (int i = 0; i < numDetectedNetworks; ++i) {
        strcpy(detectedNetworks[i].ssid, WiFi.SSID(i).c_str());
        detectedNetworks[i].rssi = WiFi.RSSI(i);
        detectedNetworks[i].open = (WiFi.encryptionType(i) == ENC_TYPE_NONE);

        Serial.print(detectedNetworks[i].ssid);
        Serial.print(" ");
        Serial.print(detectedNetworks[i].rssi);
        Serial.print("dB [");
        Serial.print(detectedNetworks[i].open?"open":"enc");
        Serial.println("]");
      }
  }

  Serial.println("");
}

void handleRoot() {
  String response;

  response += "<h1>Detected Networks</h1>";

  for (int i=0; i<numDetectedNetworks; i++) {
    response += i;
    response += ". ";
    response += detectedNetworks[i].ssid;
    response += ": (";
    response += detectedNetworks[i].rssi;
    response += ")";
    response += (detectedNetworks[i].open?" ":"*");
    response += "<br>";
  }

	server.send(200, "text/html", response.c_str());
}

void setup()
{
  Serial.begin(19200);
  Serial.setDebugOutput(true);

  // Debug functions
  pinMode(DEBUG_PIN, INPUT);
  debugMode = digitalRead(DEBUG_PIN);

  Serial.println(".......");
  Serial.print("Initilizing ESP8266...........");
  Serial.print("Finished: ");
  Serial.println(debugMode?"Debug Mode":"Slave Mode");

  if (debugMode) {
    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    scan_networks();
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    delay(100);

    // Then setup as an ap
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(hostedSSID);
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.begin();

    Serial.println("Debug Server Started");
  } else {
    ATmegaSerial.begin(ESP_ATMEGA_BAUD_RATE);
    // We print this to make the serial sync before the commands are sent
    ATmegaSerial.println("......................");

    //WiFi.setAutoConnect(false);
    //connect_to_wifi();
  	// WiFi.persistent(false);

    //setup_spi_slave();
    //
    // Wire.begin(0, 2); //SDA, SCL, D3, D4//Change to Wire.begin() for non ESP.
  }

  pinMode(RESULT_PIN, OUTPUT);
  pinMode(SUCCESS_PIN, OUTPUT);

  digitalWrite(RESULT_PIN, LOW);
  digitalWrite(SUCCESS_PIN, LOW);

  Serial.println("Done init");
}

void loop() {
  if (debugMode) {
    server.handleClient();
    yield();
    return;
  }

  //ATmegaSerial.print("Balls\n");
  //delay(500);

  if (ATmegaSerial.available()) {
    uint8_t rxByte = ATmegaSerial.read();

    if (ATmegaSerialState == STATE_WAIT) {
      ATmegaSerialState = STATE_CMD_RCVD;
      ATmegaSerialCommand = rxByte;
      Serial.print("Got CMD: ");
      Serial.println((int)ATmegaSerialCommand);
    } else if (ATmegaSerialState == STATE_CMD_RCVD) {
      ATmegaSerialLength = rxByte;

      if (ATmegaSerialLength == 0) {
        ATmegaSerialState = STATE_WAIT;
      } else {
        ATmegaSerialState = STATE_RECIEVING;
      }

      Serial.print("Got LEN: ");
      Serial.println((int)ATmegaSerialLength);
    } else if (ATmegaSerialState == STATE_RECIEVING) {
      Serial.print((char)rxByte);

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
    } else if (ATmegaSerialCommand == ESP_MSG_IDLE) {
      Serial.println("Recieved Idle Command; Settings pins low.");
      digitalWrite(RESULT_PIN, LOW);
      digitalWrite(SUCCESS_PIN, LOW);
    } else if (ATmegaSerialCommand == ESP_MSG_SLEEP) {
      Serial.println("Recieved Sleep Command; Sleeping Now.");
      digitalWrite(RESULT_PIN, LOW);
      digitalWrite(SUCCESS_PIN, LOW);
      delay(50);
      //disconnect_from_wifi();
      ESP.deepSleep(0);
    }

    // ATmegaSerial.print("Recieved: ");
    // ATmegaSerial.print(ATmegaSerialCommand, DEC);
    // ATmegaSerial.print(" ");
    // ATmegaSerial.print(ATmegaSerialLength, DEC);
    // ATmegaSerial.print(" ");
    // ATmegaSerial.println(ATmegaSerialBuffer);

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
  client.setTimeout(WIFI_CLIENT_TIMEOUT_MS);

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
    if (millis()-timeout > REQUEST_TIMEOUT_MS) {
      Serial.println("....Timed out [!]");
      client.stop();
      return false;
    }
  }

  Serial.println("\n[Response:]");

  bool success = false;

  while (client.available()) {
    yield();

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


  //WiFi.hostname("weather_station");
  WiFi.config(wifi_ip, wifi_gateway, wifi_subnet);
  WiFi.mode(WIFI_STA);

  if (!WiFi.begin(WIFI_SSID, WIFI_PASS)) {
    return false;
  }

  int timeout = CONNECT_TIMEOUT_MS;
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

  //WiFi.RSSI()

  return true;
}

bool disconnect_from_wifi() {
  DEBUG_PRINT("[Disconnecting From Wifi: ");
  bool result = WiFi.disconnect(true);
  DEBUG_PRINTLN(result?"Success]":"Failed (!)]");
  return result;
}
