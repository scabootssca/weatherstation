
// #define SUBMISSION_URL "http://192.168.1.160"
//
// class ResultStream: public StreamString {
// private:
// 	int sampleIndex = 0;
//
// public:
// 	bool append_reading() {
// 		Serial.print("Current length: ");
// 		Serial.println(length());
//
// 		if (length()) {
// 			return true;
// 		}
//
// 		if (sampleIndex < MAX_READING_STORAGE) {
// 			if (storedReadings[sampleIndex].populated == false) {
// 				Serial.print("Unpopulated at index: ");
// 				Serial.println(sampleIndex);
// 				return false;
// 			}
//
// 			char buffer[20];
// 			sprintf(buffer, "%i,", storedReadings[sampleIndex].timestamp);
//
// 			Serial.print("Next buffer: ");
// 			Serial.println(String(buffer));
//
// 			sampleIndex++;
//
// 			concat(buffer);//this += buffer; //write((const uint8_t *)buffer, strlen(buffer));
// 			return true;
// 		}
// 		return false;
// 	}
//
// 	int available() {
// 		Serial.println("Checking available");
//
// 		// if (!append_reading() && !length()) {
// 		// 	return -1;
// 		// }
//
// 		return length();
// 	}
// };

// #if SEND_RESULT_STREAM
// void submit_readings() {
// 	if (connectToWiFi()) {
// 		WiFiClient client;
// 		char buffer[300] = {};
//
// 		//const char* host="http://192.168.1.160/";
// 		//String PostData = "title=foo&body=bar&userId=1";
//
// 		if (!client.connect(IPAddress(192, 168, 1, 160), 80)) {
// 			Serial.println("Connection Failed");
// 			return;
// 		}
//
// 		client.println("POST /submit_new.php HTTP/1.1");
// 		client.println("Host: 192.168.1.160");
// 		client.println("Accept: */*");
// 		client.println("Connection: close");
// 		//
// 		//client.println("Cache-Control: no-cache");
// 		//client.println("Content-Type: application/x-www-form-urlencoded");
// 		//client.println("Content-Length: 0");
// 		client.println();
//
// 		// Go through the readings and submit any that are there
// 		for (int i=0; i<MAX_READING_STORAGE; i++) {
// 			// If we've reached an unpopulated reading then we've reached the end and stop
// 			if (storedReadings[i].populated == false) {
// 				break;
// 			}
//
// 			// Need strcpy to init array at beginning for each reading
// 			strcpy(buffer, "timestamp=");
// 			strcat(buffer, String(storedReadings[i].timestamp).c_str());
//
// 			if (!isnan(storedReadings[i].temperature)) {
// 				strcat(buffer, "&temp=");
// 				strcat(buffer, String(storedReadings[i].temperature).c_str());
// 			}
//
// 			if (!isnan(storedReadings[i].humidity)) {
// 					strcat(buffer, "&humidity=");
// 					strcat(buffer, String(storedReadings[i].humidity).c_str());
//
// 					// If we have temperature and humidity then calculate and submit the heat index also
// 					if (!isnan(storedReadings[i].temperature)) {
// 						strcat(buffer, "&heatIndex=");
// 						strcat(buffer, String(computeHeatIndex(storedReadings[i].temperature, storedReadings[i].humidity, false)).c_str());
// 					}
// 			}
//
// 			if (!isnan(storedReadings[i].pressure)) {
// 				strcat(buffer, "&pressure=");
// 				strcat(buffer, String(storedReadings[i].pressure).c_str());
// 			}
//
// 			strcat(buffer, "&bat=");
// 			strcat(buffer, String(storedReadings[i].battery).c_str());
// 			strcat(buffer, "&windSpeed=");
// 			strcat(buffer, String(storedReadings[i].windSpeed).c_str());
// 			strcat(buffer, "&windDirection=");
// 			strcat(buffer, String(storedReadings[i].windDirection).c_str());
//
//
// 			Serial.print("Buffer: ");
// 			Serial.println(buffer);
//
// 			client.println(buffer);
// 		}
//
// 		// Output the response
// 		Serial.println("Result: ");
//
// 		while (client.available()) {
// 	    String line = client.readStringUntil('\r');
// 	    Serial.print(line);
// 		}
//
// 		Serial.println("End result");
// 	}
// 	// Serial.println("\n>>>> Submitting mass readings");
//   //
// 	// HTTPClient httpClient;
//   //
// 	// httpClient.begin("192.168.1.160", 80, "/report.php");//SERVER_IP_ADDRESS, 80, "/submit_new.php");
// 	// //http.setReuse();
// 	// //http.sendHeader('POST');
//   //
// 	// Serial.println("Began httpClient");
//   //
// 	// // Connect if needed
//   // if (connectToWiFi()) {
// 	// 	// Serial.println("Sending Request!");
//   //   //
// 	// 	// ResultStream dataStream;
// 	// 	// dataStream.concat("BALLS");
// 	// 	// int result = httpClient.sendRequest("POST", &dataStream, 0);
//   //   //
// 	// 	// Serial.print("Http Code: ");
// 	// 	// Serial.println(result);
// 	// 	httpClient.sendHeader("POST");
//   //
// 	// 	WiFiClient& httpStream = httpClient.getStream();
//   //
// 	// 	char buffer[] = "Balls";
//   //
// 	// 	//HTTP_TCP_BUFFER_SIZE
// 	// 	int bytesWrite = httpStream.write((const uint8_t *) buffer, strlen(buffer));
// 	// 	Serial.print("Wrote bytes: ");
// 	// 	Serial.println(bytesWrite);
//   //
//   //
// 	// 	// String payload = httpClient.getString();
//   //   //
// 	// 	// Serial.println("Got payload: ");
// 	// 	// Serial.println(payload);
// 	//}
// }
// #endif
