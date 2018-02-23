#ifndef _HELPERS_H_
#define _HELPERS_H_

#include "RTClib.h"

// 0 = Off
// 1 = Basic: Print Readings, Blink Indicator, ESP messages
// 2 = Extra: Basic + Print Samples, Battery Readings
// 3 = Full: Extra + In Depth Sample Info
#define DEBUG 1

#define ESP_MSG_PING 0
#define ESP_MSG_REQUEST 1
#define ESP_MSG_SLEEP 2
#define ESP_MSG_REQUEST_SUCCESS 3
#define ESP_MSG_REQUEST_FAIL 4
#define ESP_MSG_AWAKE 5

#if DEBUG
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#if DEBUG >= 2
#define DEBUG2_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG2_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG2_PRINT(...)
#define DEBUG2_PRINTLN(...)
#endif

#if DEBUG >= 3
#define DEBUG3_PRINT(...) Serial.print( __VA_ARGS__ )
#define DEBUG3_PRINTLN(...) Serial.println( __VA_ARGS__ )
#else
#define DEBUG3_PRINT(...)
#define DEBUG3_PRINTLN(...)
#endif

// void send_serial(SoftwareSerial *serial, char messageType, const char *value) {
//   Serial.print("SoftwareSerial (sent)");
//   Serial.print((int)messageType);
//   Serial.print((int)strlen(value));
//   Serial.print(value);
//   Serial.println();
//
//   serial->write(messageType);
//   serial->write((char)strlen(value));
//   serial->write(value);
// }

// // Return RSSI or 0 if target SSID not found
// int32_t getRSSI(const char* target_ssid) {
//   byte available_networks = WiFi.scanNetworks();
//
//   for (int network = 0; network < available_networks; network++) {
//     if (strcmp(WiFi.SSID(network).c_str(), target_ssid) == 0) {
//       return WiFi.RSSI(network);
//     }
//   }
//   return 0;
// }

int getFreeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void print_pretty_timestamp(uint32_t unixtime) {
	// For displaying them in local time
	DateTime readingTimeLocal = DateTime(DateTime(unixtime) + TimeSpan(60*60*GMT_OFFSET));

	Serial.print(unixtime, DEC);
	Serial.print(" ");
	Serial.print(readingTimeLocal.month(), DEC);
	Serial.print('/');
	Serial.print(readingTimeLocal.day(), DEC);
	Serial.print('/');
	Serial.print(readingTimeLocal.year(), DEC);
	Serial.print(' ');
	Serial.print(readingTimeLocal.hour(), DEC);
	Serial.print(':');
	Serial.print(readingTimeLocal.minute(), DEC);
	Serial.print(':');
	Serial.print(readingTimeLocal.second(), DEC);
}

float convertCtoF(float c) {
  return c * 1.8 + 32;
}

float convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float computeDewPoint(float T, float RH) {
	// From wikipedia article and `Environmental Monitoring with Arduino`
	float dewPoint = 0.0;
	float gTRH = 0.0;

	float a = 17.271;
	float b = 237.7;

	gTRH = ((a*T)/(b+T)) + log(RH/100);
	dewPoint = (b*gTRH)/(a-gTRH);

	return dewPoint;
}

float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit=true) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

// float getBatteryPercent(float batteryMv, float tempC=25.0) {
// 	float percent = 0;
//
// 	int loadMa = 25; // 25 milliamps
//
// 	int maxMv = 4200;
// 	int minMv = 3000;
//
// 	int curveTemps[3] = {-10, 0, 25};
// 	float curves[3][3] = {{3600, 3500, 3100}, {3700, 3600, 3100}, {4000, 3500, 3200}};
// 	float curvePercents[3][3] = {{98.0, 70.0, 1.00}, {98.0, 80.0, 10.0}, {98.0, 50.0, 20.0}};
//
// 	float steps[3] = {curves[2][0], curves[2][1], curves[2][2]};
// 	float percents[3] = {curvePercents[2][0], curvePercents[2][1], curvePercents[2][2]};
//
// 	// For the current capacitity changes based off C
// 	// int c = 1625;
// 	// int cPercent = loadMa/c;
// 	// float capacityCoef = 1.0;
//
// 	// 1c, 0.5c, 0.2c
// 	//int capacityTargets[3] = {2900, 3150, 3300};
//
// 	// Curve values are for at 1c and we're drawing less than 0.2c so use that capacity
// 	float capacityCoef = 1.0;//1.13793103448;
//
// 	for (int i=0; i<3; i++) {
// 		if (tempC < curveTemps[i]) {
// 			Serial.print("Using ");
// 			Serial.print(curveTemps[i]);
// 			Serial.println("*C curve");
//
// 			if (i == 0) {
// 				// Use -10c
// 				for (int valueIndex=0; valueIndex<3; valueIndex++) {
// 					steps[valueIndex] = curves[i][valueIndex];
// 					percents[valueIndex] = curvePercents[i][valueIndex];
// 				}
//
// 				break;
// 			}
//
// 			int range = curveTemps[i]-curveTemps[i-1];
// 			float coef = tempC/range;
//
// 			Serial.print("Curve Coef: ");
// 			Serial.println(coef);
//
// 			for (int valueIndex=0; valueIndex<3; valueIndex++) {
// 				steps[valueIndex] = (curves[i][valueIndex] + ((curves[i][valueIndex]-curves[i-1][valueIndex]) * coef)) * capacityCoef;
// 				percents[valueIndex] = (curvePercents[i][valueIndex] + ((curvePercents[i][valueIndex]-curvePercents[i-1][valueIndex]) * coef)) * capacityCoef;
// 			}
//
// 			break;
// 		}
// 	}
//
// 	Serial.print("Steps: ");
// 	Serial.print(maxMv);
// 	Serial.print(", ");
// 	Serial.print(steps[0]);
// 	Serial.print(", ");
// 	Serial.print(steps[1]);
// 	Serial.print(", ");
// 	Serial.print(steps[2]);
// 	Serial.print(", ");
// 	Serial.println(minMv);
//
// 	Serial.print("Percents: ");
// 	Serial.print("100, ");
// 	Serial.print(percents[0]);
// 	Serial.print(", ");
// 	Serial.print(percents[1]);
// 	Serial.print(", ");
// 	Serial.print(percents[2]);
// 	Serial.println(", 0");
//
// 	if (batteryMv > maxMv) {
// 		Serial.println("Using target mavMv");
// 		percent = 100;
// 	} else if (batteryMv > steps[0]) {
// 		Serial.println("Using target 0");
// 		percent = percents[0] + ((100-percents[0])*((maxMv-batteryMv)/(maxMv-steps[0])));
// 	} else if (batteryMv > steps[1]) {
// 		Serial.println("Using target 1");
// 		percent = percents[1] + ((percents[0]-percents[1])*((steps[0]-batteryMv)/(steps[0]-steps[1])));
// 	} else if (batteryMv > steps[2]) {
// 		Serial.println("Using target 2");
// 		percent = percents[2] + ((percents[1]-percents[2])*((steps[1]-batteryMv)/(steps[1]-steps[2])));
// 	} else {
// 		Serial.println("Using target minMv");
// 		percent = percents[2] * ((batteryMv-minMv)/(steps[2]-minMv));
// 	}
//
// 	return percent;
// }

#endif
