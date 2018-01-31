#ifndef _HELPERS_H_
#define _HELPERS_H_

#include "RTClib.h"

#define DEBUG 3

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

struct WeatherReading {
	uint32_t timestamp;
	float temperature = 0;
	float humidity = 0;
	float pressure = 0;
	float battery = 0;
	float windSpeed = 0;
	float windDirection = 0;
};

struct WeatherReadingAccumulator {
	uint64_t timestamp;
	double temperature = 0;
	double humidity = 0;
	double pressure = 0;
	double battery = 0;
	double windSpeed = 0;
	double windDirection = 0;

	uint32_t numSamples = 0;
	uint32_t numBatterySamples = 0;
};

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

void printWeatherReading(WeatherReading reading) {
	Serial.print("Timestamp: ");
	print_pretty_timestamp(reading.timestamp);
	Serial.println();

	Serial.print("Temp: ");
	Serial.print(reading.temperature);
	Serial.println("*C");
	Serial.print("Pressure: ");
	Serial.print(reading.pressure);
	Serial.println("hpa");
	Serial.print("Humidity: ");
	Serial.print(reading.humidity);
	Serial.println('%');
	Serial.print("Battery: ");
	Serial.print(reading.battery);
	Serial.println("mV");
	Serial.print("Wind Speed: ");
	Serial.print(reading.windSpeed);
	Serial.println("mph");
	Serial.print("Wind Direction: ");
	Serial.print(reading.windDirection);
	Serial.println("deg");
}

void printWeatherReading(WeatherReadingAccumulator reading) {
	// For displaying them in local time
	DateTime readingTimeLocal = DateTime(DateTime(reading.timestamp) + TimeSpan(60*60*GMT_OFFSET));

	Serial.print("Timestamp: ");
	Serial.print(uint32_t(reading.timestamp/reading.numSamples), DEC);
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
	Serial.println();

	Serial.print("Temp: ");
	Serial.print(reading.temperature/reading.numSamples);
	Serial.println("*C");
	Serial.print("Pressure: ");
	Serial.print(reading.pressure/reading.numSamples);
	Serial.println("hpa");
	Serial.print("Humidity: ");
	Serial.print(reading.humidity/reading.numSamples);
	Serial.println('%');
	Serial.print("Battery: ");
	Serial.print(reading.battery/reading.numBatterySamples);
	Serial.println("mV");
	Serial.print("Wind Speed: ");
	Serial.print(reading.windSpeed/reading.numSamples);
	Serial.println("mph");
	Serial.print("Wind Direction: ");
	Serial.print(reading.windDirection/reading.numSamples);
	Serial.println("deg");
	Serial.print("Num Samples: ");
	Serial.println(reading.numSamples);
	Serial.print("Num Battery Samples: ");
	Serial.println(reading.numBatterySamples);
}

void zeroWeatherReading(WeatherReading *reading) {
	reading->timestamp = 0;
	reading->temperature = 0;
	reading->humidity = 0;
	reading->pressure = 0;
	reading->battery = 0;
	reading->windSpeed = 0;
	reading->windDirection = 0;
}

void zeroWeatherReading(WeatherReadingAccumulator *reading) {
	reading->timestamp = 0;
	reading->temperature = 0;
	reading->humidity = 0;
	reading->pressure = 0;
	reading->battery = 0;
	reading->windSpeed = 0;
	reading->windDirection = 0;
	reading->numBatterySamples = 0;
	reading->numSamples = 0;
}

void copyWeatherReading(WeatherReading src, WeatherReading dest) {
	dest.timestamp = src.timestamp;
	dest.temperature = src.temperature;
	dest.humidity = src.humidity;
	dest.pressure = src.pressure;
	dest.battery = src.battery;
	dest.windSpeed = src.windSpeed;
	dest.windDirection = src.windDirection;
}

void store_accumulator(WeatherReading *dest, WeatherReadingAccumulator src) {
	dest->timestamp = src.timestamp/src.numSamples;
	dest->temperature = src.temperature/src.numSamples;
	dest->humidity = src.humidity/src.numSamples;
	dest->pressure = src.pressure/src.numSamples;
	dest->battery = src.battery/src.numBatterySamples;
	dest->windSpeed = src.windSpeed/src.numSamples;
	dest->windDirection = src.windDirection/src.numSamples;
}

WeatherReading get_averaged_accumulator(WeatherReadingAccumulator src) {
	WeatherReading dest;

	dest.timestamp = src.timestamp/src.numSamples;
	dest.temperature = src.temperature/src.numSamples;
	dest.humidity = src.humidity/src.numSamples;
	dest.pressure = src.pressure/src.numSamples;
	dest.battery = src.battery/src.numBatterySamples;
	dest.windSpeed = src.windSpeed/src.numSamples;
	dest.windDirection = src.windDirection/src.numSamples;

	return dest;
}

void waitMs(int delayMs) {
  delay(delayMs);
  return;

	int startTime = millis();

	while (millis()-startTime < delayMs)
	{
		delay(1);
		//ESP.wdtFeed();
	}
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


String generate_request_url(WeatherReading weatherReading) {
	// Make the url
	String outputUrl = "/report.php?";

  if (!isnan(weatherReading.temperature)) {
    outputUrl += "temp=";
    outputUrl += weatherReading.temperature;
  }

  if (!isnan(weatherReading.humidity)) {
      outputUrl += "&humidity=";
      outputUrl += weatherReading.humidity;

      // If we have temperature and humidity then calculate and submit the heat index also
      if (!isnan(weatherReading.temperature)) {
        outputUrl += "&heatIndex=";
        outputUrl += computeHeatIndex(weatherReading.temperature, weatherReading.humidity, false);
      }
  }

  if (!isnan(weatherReading.pressure)) {
    outputUrl += "&pressure=";
    outputUrl += weatherReading.pressure;
  }

  outputUrl += "&bat=";
  outputUrl += weatherReading.battery;
  outputUrl += "&windSpeed=";
  outputUrl += weatherReading.windSpeed;
  outputUrl += "&windDirection=";
  outputUrl += weatherReading.windDirection;

  outputUrl += "&timestamp=";
  outputUrl += weatherReading.timestamp;

  // Secret key for security (>_O)
  outputUrl += "&key=f6f9b0b8348a85843e951723a3060719f55985fd"; // frie!ggandham!!%2{[ sha1sum

	return outputUrl;
}

#endif
