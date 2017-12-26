#include "RTClib.h"

struct WeatherReading {
	uint32_t timestamp;
	float temperature = 0;
	float humidity = 0;
	float pressure = 0;
	float battery = 0;
	float windSpeed = 0;
	float windDirection = 0;

  bool populated = false;
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

void printWeatherReading(WeatherReading reading) {
	// For displaying them in local time
	DateTime readingTimeLocal = DateTime(DateTime(reading.timestamp) + TimeSpan(60*60*GMT_OFFSET));

	Serial.print("Timestamp: ");
	Serial.print(reading.timestamp, DEC);
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

void zeroWeatherReading(WeatherReading *reading) {
	reading->timestamp = 0;
	reading->temperature = 0;
	reading->humidity = 0;
	reading->pressure = 0;
	reading->battery = 0;
	reading->windSpeed = 0;
	reading->windDirection = 0;
  reading->populated = false;
}

void copyWeatherReading(WeatherReading src, WeatherReading dest) {
	dest.timestamp = src.timestamp;
	dest.temperature = src.temperature;
	dest.humidity = src.humidity;
	dest.pressure = src.pressure;
	dest.battery = src.battery;
	dest.windSpeed = src.windSpeed;
	dest.windDirection = src.windDirection;
	dest.populated = src.populated;
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
