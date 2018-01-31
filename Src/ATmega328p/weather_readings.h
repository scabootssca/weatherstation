#ifndef _WEATHER_READINGS_H_
#define _WEATHER_READINGS_H_

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
  //outputUrl += "&key=f6f9b0b8348a85843e951723a3060719f55985fd"; // frie!ggandham!!%2{[ sha1sum

	return outputUrl;
}

#endif
