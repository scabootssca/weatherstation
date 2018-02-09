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
	uint32_t rain = 0;
};

struct WeatherReadingAccumulator {
	uint32_t timestamp;
	int64_t temperature = 0;
	uint64_t humidity = 0;
	uint64_t pressure = 0;
	uint64_t battery = 0;
	uint64_t windSpeed = 0;
	uint64_t windDirection = 0;
	uint64_t rain = 0;

	uint32_t numSamples = 0;
	uint32_t numBatterySamples = 0;
};

void zeroWeatherReading(WeatherReading *reading) {
	reading->timestamp = 0;
	reading->temperature = 0;
	reading->humidity = 0;
	reading->pressure = 0;
	reading->battery = 0;
	reading->windSpeed = 0;
	reading->windDirection = 0;
	reading->rain = 0;
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
	reading->rain = 0;
}

void copyWeatherReading(WeatherReading src, WeatherReading dest) {
	dest.timestamp = src.timestamp;
	dest.temperature = src.temperature;
	dest.humidity = src.humidity;
	dest.pressure = src.pressure;
	dest.battery = src.battery;
	dest.windSpeed = src.windSpeed;
	dest.windDirection = src.windDirection;
	dest.rain = src.rain;
}

void store_accumulator(WeatherReading *dest, WeatherReadingAccumulator src) {
	dest->timestamp = src.timestamp;
	dest->temperature = (src.temperature/float(src.numSamples))*.01;
	dest->humidity = (src.humidity/float(src.numSamples))*.01;
	dest->pressure = (src.pressure/float(src.numSamples))*.01;
	dest->battery = (src.battery/float(src.numBatterySamples))*.01;
	dest->windSpeed = (src.windSpeed/float(src.numSamples))*.01;
	dest->windDirection = src.windDirection/float(src.numSamples);
	dest->rain = src.rain/float(src.numSamples);
}

WeatherReading get_averaged_accumulator(WeatherReadingAccumulator src) {
	WeatherReading dest;
	store_accumulator(&dest, src);

	return dest;
}

void printWeatherReading(WeatherReading reading) {
  // For displaying them in local time
	//DateTime readingTimeLocal = DateTime(DateTime(reading.timestamp) + TimeSpan(60*60*GMT_OFFSET));

	Serial.print("Timestamp: ");
	print_pretty_timestamp(reading.timestamp);//readingTimeLocal.unixtime());
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
	Serial.print("Rain: ");
	Serial.println(reading.rain);
}

void printWeatherReading(WeatherReadingAccumulator sampleAccumulator) {
  printWeatherReading(get_averaged_accumulator(sampleAccumulator));

	Serial.print("Num Samples: ");
	Serial.println(sampleAccumulator.numSamples);
	Serial.print("Num Battery Samples: ");
	Serial.println(sampleAccumulator.numBatterySamples);
}

#endif
