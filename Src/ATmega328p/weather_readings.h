#ifndef _WEATHER_READINGS_H_
#define _WEATHER_READINGS_H_
#include <math.h>

struct WeatherReading {
	uint32_t timestamp;
	float temperature = 0;
	float humidity = 0;
	float pressure = 0;
	float batteryMv = 0;
	float windGust = 0;
	float windSpeed = 0;
	float windDirection = 0;
	float lux = 0;
	uint32_t rain = 0;
};

struct WeatherReadingAccumulator {
	uint32_t timestamp;

	// Bme sample things
	int32_t temperature = 0;
	uint32_t humidity = 0;
	uint64_t pressure = 0;

	uint32_t batteryMv = 0;
	uint32_t windGust = 0;
	uint32_t windSpeed = 0;
	float windDirectionX = 0;
	float windDirectionY = 0;
	uint32_t rain = 0;

	uint64_t lux = 0;

	uint8_t numSamples = 0;
	uint8_t numBatterySamples = 0;
};

void zeroWeatherReading(WeatherReading *reading) {
	reading->timestamp = 0;
	reading->temperature = 0;
	reading->humidity = 0;
	reading->pressure = 0;
	reading->batteryMv = 0;
	reading->windGust = 0;
	reading->windSpeed = 0;
	reading->windDirection = 0;
	reading->rain = 0;
	reading->lux = 0;
}

void zeroWeatherReading(WeatherReadingAccumulator *reading) {
	reading->timestamp = 0;
	reading->temperature = 0;
	reading->humidity = 0;
	reading->pressure = 0;
	reading->batteryMv = 0;
	reading->windGust = 0;
	reading->windSpeed = 0;
	reading->windDirectionX = 0;
	reading->windDirectionY = 0;
	reading->numBatterySamples = 0;
	reading->numSamples = 0;
	reading->rain = 0;
	reading->lux = 0;
}

void store_accumulator(WeatherReading *dest, WeatherReadingAccumulator src) {
	dest->timestamp = src.timestamp;
	dest->temperature = (src.temperature/float(src.numSamples))*.01;
	dest->humidity = (src.humidity/float(src.numSamples))*.01;
	dest->pressure = (src.pressure/float(src.numSamples))*.01;
	dest->batteryMv = (src.batteryMv/float(src.numBatterySamples))*.01;

	dest->windGust = float(src.windGust)*.01; // This is direct except for making a float
	dest->windSpeed = (src.windSpeed/float(src.numSamples))*.01;

	// Do the average of the X,Y cartesian wind coordinates
	// Convert that to polar and you have your degrees and the variance
	// Need that because avg of the degrees for getting a circular mean
	// eg. (359+0)/2 == ~180 is totally wrong says south not north
	float avgWindX = src.windDirectionX/float(src.numSamples);
	float avgWindY = src.windDirectionY/float(src.numSamples);

	// Serial.print(F("Avg (X, Y) ("));
	// Serial.print(avgWindX);
	// Serial.print(", ");
	// Serial.print(avgWindY);
	// Serial.println(")");

	// Atan+modifier is in radians, then convert to degrees
	float angleRadians = atan(avgWindY/avgWindX);
	float angleDegrees = angleRadians * 180 / M_PI;

	if (avgWindX < 0) {
		// Quadrant 2
		if (avgWindY >= 0) {
			angleDegrees = 180-angleDegrees;
		// Quadrant 3
		} else {
			angleDegrees = 180+angleDegrees;
		}
	// Quadrant 4
	} else if (avgWindY < 0) {
		angleDegrees = 360+angleDegrees;
	}

	dest->windDirection = angleDegrees;
	// float variance = 1-(sqrt((avgWindX*avgWindX) + (avgWindY*avgWindY))); // Magnitude of polar vector
  //
	// Serial.print(F("Variance: "));
	// Serial.println(variance);
	// Serial.print(F("AvgRadians: "));
	// Serial.println(angleRadians);
	// Serial.print(F("Avg Wind Degrees: "));
	// Serial.println(dest->windDirection);

	dest->rain = src.rain/float(src.numSamples);
	dest->lux = src.lux/float(src.numSamples);
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
	Serial.print(reading.batteryMv);
	Serial.println("mV");
	Serial.print("Wind Gust: ");
	Serial.println(reading.windGust);
	Serial.print("Wind Speed: ");
	Serial.print(reading.windSpeed);
	Serial.println("mph");
	Serial.print("Wind Direction: ");
	Serial.print(reading.windDirection);
	Serial.println("deg");
	Serial.print("Rain: ");
	Serial.println(reading.rain);
	Serial.print("Lux: ");
	Serial.println(reading.lux);
}

void printWeatherReading(WeatherReadingAccumulator sampleAccumulator) {
  printWeatherReading(get_averaged_accumulator(sampleAccumulator));

	Serial.print("Num Samples: ");
	Serial.println(sampleAccumulator.numSamples);
	Serial.print("Num Battery Samples: ");
	Serial.println(sampleAccumulator.numBatterySamples);
}

#endif
