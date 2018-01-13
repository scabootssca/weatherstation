struct SampleAccumulator {
	uint32_t timestamp;
	double temperature = 0;
	double humidity = 0;
	double pressure = 0;
	double battery = 0;
	double windSpeed = 0;
	double windDirection = 0;

	int numBatterySamples = 0;

	bool populated = false;
};

#define WeatherReadingAccumulator SampleAccumulator
