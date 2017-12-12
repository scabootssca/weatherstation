struct WeatherReading {
	uint32_t timestamp;
	float temperature = NAN;
	float humidity = NAN;
	float pressure;
	float battery;
	float windSpeed;
	float windDirection;

  bool populated = false;
};

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
