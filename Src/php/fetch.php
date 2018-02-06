<?php
define('CHART_DIVISION_BASE', 15*60); // Default 15 minutes
define('CHART_MAX_DIVISIONS', 10); // Max 10 divisions
define('CHART_MAX_DATAPOINTS', 400); // Max 50 datapoints in each chart
define('DISPLAY_AVERAGED_SAMPLES', False);

define('CHART_DATA', 1<<0);
define('CHART_MEAN', 1<<1);
define('CHART_ALL', CHART_DATA|CHART_MEAN);

function get_data_timeframe() {
	$maxTime = time();
	$minTime = $maxTime-43200; // Default time is 12 hours in seconds (43200 seconds)
	$fromNow = true;

	if (isset($_GET['range']) && $_GET['range']) {
		$exploded = explode(',', $_GET['range'], 2); // Split from,to

		// If we have a from,to range
		if (sizeof($exploded) > 1) {
			$minTime = floatval($exploded[0]);
			$maxTime = floatval($exploded[1]);
			$fromNow = false;
		// Only 1 term
		} else {
			$minTime = $maxTime-floatval($exploded[0])*60*60;
		}
	} else if (isset($_GET['minDate']) && isset($_GET['maxDate'])) {
		$parsedTime = strtotime($_GET['minDate']);

		if (is_int($parsedTime)) {
			$minTime = $parsedTime;
		}

		$parsedTime = strtotime($_GET['maxDate']);

		if (is_int($parsedTime)) {
			// If it's before minTime then set it to minTime
			if ($parsedTime < $minTime) {
				$parsedTime = $minTime;
			}

			$maxTime = $parsedTime+86400; // So it's the end of the day
			$fromNow = false;
		}
	}

	return array($minTime, $maxTime, $fromNow);
}

$timeframeInfo = get_data_timeframe();
define('CHART_MIN_TIMESTAMP', $timeframeInfo[0]);
define('CHART_MAX_TIMESTAMP', $timeframeInfo[1]);
define('CHART_RANGE_IS_UNTIL_CURRENT', $timeframeInfo[2]);

if (isset($_GET['s'])) {
	$smoothness = intval($_GET['s']);
} else {
	$smoothness = 1;
}

define('MOVING_MEAN_SAMPLE_SIZE', $smoothness); // Samples each before and after; samples*2 for total num of samples

define('WIND_DIR_INT', 0);
define('WIND_DIR_STEP', 1);
define('WIND_DIR_TEXT', 2);
define('WIND_DIR_TEXT_SHORT', 3);

function get_wind_dir($windDeg, $type=WIND_DIR_TEXT) {//$getInt=false, $getShort=false) {
	$directionSuccinctNames = array('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW');

	$directionVerboseNames = array(
		'North',
		'North North East',
		'North East',
		'East North East',
		'East',
		'East South East',
		'South East',
		'South South East',
		'South',
		'South South West',
		'South West',
		'West South West',
		'West',
		'West North West',
		'North West',
		'North North West'
	);

	$steps = 16.0;
	$stepSize = 360/$steps;

	$textWindDir = $directionVerboseNames[0];
	$step = 0;

	for ($step = 0; $step < $steps; $step++) {
		if ($windDeg < ($stepSize*0.5 + $stepSize*$step)) {
			$textWindDir = ($type == WIND_DIR_TEXT_SHORT) ? $directionSuccinctNames[$step] : $directionVerboseNames[$step];
			break;
		}
	}

	// Cause north and we'd be over the last < then upper range so it'd ++ after
	if ($step == 16) {
		$step = 0;
	}

	if ($type == WIND_DIR_INT) {
		return intval($windDeg);
	}

	if ($type == WIND_DIR_STEP) {
		return $step;
	}

	return $textWindDir;
}

global $chartAttributes;
$chartAttributes = array(
	2=>array("Temperature", "°F", function ($x) { return round(($x*1.8)+32, 2); }, 'spline', CHART_ALL, 6, 0, ''),
	3=>array("Humidity", "%", function ($x) { return round($x, 2); }, 'spline', CHART_DATA|CHART_MEAN, 24, 0, ''),
	5=>array("Pressure", "mb", function ($x) { return round($x*.01, 2); }, 'spline', CHART_ALL, 0, 0, ''),
	7=>array("Wind Speed", "mph", function ($x) { return round($x, 1); }, 'spline', CHART_ALL, 12, 0, ''),
	8=>array("Wind Direction", '', function ($x) { return get_wind_dir($x, WIND_DIR_INT); }, 'spline', CHART_DATA|CHART_MEAN, 6, 0, ""),//"interval: 22,\n					labelFormatter: function (e) { return ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW', 'N'][e.value%15]; }"),
	6=>array("Battery", "mV", function ($x) { return round($x, 1); }, 'line', CHART_DATA|CHART_MEAN, 18, 0, ''),
);

Class ChartDefinition {
	public $index;

	function __construct($index) {
			$this->index = $index;
	}

	function generate_javascript($chartVars, &$queryResults) {
		$output = '
			var chart%s = new CanvasJS.Chart("chartContainer%s", {
				theme: "theme2",
				backgroundColor: "#FBFBFB",
				title: {
					text: "%s"
				},
				animationEnabled: false,

				toolTip: {
				},

				axisX: {
					interval: %s,
					intervalType: "%s",
					valueFormatString: "MM/DD HH:mm",
				},
				axisY: {
					includeZero: false,
					suffix: "%s",
					margin: %s,
					%s
				},
				data: [
				%s%s
				]
			});

			window["chart%s"] = chart%s;
			chart%s.render();
			';

		// Split the date stamps into the approprite size and count of intervals
		$secondRange = time()-end($queryResults)[1];

		// This is all in seconds
		if ($secondRange/CHART_DIVISION_BASE/CHART_MAX_DIVISIONS < 1) {
			$interval = intval($secondRange/CHART_MAX_DIVISIONS);
		} else {
			$interval = (ceil($secondRange/CHART_DIVISION_BASE/CHART_MAX_DIVISIONS)*CHART_DIVISION_BASE);
		}


		// Convert to minutes or hours
		// Less than an hour do minutes
		if ($interval < 3600) {
			$spacing = 'minute';
			$interval /= 60;
		// More than an hour
		} else {
			$spacing = 'hour';
			$interval = ceil($secondRange/CHART_MAX_DIVISIONS/60/60);
		}

		$formattedChartData = '';
		if ($chartVars[4] & CHART_DATA) {
			if ($chartVars[4] & CHART_MEAN && MOVING_MEAN_SAMPLE_SIZE) {
				// Light blue if there is also a mean
				$color = '9ec1c1';
			} else {
				$color = '222F2F';
			}

			$formattedChartData = sprintf('
				{
					type: "%s",
					lineThickness: %s,
					markerSize: %s ,
					connectNullData: true,
					color: "#%s",
					xValueFormatString: "MM/DD/YY HH:mm",
					dataPoints: [
					%s
					]
				}',
				$chartVars[3],
				($chartVars[4] & CHART_MEAN && MOVING_MEAN_SAMPLE_SIZE) ? 1 : 2,
				$chartVars[6],
				$color,
				$this->generate_data_points($queryResults, $chartVars[2])
			);
		}

		// $chartVars[4] == true for movingMean false for actual data
		$movingMeanData = '';

		if ($chartVars[4] & CHART_MEAN && MOVING_MEAN_SAMPLE_SIZE) {
			$meanChartType = $chartVars[3];

			if ($meanChartType == 'line') {
				$meanChartType = 'spline';
			}

			$movingMeanData = sprintf('%s
				{
					type: "%s",
					lineThickness: %s,
					markerSize: 0,
					connectNullData: true,
					color: "#222F2F",
					xValueFormatString: "MM/DD/YY HH:mm",
					dataPoints: [
					%s
					]
				}',
				($chartVars[4] & CHART_DATA ? ',' : ''),
				$meanChartType,
				2,
				$this->generate_moving_mean($queryResults, $chartVars[2], MOVING_MEAN_SAMPLE_SIZE)
			);
		}

		// $axisYFormat = $chartVars[7];
    //
		// if ($this->index == 8) {
		// 	$axisYFormat = 'labelFormatter: function (e) { return ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"][e.value]; }';
		// }

		$output = sprintf(
			$output,
			$this->index,
			$this->index,
			$chartVars[0],
			ceil($interval),
			$spacing,
			$chartVars[1],
			$chartVars[5],
			$chartVars[7], // Axis Y Format
			$formattedChartData,
			$movingMeanData,
			$this->index,
			$this->index,
			$this->index
		);

		return $output;
	}

	private function generate_moving_mean(&$queryResults, $modifierFunction, $sampleSize=5) {
		$output = array();

		// Starting at $sampleSize so we can get info before it
		// Then we go until length - $sampleSize so we can get info after
		$numResults = sizeof($queryResults);
		for ($i = 0; $i < $numResults; $i++) {
			$total = 0;
			$testPoint = 0;
			$numTestPoints = 0;

			// If we're too close to the edges then use as far as we can
			$numLowSamples = ($i < $sampleSize ? $i : $sampleSize);
			$numHighSamples = ($i >= $numResults-$sampleSize ? $numResults-$i-1 : $sampleSize);

			//printf("\n%s - Starting at %s going until \$j <= %s\n", $i, $i - $numLowSamples, $i + $numHighSamples);

			// For each in the current index($i) - $sampleSize
			// Until the index + $sampleSize
			for ($j = $i - $numLowSamples; $j <= $i + $numHighSamples; $j++) {
				$testPoint = $queryResults[$j][$this->index];

				// Make sure it's not null data
				if ($testPoint != NULL) {
					$total += $testPoint;
					$numTestPoints += 1;
				}
			}

			if ($numTestPoints == 0) {
				$output[] = sprintf("{ x: new Date(%s), y: null }", $queryResults[$i][1]*1000);
			} else {
				if (($numLowSamples + $numHighSamples) == 0) {
					$movingMean = $total;
				} else {
					// Get the mean and save the point
					$movingMean = round($total / $numTestPoints, 2);
				}

				if ($modifierFunction != NULL) {
					$output[] = sprintf("{ x: new Date(%s), y: %s }", $queryResults[$i][1]*1000, $modifierFunction($movingMean));
				} else {
					$output[] = sprintf("{ x: new Date(%s), y: %s }", $queryResults[$i][1]*1000, $movingMean);
				}
			}
		}

		return join(",\n					", $output);
	}

	private function generate_data_points(&$queryResults, $modifierFunction) {
		$output = array();

		// Does unixtime*1000 cause Date in js dates milliseconds
		if ($modifierFunction != NULL) {
			foreach ($queryResults as $row) {
				if (gettype($row[$this->index]) == "NULL") {
					$output[] = sprintf("{ x: new Date(%s), y: null }", $row[1]*1000);
				} else {
					$output[] = sprintf("{ x: new Date(%s), y: %s }", $row[1]*1000, $modifierFunction($row[$this->index]));
				}
			}
		} else {
			foreach ($queryResults as $row) {
				if (gettype($row[$this->index]) == "NULL") {
					$output[] = sprintf("{ x: new Date(%s), y: null }", $row[1]*1000);
				} else {
					$output[] = sprintf("{ x: new Date(%s), y: %s }", $row[1]*1000, $row[$this->index]);
				}
			}
		}

		return join(",\n					", $output);
	}
}

function generate_chart_javascript($queryResults, $chartIndices) {
	global $chartAttributes;

	print('<script type="text/javascript">
		window.onload = function() {');

	foreach ($chartIndices as $chartIndex) {
		$chart = new ChartDefinition($chartIndex);
		print($chart->generate_javascript($chartAttributes[$chartIndex], $queryResults));
	}

	print("\n		}\n		</script>\n");

}

function generate_chart_html($chartIndices) {
	foreach ($chartIndices as $chartIndex) {
		printf("<div id=\"chartContainer%s\" class=\"chartContainer\"></div><br />\n", $chartIndex);
	}
}

function get_wind_mph($windMph, $round=false) {
	if ($round != false && intval($round)) {
		return round($windMph, intval($round));
	} else {
		return $windMph;
	}
}

function get_temp_extremes($dbConn) {
	$query = sprintf("SELECT
			MAX(temperature), MIN(temperature)
		FROM records WHERE time >= %s AND time <= %s
		ORDER BY time DESC
		",
		$dbConn->real_escape_string(CHART_MIN_TIMESTAMP),
		$dbConn->real_escape_string(CHART_MAX_TIMESTAMP)
	);

	$queryResult = $dbConn->query($query);

	if ($queryResult && $queryResult->num_rows) {
		return $queryResult->fetch_row();
	}

	return array(NONE, NONE);
}

function query_current_stats($existingDbConn=NULL, $maxAge=0, $batSamples=5) {
	// Get the connection
	if ($existingDbConn == NULL) {
		require('sqlUserCfg.php');
		$dbConn = new mysqli("localhost", $sqlUser, $sqlPass, $sqlDb);

	  // Check connection
	  if ($dbConn->connect_errno) {
	 	 printf("Connect failed: %s\n", $dbConn->connect_error);
	 	 exit;
	  }
	} else {
		$dbConn = $existingDbConn;
	}

	// Get the high and low
	list($highTemp, $lowTemp) = get_temp_extremes($dbConn);

	// Get the most recent entry for the current stats
	$query = sprintf("SELECT
			records.temperature,
			records.humidity,
			records.heatIndex,
			records.pressure,
			records.batteryMv,
			records.time,
			records.windSpeed,
			records.windDirection,
			records.id,
			records.batteryPercent
		FROM records
		WHERE records.time > (UNIX_TIMESTAMP()-3600)
		GROUP BY records.id
		ORDER BY records.time DESC LIMIT 1
		",
	);

	if ($queryResult = $dbConn->query($query)) {
		 $currentInfo = $queryResult->fetch_row();
		 array_push($currentInfo, $highTemp);
		 array_push($currentInfo, $lowTemp);
		 return $currentInfo;
	 }

	 return NULL;
}

function get_wind_chill($tempF, $windSpeed, $digits=-1) {
	// Windchill temperature is defined only for temperatures at or below 10 °C (50 °F) and wind speeds above 4.8 kilometres per hour (3.0 mph)
	// http://www.nws.noaa.gov/om/cold/wind_chill.shtml
	if ($windSpeed > 3) {
		$windChill = 35.74 + (0.6215 * $tempF) - (35.75 * ($windSpeed ** 0.16)) + (0.4275 * $tempF * ($windSpeed ** 0.16));
	} else {
		$windChill = $tempF;
	}

	if ($digits == -1) {
		return $windChill;
	}

	return round($windChill, $digits);
}

function c_to_f($temp, $round=2) {
	return round(($temp*1.8)+32, $round);
}

function get_apparent_temp($tempF, $windSpeed, $heatIndex, $digits=-1) {
	/* Also This
	https://www.ncdc.noaa.gov/societal-impacts/apparent-temp/at

	AT = -2.7 + 1.04*T + 2.0*e -0.65*v
		where AT and T (air temperature) are °C,
		e is vapor pressure in kPa,
		and v is 10m wind speed in m/sec.
	*/


	// Less than or equal to 50 use wind chill
	if ($tempF <= 50) {
		return get_wind_chill($tempF, $windSpeed, $digits);
	// More than 80 use heat index
	} elseif ($tempF > 80) {
		return $heatIndex;
	}

	// 51 to 80 use air temp
	return $tempF;
}

function generate_chart_header($currentInfo) {
	printf('<div class="block, center, header"><h2>%s</h2>', strftime("Currently %H:%M on %A, %B %d"));
	//printf('<h3 style="margin-top: -1em;">%s</h3>', strftime("%H:%M"));

	// If the query was successful
	if ($currentInfo) {
		$windMph = get_wind_mph($currentInfo[6], 2);

		$outData = array();

		if ($currentInfo[0] != '') {
			$tempF = c_to_f($currentInfo[0]);
			array_push($outData, sprintf("Temp: %s&deg;F", $tempF));

			if ($currentInfo[2] != '') {
				$apparentTemp = get_apparent_temp($tempF, $windMph, round(($currentInfo[2]*1.8)+32, 2), 2);
				array_push($outData, sprintf("Feels Like: %s&deg;F", $apparentTemp));
			}
		}

		if ($currentInfo[1] != '') {
			array_push($outData, sprintf("Humidity: %s%%", $currentInfo[1]));
		}

		if ($currentInfo[3] != '') {
			array_push($outData, sprintf("Pressure: %smb", round($currentInfo[3]*.01, 2)));
		}

		printf('<h5 style="margin-top: -1em; line-height: 150%%;">%s<br/ >High: %s&deg;F - Low: %s&deg;F<br />Wind: %smph - %s<br />Battery: %smV</h5>',
			implode(' - ', $outData),
			c_to_f($currentInfo[10]),
			c_to_f($currentInfo[11]),
			$windMph,
			get_wind_dir($currentInfo[7]),
			round($currentInfo[4], 1)
		);
	} else {
		printf("No Recent Info");
	}

	print("</div>\n");
}

function generate_chart_footer($latestTimestamp=0) {
	printf('<br /><div class="block, center">');

	if ($latestTimestamp) {
		if (CHART_RANGE_IS_UNTIL_CURRENT) {
			printf('<h5>Showing last %s hours</h5>', round((CHART_MAX_TIMESTAMP-CHART_MIN_TIMESTAMP)/60/60, 1));
			printf('<h5 style="margin-top: -1em">Latest data from %s</h5>', strftime("%H:%M on %A, %B %d", $latestTimestamp));
		} else {
			printf('<h5>Showing %s -to- %s</h5>',
				strftime('%D %H:%M', CHART_MIN_TIMESTAMP),
				strftime('%D %H:%M', (CHART_MAX_TIMESTAMP > $latestTimestamp) ? $latestTimestamp : CHART_MAX_TIMESTAMP)
			);
		}
	} else {
		printf('<h5>Showing %s -to- %s</h5>',
			strftime('%D %H:%M', CHART_MIN_TIMESTAMP),
			strftime('%D %H:%M', CHART_MAX_TIMESTAMP)
		);
	}

	printf('<h6>Change data range:</h6><form style="margin-top: -1em;">From: <input type="date" name="minDate" max="%s"> To: <input type="date" name="maxDate" max="%s"><br /><br /><input type="submit" value="Update"></form><br />',
		strftime('%Y-%m-%d'),
		strftime('%Y-%m-%d')
	);

	print('</div>');
}

function show_no_results($error="") {
	printf('<div class="block, center">
	<h1 style="font-weight: bold; color: red; margin-top: 3em; margin-bottom: 3em;">No Readings in Selected Timeframe<br />%s</h1>
	</div>',$error);
}

function queryDatabase() {
	require('sqlUserCfg.php');
	$dbConn = new mysqli("localhost", $sqlUser, $sqlPass, $sqlDb);

	// Check connection
	if ($dbConn->connect_errno) {
		printf("Connect failed: %s\n", $dbConn->connect_error);
		exit;
	}

	// Get the count of records that fall within the range and use that to figure every n row to select
	$query = sprintf(
		"SELECT COUNT(id) FROM records WHERE time >= %s AND time <= %s",
		$dbConn->real_escape_string(CHART_MIN_TIMESTAMP),
		$dbConn->real_escape_string(CHART_MAX_TIMESTAMP)
	);

	$queryResult = $dbConn->query($query);

	if ($queryResult && $queryResult->num_rows) {
		$recordCount = $queryResult->fetch_row()[0];
		$recordModulo = ceil($recordCount/CHART_MAX_DATAPOINTS);
	} else {
		$recordModulo = 1;
	}

	if ($recordCount && DISPLAY_AVERAGED_SAMPLES) {
			$query = sprintf("SELECT
					records.id, records.time,
					AVG(nearColumns.temperature) temperature,
					AVG(nearColumns.humidity) humidity,
					AVG(nearColumns.heatIndex) heatIndex,
					AVG(nearColumns.pressure) pressure,
					AVG(nearColumns.batteryMv) batteryMv,
					AVG(nearColumns.windSpeed) windSpeed,
					AVG(nearColumns.windDirection) windDirection,
			FROM records
				JOIN records nearColumns ON nearColumns.id > records.id-%s AND nearColumns.id < records.id+%s
			WHERE records.time >= %s AND records.time <= %s
			AND records.id MOD %s = 0
			GROUP BY records.id
			ORDER BY records.time DESC
			",
			$dbConn->real_escape_string($recordModulo),
			$dbConn->real_escape_string($recordModulo),
			$dbConn->real_escape_string(CHART_MIN_TIMESTAMP),
			$dbConn->real_escape_string(CHART_MAX_TIMESTAMP),
			$dbConn->real_escape_string($recordModulo)
		);
	} else {
		$query = sprintf("SELECT
			id, time, temperature, humidity, heatIndex, pressure, batteryMv, windSpeed, windDirection
			FROM records WHERE time >= %s AND time <= %s
			AND id MOD %s = 0 ORDER BY time DESC
			",
			$dbConn->real_escape_string(CHART_MIN_TIMESTAMP),
			$dbConn->real_escape_string(CHART_MAX_TIMESTAMP),
			$dbConn->real_escape_string($recordModulo)
		);
	}

	$queryResult = $dbConn->query($query);
	return array($dbConn, $queryResult);
}

function generatePage() {
	require("header.php");
	output_header();

	list($dbConn, $query) = queryDatabase();

	$currentInfo = query_current_stats($dbConn);

	// If the query was successful
	if ($query) {
		// If we have results
		if ($query->num_rows > 0) {
			// Generate the js for all the charts
			global $chartAttributes;
			//array(2, 3, 4, 5, 7, 6, 8);
			$chartIndices = array_keys($chartAttributes);
			$queryResults = $query->fetch_all();

			generate_chart_javascript($queryResults, $chartIndices);

			// This is an alpha for auto updating the charts
			// Will make this something else later? php file to auto generate new csv for values?
			// Then this can fetch and update the values with jquery
			//printf("<script src=\"./update.js\"></script>\n");
			printf("</head><body>\n");

			printf('<div id="windLink">Wind Rose<br /><a href="./wind.php?range=%s,%s"><img style="width: 100px; height: 100px" src="./windRose.png"></img></a></div>%s', CHART_MIN_TIMESTAMP, CHART_MAX_TIMESTAMP, "\n");

			printf("<div id=\"slider\">
      <div class=\"handle\" style=\"left: 382.508px;\">
        <div class=\"line\"></div>
      </div>
    </div>\n");

			$currentInfo = query_current_stats($dbConn);

			$latestTimestamp = array_values(array_slice($queryResults, -1))[0];[5];

			if ($currentInfo) {
				$latestTimestamp = $currentInfo[5];
			}

			// Generate the html to go with them
			generate_chart_header($currentInfo);
			generate_chart_html($chartIndices);
			generate_chart_footer($latestTimestamp);

		// No results
		} else {
			print("</head><body>\n");
			printf('<div id="windLink">Wind Rose<br /><a href="./wind.php?range=%s,%s"><img style="width: 100px; height: 100px" src="./windRose.png"></img></a></div>%s', CHART_MIN_TIMESTAMP, CHART_MAX_TIMESTAMP, "\n");

			generate_chart_header($currentInfo);
			show_no_results();
			generate_chart_footer();
		}
	// Unsucessful query
	} else {
		print("</head><body>\n");
		printf('<div id="windLink">Wind Rose<br /><a href="./wind.php?range=%s,%s"><img style="width: 100px; height: 100px" src="./windRose.png"></img></a></div>%s', CHART_MIN_TIMESTAMP, CHART_MAX_TIMESTAMP, "\n");

		generate_chart_header($currentInfo);
		show_no_results("Unsucessful Query");
		generate_chart_footer();
	}

	print("<script src=\"init.js\"></script>");

	if (isset($_GET['l'])) {
		print("<style type=\"text/css\">#slider {
			display: block !important;
		}</style>");
	}

	print("</body></html>");

	$dbConn->close();
}

function generate_wind_table($dbConn=NULL) {
	$success = false;

	if ($dbConn == NULL) {
		require('sqlUserCfg.php');
		$dbConn = new mysqli("localhost", $sqlUser, $sqlPass, $sqlDb);
	}

 // Check connection
 if ($dbConn->connect_errno) {
	 printf("Connect failed: %s\n", $dbConn->connect_error);
	 exit;
 }

	$query = sprintf(
		//"SELECT windSpeed, windDirection FROM records WHERE time > (UNIX_TIMESTAMP() - %s) AND windSpeed > 0 ORDER BY time",
		"SELECT windSpeed, windDirection FROM records WHERE time > %s AND time < %s ORDER BY time",
		$dbConn->real_escape_string(CHART_MIN_TIMESTAMP),
		$dbConn->real_escape_string(CHART_MAX_TIMESTAMP)
	);

	$queryResult = $dbConn->query($query);

	$windTable = array();
	$speedSteps = array(1, 3, 5, 10, 15, 20, 25, 30, 35, 40);

	for ($i = 0; $i < 16; $i++) {
		array_push($windTable, array_fill(0, count($speedSteps), 0));
	}

	// If the query was successful
	if ($queryResult) {
		$totalResults = $queryResult->num_rows;

		if ($totalResults > 0) {
			printf('
			<div class="box">
			<div id="container"></div>
			</div>');

			printf("<br /><br />");

			$allResults = $queryResult->fetch_all();

			foreach ($allResults as $row) {
				$windDir = get_wind_dir($row[1], WIND_DIR_STEP);
				$windSpeed = get_wind_mph($row[0]);

				for ($speedRow = 0; $speedRow < count($speedSteps); $speedRow++) {
					if ($windSpeed < $speedSteps[$speedRow]) {
						break;
					}
				}

				//printf("%s [%s][%s]\n<br />", $row[1], $windDir, $speedRow);
				$windTable[$windDir][$speedRow] += 1;
			}

			printf('
<br /><br /><br />
<div id="chartDiv">
		<table id="localFreq" border="0" cellspacing="0" cellpadding="0" style="width: 100%%;">
				<tr nowrap bgcolor="#CCCCFF">
						<th colspan="%s" class="hdr">Table of Frequencies (percent)</th>
				</tr>
				<tr nowrap bgcolor="#CCCCFF">
						<th class="freq">Direction</th>',
				count($speedSteps)+2
			);

			for ($i = 0; $i < count($speedSteps); $i++) {
				$symbol = '';

				if ($i == 0) {
					$symbol = "&lt; ";
				} else if ($i == count($speedSteps)-1) {
					$symbol = "&gt; ";
				}

				printf(
					"<th class=\"freq\">%s%s MPH </th>\n",
					$symbol,
					$speedSteps[$i]
				);
			}

			printf("<th class=\"freq\">Total</th></tr>\n");

			$letterDirs = array("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW");

			$totalForSpeeds = array_fill(0, count($speedSteps), 0);

			for ($i = 0; $i < 16; $i++) {
				printf("<tr nowrap>\n<td class=\"dir\">%s</td>\n", $letterDirs[$i]);

				$rowTotal = 0;

				for ($speedIndex = 0; $speedIndex < count($windTable[$i]); $speedIndex++) {
					$rowTotal += $windTable[$i][$speedIndex];
					$totalForSpeeds[$speedIndex] += $windTable[$i][$speedIndex];
					printf("<td class=\"data\">%s</td>\n", round(($windTable[$i][$speedIndex] / $totalResults) * 100, 2));
				}

				printf("<td class=\"data\">%s</td>\n", round(($rowTotal / $totalResults) * 100, 2));

				printf("</tr>\n");
			}

			printf("<tr nowrap>\n<td class=\"totals\">Total</td>\n");

			for ($speedIndex = 0; $speedIndex < count($speedSteps); $speedIndex++) {
				printf("<td class=\"totals\">%s</td>\n", round(($totalForSpeeds[$speedIndex] / $totalResults) * 100, 2));
			}

			printf("</tr>\n");

			printf("</table></div>\n");

			$success = true;
		}
	}

	$dbConn->close();
	return $success;
}
?>
