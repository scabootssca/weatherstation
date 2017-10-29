<?php
define('CHART_MAX_SECONDS', 2*60*60); // Default 2 hours
define('CHART_DIVISION_BASE', 15*60); // Default 15 minutes
define('CHART_MAX_DIVISIONS', 10); // Max 10 divisions
define('CHART_MAX_DATAPOINTS', 200); // Max 50 datapoints in each chart
define('MOVING_MEAN_SAMPLE_SIZE', 5); // Samples each before and after; samples*2 for total num of samples
define('DISPLAY_AVERAGED_SAMPLES', False);

define('CHART_DATA', 1<<0);
define('CHART_MEAN', 1<<1);
define('CHART_ALL', CHART_DATA|CHART_MEAN);

global $chartAttributes;
$chartAttributes = array(
	2=>array("Temperature", "*F", function ($x) { return round(($x*1.8)+32, 2); }, 'spline', CHART_DATA),
	3=>array("Humidity", "%", function ($x) { return round($x, 2); }, 'spline', CHART_DATA),
	4=>array("Heat Index", "*F", function ($x) { return round(($x*1.8)+32, 2); }, 'spline', CHART_DATA),
	5=>array("Pressure", "mb", function ($x) { return round($x*.01, 2); }, 'spline', CHART_DATA),
	6=>array("Battery", "mV", function ($x) { return round($x, 1); }, 'line', CHART_DATA),
	7=>array("Wind Speed", "rpm", function ($x) { return round($x, 1); }, 'spline', CHART_ALL),
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
					suffix: "%s"
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
			$formattedChartData = sprintf('
				{
					type: "%s",
					lineThickness: %s,
					markerSize: 0,
					connectNullData: true,
					color: "#222F2F",
					dataPoints: [
					%s
					]
				}',
				$chartVars[3],
				2,
				$this->generate_data_points($queryResults, $chartVars[2])
			);
		}

		// $chartVars[4] == true for movingMean false for actual data
		$movingMeanData = '';

		if ($chartVars[4] & CHART_MEAN) {
			$movingMeanData = sprintf('%s
				{
					type: "%s",
					lineThickness: %s,
					markerSize: 0,
					connectNullData: true,
					color: "#77AABB",
					dataPoints: [
					%s
					]
				}',
				($chartVars[4] & CHART_DATA ? ',' : ''),
				$chartVars[3],
				3,
				$this->generate_moving_mean($queryResults, $chartVars[2], MOVING_MEAN_SAMPLE_SIZE)
			);
		}

		$output = sprintf(
			$output,
			$this->index,
			$this->index,
			$chartVars[0],
			ceil($interval),
			$spacing,
			$chartVars[1],
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

			// If we're too close to the edges then use as far as we can
			$numLowSamples = ($i < $sampleSize ? $i : $sampleSize);
			$numHighSamples = ($i > $numResults-$sampleSize ? $numResults-$i-1 : $sampleSize);

			// For each in the current index($i) - $sampleSize
			// Until the index + $sampleSize
			for ($j = $i - $numLowSamples; $j < $i + $numHighSamples; $j++) {
				$testPoint = $queryResults[$j][$this->index];

				// Make sure it's not null data
				if ($testPoint != NULL) {
					$total += $testPoint;
				}
			}

			// Get the mean and save the point
			$movingMean = $total / ($numLowSamples + $numHighSamples);

			if ($modifierFunction != NULL) {
				$output[] = sprintf("{ x: new Date(%s), y: %s }", $queryResults[$i][1]*1000, $modifierFunction($movingMean));
			} else {
				$output[] = sprintf("{ x: new Date(%s), y: %s }", $queryResults[$i][1]*1000, $movingMean);
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

function generate_chart_javascript($queryResult, $chartIndices) {
	global $chartAttributes;
	$result = $queryResult->fetch_all();

	print('<script type="text/javascript">
		window.onload = function() {');

	foreach ($chartIndices as $chartIndex) {
		$chart = new ChartDefinition($chartIndex);
		print($chart->generate_javascript($chartAttributes[$chartIndex], $result));
	}

	print("\n		}\n		</script>\n");

}

function generate_chart_html($chartIndices) {
	foreach ($chartIndices as $chartIndex) {
		printf("<div id=\"chartContainer%s\" class=\"chartContainer\"></div><br />\n", $chartIndex);
	}
}

function generate_chart_header($dbConn) {
	// Get the most recent entry for the current stats
	$query = "SELECT temperature, humidity, heatIndex, pressure, batteryMv, time, windSpeed FROM records ORDER BY time DESC LIMIT 1";

	printf('<div class="block, center"><h1>%s</h1>', strftime("%A, %B %d"));
	printf('<h2 style="margin-top: -1em;">%s</h2>', strftime("%H:%M"));

	 // If the query was successful
	 if ($queryResult = $dbConn->query($query)) {
	 		$currentInfo = $queryResult->fetch_row();

			printf(
				'<h4 style="margin-top: -1em;">Currently: %s&deg;F - Humidity: %s%% - Pressure: %smb - Wind: %srpm</h4>',
				round(($currentInfo[0]*1.8)+32, 2),
				$currentInfo[1],
				round($currentInfo[3]*.01, 2),
				$currentInfo[6]
			);

			$voltage = $currentInfo[4];

			if ($voltage < 3350) {
				$batteryPercent = "Disconnected";
			} elseif ($voltage <= 3700) {
				$batteryPercent = "0%";
			} elseif ($voltage >= 4100) {
				$batteryPercent = "100%";
			} else {
				$batteryPercent = sprintf("%s%%", intval(($voltage - 3700) * 100 / (4100-3700)));
			}

			printf(
				'<h4 style="margin-top: -1em;">Battery %smV %s</h4>',
				round($currentInfo[4], 1),
				$batteryPercent
			);
	}

	print("</div>\n");
}

function show_no_results($error="") {
	printf('</head><body><div class="block, center""><br /><br /><br /><br /><br /><br /><br />
	<h1 style="font-weight: bold; color: red;">No Readings Have Been Logged<br />%s</h1></div>',$error);
}

function queryDatabase() {
	$dbConn = new mysqli("localhost", "weather_station", "", 'WeatherStation');

	// Check connection
	if ($dbConn->connect_errno) {
		printf("Connect failed: %s\n", $dbConn->connect_error);
		exit;
	}

	if (isset($_GET['h'])) {
		$maxSecondsBack = $_GET['h']*60*60;

		if ($maxSecondsBack/60/60 > 120) {
			$maxSecondsBack = 48*60*60;
		}
	} else {
		$maxSecondsBack = CHART_MAX_SECONDS;
	}

	// Get the count of records that fall within the range and use that to figure every n row to select
	$query = sprintf(
		"SELECT COUNT(id) FROM records WHERE time > (UNIX_TIMESTAMP() - %s)",
		$dbConn->real_escape_string($maxSecondsBack)
	);

	if ($queryResult = $dbConn->query($query)) {
		$recordModulo = ceil($queryResult->fetch_row()[0]/CHART_MAX_DATAPOINTS);
	} else {
		$recordModulo = 1;
	}

	if ($recordModulo != 1 && DISPLAY_AVERAGED_SAMPLES) {
			$query = sprintf("SELECT
					records.id, records.time,
					AVG(nearColumns.temperature) temperature,
					AVG(nearColumns.humidity) humidity,
					AVG(nearColumns.heatIndex) heatIndex,
					AVG(nearColumns.pressure) pressure,
					AVG(nearColumns.batteryMv) batteryMv,
					AVG(nearColumns.windSpeed) windSpeed
			FROM records
				JOIN records nearColumns ON nearColumns.id > records.id-%s AND nearColumns.id < records.id+%s
			WHERE records.time > (UNIX_TIMESTAMP() - %s)
			AND records.id MOD %s = 0
			GROUP BY records.id
			ORDER BY records.time DESC
			",
			$dbConn->real_escape_string($recordModulo),
			$dbConn->real_escape_string($recordModulo),
			$dbConn->real_escape_string($maxSecondsBack),
			$dbConn->real_escape_string($recordModulo)
		);
	} else {
		$query = sprintf("SELECT id, time, temperature, humidity, heatIndex, pressure, batteryMv, windSpeed FROM records WHERE time > (UNIX_TIMESTAMP() - %s) AND id MOD %s = 0 ORDER BY time DESC",
			$dbConn->real_escape_string($maxSecondsBack),
			$dbConn->real_escape_string($recordModulo)
		);
	}

	$queryResult = $dbConn->query($query);
	return array($dbConn, $queryResult);
}

function generatePage() {
	require("header.php");
	output_header();

	list($dbConn, $queryResult) = queryDatabase();

	// If the query was successful
	if ($queryResult) {
		// If we have results
		if ($queryResult->num_rows > 0) {
			// Generate the js for all the charts
			global $chartAttributes;
			$chartIndices = array_keys($chartAttributes);//array(2, 3, 4, 5, 7, 6, 8);
			generate_chart_javascript($queryResult, $chartIndices);

			// This is an alpha for auto updating the charts
			// Will make this something else later? php file to auto generate new csv for values?
			// Then this can fetch and update the values with jquery
			printf("<script src=\"./update.js\"></script>\n");
			printf("</head><body>\n");

			printf("<div id=\"slider\">
      <div class=\"handle\" style=\"left: 382.508px;\">
        <div class=\"line\"></div>
      </div>
    </div>\n");

			//printf("<button onclick=\"updateData()\">Update</button>");

			// Generate the html to go with them
			generate_chart_header($dbConn);
			generate_chart_html($chartIndices);

		// No results
		} else {
			show_no_results();
		}
	// Unsucessful query
	} else {
		show_no_results("Unsucessful Query");
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
?>
