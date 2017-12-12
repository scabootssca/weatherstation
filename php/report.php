<?php
//ini_set('display_errors', 1);
//error_reporting(E_ALL | E_STRICT);

$dbConn = new mysqli("localhost", "weather_station", "", 'WeatherStation');

// Check connection
if ($dbConn->connect_errno) {
	printf("Connect failed: %s\n", $dbConn->connect_error);
	exit;
}

$valueMap = array(
	"temp"=>"temperature",
	"humidity"=>"humidity",
	"heatIndex"=>"heatIndex",
	"pressure"=>"pressure",
	"altitude"=>"altitude",
	"bat"=>"batteryMv",
	"windSpeed"=>"windSpeed"
);

$submitKeys = [];
$submitValues = [];
$hasSecretKey = false;

foreach ($_GET as $getKey => $getValue) {
	if ($getValue == "nan") {
		continue;
	}

	if ($getKey == "key" && $getValue == "frieggandham") {
		$hasSecretKey = true;
	}

	if (array_key_exists($getKey,  $valueMap)) {
		array_push($submitKeys, $dbConn->real_escape_string($valueMap[$getKey]));
		array_push($submitValues, $dbConn->real_escape_string($getValue));
	}
}

if ($hasSecretKey && count($submitKeys)) {
	$query = sprintf("INSERT INTO records (time, %s) VALUES (UNIX_TIMESTAMP(), %s)",
		join(", ", $submitKeys),
		join(", ", $submitValues)
	);

	if ($dbConn->query($query)) {
		printf("Success");
	} else {
		printf("Failure\n");
		printf($dbConn->error);
	}
} else {
	printf("Failure\n");
	printf("Wrong count or no secret key");
}

$dbConn->close();