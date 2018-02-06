<?php
//ini_set('display_errors', 1);
//error_reporting(E_ALL | E_STRICT);

require('sqlUserCfg.php');
$dbConn = new mysqli("localhost", $sqlUser, $sqlPass, $sqlDb);

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
	"windSpeed"=>"windSpeed",
	"windDirection"=>'windDirection',
	"timestamp"=>"time",
	"batPercent"=>"batteryPercent"
);

$submitKeys = [];
$submitValues = [];
$hasSecretKey = false;
$valid = true;

// Format the keys correctly
foreach ($_GET as $getKey => $getValue) {
	if ($getValue == "nan") {
		continue;
	}

	if ($getKey == "key" && $getValue == sha1('frie!ggandham!!%2{[')) {
		$hasSecretKey = true;
		continue;
	}

	$submitTime = time();

	// If it's in the future (>10 minute) (We get that sometimes) then don't accept it
	if ($getKey == "timestamp" && $getValue > $submitTime+600) {
		$valid = false;
		break;
	}

	if (array_key_exists($getKey,  $valueMap)) {
		array_push($submitKeys, $dbConn->real_escape_string($valueMap[$getKey]));
		array_push($submitValues, $dbConn->real_escape_string($getValue));
	}
}

// If success
if ($valid && $hasSecretKey && count($submitKeys)) {
	$query = sprintf("INSERT INTO records (%s) VALUES (%s)",
		join(", ", $submitKeys),
		join(", ", $submitValues)
	);

	if ($dbConn->query($query)) {
		printf("Success\n%s", time());
	} else {
		printf("Failure\n");
		printf($dbConn->error);
	}
} else {
	printf("Failure\n");
	printf("Forbidden.\n");
	if (!$valid) {
		printf("Future timestamp > 1min: %s > %s\n", $getValue, $submitTime);
	}

	if (!$hasSecretKey) {
		printf("Invalid auth");
	}
}

printf("\n");
$dbConn->close();
