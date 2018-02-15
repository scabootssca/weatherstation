<?php
$dbConn = new mysqli("localhost", "weather_station", "", 'WeatherStation');

// Check connection
if ($dbConn->connect_errno) {
	printf("Connect failed: %s\n", $dbConn->connect_error);
	exit;
}

$message = $dbConn->real_escape_string($_SERVER['QUERY_STRING']);

if ($message) {
	// If success
	$query = sprintf("INSERT INTO `debug` (`comment`) VALUES ('%s')", $message);

	if ($dbConn->query($query)) {
		printf("Success\n%s", time());
	} else {
		printf("Failure\n");
		printf($dbConn->error);
	}
} else {
	$query = sprintf("SELECT `id`, `time`, `comment` FROM `debug` ORDER BY time DESC");
	$queryResult = $dbConn->query($query);

	if ($queryResult) {
		printf("<h2>Debug Results: %s</h2>", $queryResult->num_rows);
		printf("<table><tr><td>Id</td><td>Time</td><td>Comment</td></tr>");


		for ($i=0; $i < $queryResult->num_rows; $i++) {
			$row = $queryResult->fetch_row();

			printf("<tr><td>%s</td><td>%s</td><td>%s</td></tr>", $row[0], $row[1], $row[2]);
		}

		printf("</table>");
	} else {
		printf("Failure\n");
		printf($dbConn->error);
	}
}

printf("\n");
$dbConn->close();
