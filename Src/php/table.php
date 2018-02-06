<?php
require('fetch.php');

list($dbConn, $queryResult) = queryDatabase();

// If the query was successful
if ($queryResult) {
  // If we have results
  if ($queryResult->num_rows > 0) {
    printf("<table style=\"width: 100%%\">");

    printf("<tr style=\"font-weight: bold;\"><td>Id</td><td>Timestamp</td><td>Temp</td><td>Humidity</td><td>Heat Index</td><td>Barometer</td><td>BatV</td><td>Wind Speed</td><td>Wind Dir</td></tr>");

    $queryResults = $queryResult->fetch_all();
    $numResults = sizeof($queryResults);

    $bgColors = array('#EfEfEf', '#FFFFFF');

		for ($i = 0; $i < $numResults; $i++) {
      printf("<tr style=\"background-color: %s\">", $bgColors[$i%2]);

      $columnIndex = 0;
      foreach ($queryResults[$i] as $data) {
        if ($columnIndex == 1) {
          $data = strftime('%D @ %H:%M:%S', intVal($data));
        }

        printf("<td>%s</td>", $data);
        $columnIndex++;
      }

      printf("</tr>");
    }
  }

  printf("</table>");
}
?>
