<?php
require('fetch.php');

// temperature, humidity, heatIndex, pressure, batteryMv, time, windSpeed, windDirection
$currentWeather = query_current_stats();

if (isset($_GET['seafoam'])) {
  // For csv
  if (!isset($_GET['human'])) {
    if ($currentWeather) {
      printf(join(',', $currentWeather));
    }
  // For human readable current info string
  } else {
    if ($currentWeather) {
      $temperature = $currentWeather[0] * (9/5) + 32;

      $output = sprintf(
        "%s°F %s%% - %s @ %smph",
        round($temperature, 1),
        round($currentWeather[1], 1),
        get_wind_dir($currentWeather[7], WIND_DIR_TEXT_SHORT),
        get_wind_mph($currentWeather[6], 2)
      );
    } else {
      $output = 'No recent readings';
    }

    printf("<div style=\"text-align: center;\"><a href=\"http://192.168.1.160/\" target=\"_blank\" style=\"color: #666666; text-decoration: none;\">%s</a></div>", $output);
  }
}
