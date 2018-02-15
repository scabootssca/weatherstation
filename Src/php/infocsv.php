<?php
require("./fetch.php");
list($dbConn, $queryResult) = queryDatabase();

function generate_data_points(&$queryResults, $indices=array(), $modifierFunction=NULL) {
	$output = array();

	// foreach ($queryResults->fetch_all() as $row) {
	// 	foreach ($indices as $columnIndex) {
	//
	// 	}
	//
	// 	}
	// }
	// Does unixtime*1000 cause Date in js dates milliseconds
	if ($modifierFunction != NULL) {
		foreach ($queryResults->fetch_all() as $row) {
			$output[] = $row[1]*1000;
			foreach ($indices as $index) {
				$output[] = $modifierFunction($row[$index]);
			}
		}
	} else {
		foreach ($queryResults->fetch_all() as $row) {
			$output[] = $row[1]*1000;
			foreach ($indices as $index) {
				$output[] = $row[$index];
			}
		}
	}

	return join(",", $output);
}

function g2(&$queryResults, $chartIndices) {
	global $chartAttributes;
	$arrayResults = $queryResults->fetch_all();

	// First the times
	printf("%s\n", join(",", array_column($arrayResults, 1)));

	// Then each chart value
	foreach ($chartIndices as $index) {
		printf(
			"%s\n",
			join(
				",",
				array_map($chartAttributes[$index][2], array_column($arrayResults, $index))
				//array_column($arrayResults, $index)
			)
		);
	}
}

// If the query was successful
if ($queryResult) {
	// If we have results
	if ($queryResult->num_rows > 0) {
		// Generate the js for all the charts
		$chartIndices = array(2, 3, 4, 5, 6);

		$result = g2($queryResult, $chartIndices);
		//print($result);
	}
}

$dbConn->close();
?>
