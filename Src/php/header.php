<?php
function output_header($pageTitle="Historical Weather Data", $headHtml='') {
	$chartHeight = 250;

	if (isset($_GET['height'])) {
		$chartHeight = intVal($_GET['height']);
	}

	if ($chartHeight < 100) {
		$chartHeight = 100;
	} else if ($chartHeight > 1000) {
		$chartHeight = 1000;
	}

	printf('
<!DOCTYPE HTML>
<html>
<head>
	<style type="text/css">
	body {
		background-color: #FBFBFB;
	}

	.center {
		margin-left: auto;
		margin-right: auto;
		text-align: center;
	}

	.block {
		display: block;
	}

	.chartContainer {
		width: 90%%;
		margin-left: auto;
		margin-right: auto;
		height: %spx;
		display: block;
	}

	#slider {
		display: none;
		position: absolute;
		top: 160px;
		left: 0;
		width: 100%%;
		height: 100%%;
		z-index: 9;
	}

	#slider .handle {
		background-color: rgba(255, 50, 0, 0.04);
		height: 100%%;
		margin-left: -15px;
		width: 32px;
		position: absolute;
		z-index: 9;
	}

	#slider .handle .line {
		background-color: rgba(255, 0, 0, 0.5);
		height: 100%%;
		margin: 0 auto;
		width: 1px;
	}

	.header {
		height: 120px;
		text-align: center;
	}

	#windLink {
		display: inline;
		float: right;

		position: relative;
		right: 5%%;
	}

	</style>
	<title>%s</title>
	<meta http-equiv="refresh" content="300" >
	<script src="./jquery-3.2.1.min.js"></script>
	<script src="./canvasjs.min.js"></script>
	<script src="./slider.js"></script>
	',
	$chartHeight,
	$pageTitle
	);
} ?>
