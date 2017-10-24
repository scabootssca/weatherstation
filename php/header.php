<?php
function output_header() {
	print('<!DOCTYPE HTML>
	<html>
	<head>
		<style type="text/css">
		.canvasjs-chart-credit {
		display: none;
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
			width: 90%;
			margin-left: auto;
			margin-right: auto;
			height: 160px;
			display: block;
		}
		</style>
		<title>Historical Weather Data</title>
		<meta http-equiv="refresh" content="60" >
		<script src="./jquery-3.2.1.min.js"></script>
		<script src="./canvasjs.min.js"></script>
		');
} ?>
