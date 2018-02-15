<?php
require("./fetch.php");
require("./header.php");
output_header('Local Wind Rose');
?>
<script src="https://code.highcharts.com/highcharts.js"></script>
<script src="https://code.highcharts.com/highcharts-more.js"></script>
<script src="https://code.highcharts.com/modules/data.js"></script>
<script src="https://code.highcharts.com/modules/exporting.js"></script>

<style type="text/css">
.box{
	position: relative;
	width: 50%;		/* desired width */
	margin: 0 auto;
	max-width: 1000px;
}
.box:before{
	content: "";
	display: block;
	padding-top: 100%; 	/* initial ratio of 1:1*/
}

#container{
	position:  absolute;
	top: 0;
	left: 0;
	bottom: 0;
	right: 0;
}

#chartDiv {
	margin: 0 auto;
	width: 50%;
	max-width: 1000px;
}
</style>

</head>
<body>

<?php
$dbConn = new mysqli("localhost", "weather_station", "", 'WeatherStation');

$currentInfo = query_current_stats($dbConn);

printf('<div id="windLink">Back Home<br /><a href="./index.php?range=%s,%s"><img style="width: 100px; height: 100px" src="./historicalCharts.png"></img></a></div>%s', CHART_MIN_TIMESTAMP, CHART_MAX_TIMESTAMP, "\n");
generate_chart_header($currentInfo);

$success = generate_wind_table($dbConn);

if (!$success) {
	show_no_results("");
}

generate_chart_footer($currentInfo);
?>

<script>
// Parse the data from an inline table using the Highcharts Data plugin
Highcharts.chart('container', {
    data: {
        table: 'localFreq',
        startRow: 1,
        endRow: 17,
        endColumn: 10
    },

    chart: {
        polar: true,
        type: 'column'
    },

    title: {
        text: 'Wind Rose for Home'
    },

    subtitle: {
        text: "<?php printf(strftime("%H:%M")); ?>"
    },

    pane: {
        size: '100%'
    },

    legend: {
        align: 'right',
        verticalAlign: 'top',
        y: 100,
        layout: 'vertical'
    },

    xAxis: {
        tickmarkPlacement: 'on'
    },

    yAxis: {
        min: 0,
        endOnTick: false,
        showLastLabel: true,
        title: {
            text: 'Frequency (%)'
        },
        labels: {
            formatter: function () {
                return this.value + '%';
            }
        },
        reversedStacks: false
    },

    tooltip: {
        valueSuffix: '%'
    },

    plotOptions: {
        series: {
            stacking: 'normal',
            shadow: false,
            groupPadding: 0,
            pointPlacement: 'on'
        }
    }
});
</script>

</body>
</html>
