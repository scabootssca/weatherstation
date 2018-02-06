var dataPoints = [[], [], [], [], [], []];

// function handlePoint(key, value) {
// 	dataPoints.push({x: value[0], y: parseInt(value[1])});
// }

function handleData(data) {
	//$.each(data, handlePoint(key, value))
	let values = data.split(",");
	let dataTime = 0;

	for (var i = 0; i < values.length; i++) {
			var columnIndex = i%6;

			if (columnIndex == 0) {
				dataTime = values[i];
			} else {
				dataPoints[columnIndex-1].push({x: new Date(parseInt(dataTime)), y: parseFloat(values[i])});
			}
	}

	for (var i = 2; i<7; i++) {
		window["chart"+i].options.data[0].dataPoints = dataPoints[i-2];
		window["chart"+i].render();
	}
}

function h2(data) {
	let groups = data.trim().split("\n");

	for (var groupIndex=0; groupIndex<groups.length; groupIndex++) {
		groupValues = groups[groupIndex].split(",");

		if (groupIndex == 0) {
			dataPoints[0] = groupValues.map(function(x) {
				return new Date(parseInt(x));
			});
		} else {
			dataPoints[groupIndex] = groupValues.map(function(value, index) {
				return {x: dataPoints[0][index], y: parseFloat(value)}
			});

			window["chart"+(groupIndex+1)].options.data[0].dataPoints = dataPoints[groupIndex];
			window["chart"+(groupIndex+1)].render();
		}
	}
}

function updateData() {
	//jQuery.getJSON("./infocsv.php", handleData);
	jQuery.get("./infocsv.php?h=.5", h2);


	//chart.data.datasets[0].data[index] = value;
	//chart.update();
}
