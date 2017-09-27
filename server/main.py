import os
import time
import sqlite3
import queue
import server as serverModule

resultsHeader = """<!DOCTYPE HTML>
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
	</style>
	<title>%s</title>
	<meta http-equiv="refresh" content="60" >
	<script type="text/javascript">
		window.onload = function () {
""" % ("Historical Weather Readings")

resultsMid = """
	}
	</script>
	<script src="./canvasjs.min.js"></script>
</head>

<body>"""

resultsFooter = """
</body>

</html>
"""

DATAPOINTS = 50  # Max results to return
LINE_THICKNESS = 4

AXISX_STEP = 15  # X axis intervals step
AXISX_STEP_TYPE = "minute"  # X axis interval type

CHART_SCALE = 60*60*2  # How long ago to display; last 6, 12, 24, whatever hours or minutes expressed as seconds


class SqlHandler:
	def __init__(self):
		self.queryLog = []
		self.queue = queue.Queue()

		if not os.path.exists("weather.db"):
			self.sqlConnection = sqlite3.connect("weather.db")
			self.sqlCursor = self.sqlConnection.cursor()
			self.sqlCursor.execute("CREATE TABLE weather (time INTEGER, temperature REAL, pressure REAL, altitude REAL, humidity REAL, heatIndex REAL)")
			self.sqlConnection.commit()
		else:
			self.sqlConnection = sqlite3.connect("weather.db")
			self.sqlCursor = self.sqlConnection.cursor()

	def push(self, *values):
		self.queue.put(values)

	def flush(self):
		task = None

		while True:
			try:
				task = self.queue.get(False)
			except queue.Empty:
				break

			# print("Handle Query: %s" % ", ".join((str(x) for x in task)))
			self.sqlCursor.execute("INSERT INTO weather VALUES (?, ?, ?, ?, ?, ?)", task)
			self.queue.task_done()

		if task is not None:
			self.sqlConnection.commit()

	def query(self, query, *args):
		result = self.sqlCursor.execute(query, args)
		return result.fetchall()


def clientHandler(self):
	request = self.recieve()

	splitRequest = request.split(" ", 2)

	if len(splitRequest) > 1:
		requestType = splitRequest[0]
		requestPath = splitRequest[1]
	else:
		self.server.disconnect_client(self)
		return

	print("[%s] %s %s" % (time.strftime("%d/%m/%Y %H:%M:%S"), self.address, requestPath))

	if requestType == "GET":
		if requestPath == "/canvasjs.min.js":
			with open("./canvasjs.min.js", "r") as f:
				content = f.read()

			self.respond(content, "text/javascript")

		elif "?" in requestPath:
			argDict = {}
			for arg in requestPath.split("?", 1)[1].split(","):
				if "=" in arg:
					split = arg.split("=", 1)
					argDict[str(split[0])] = split[1]

			self.server.sqlHandler.push(
				int(time.time()),
				argDict['temp'],
				argDict['pressure'],
				argDict['altitude'],
				argDict['humidity'],
				argDict['heatIndex']
				)

			self.respond("Accepted:\n%s" % "\n".join(argDict.keys()), "text/plain")

		else:
			self.server.replyQueue.put(self)
			return True


class ChartDefinition:
	def __init__(self, index, title, axisX, axisY, queryResult, queryResultFunc=None):
		self.index = index
		self.title = title
		self.axisX = axisX
		self.axisY = axisY
		self.queryResult = [(i[index], i[5]) for i in queryResult]
		self.queryResultFunc = queryResultFunc


def generateChartHtml(chart):
	data = ""

	# Take the data and make it into the chart datapoint format stored in `data`
	for index, item in enumerate(chart.queryResult):
		if chart.queryResultFunc is not None:
			data += "{ x: new Date(%s), y: %s }," % (int(item[1]) * 1000, chart.queryResultFunc(item[0]))
		else:
			data += "{ x: new Date(%s), y: %s }," % (int(item[1]) * 1000, item[0])

	def format_axis_items(axis):
		output = []

		for k, v in axis.items():
			if type(v) is bool:
				v = "true" if v else "false"
			elif type(v) is str:
				v = '"%s"' % v

			output.append("%s: %s" % (k, v))

		return output

	output = '''
			var chart = new CanvasJS.Chart("chartContainer%s", {
				theme: "theme2",
				title: {
					text: "%s"
				},
				animationEnabled: false,

				axisX: {
					interval: %s,
					intervalType: "%s",
					valueFormatString: "MM/DD HH:mm",
					%s

				},
				axisY: {
					%s

				},
				data: [{
					type: "line",
					lineThickness: %s,
					dataPoints: [
					%s
					]
				}
				]
			});

			chart.render();\n\n''' % (
				chart.index,
				chart.title,
				AXISX_STEP,
				AXISX_STEP_TYPE,
				",\n					".join(format_axis_items(chart.axisX)),
				",\n					".join(format_axis_items(chart.axisY)),
				LINE_THICKNESS,
				data
			)

	return output


if __name__ == "__main__":
	sqlHandler = SqlHandler()
	server = serverModule.Server()
	server.replyQueue = queue.Queue()

	chartInfo = (
			(
				"Temperature",
				{},
				{
					"includeZero": False,
					"suffix": "*F"
				},
				lambda x: round((x*1.8)+32, 2)
			),
			(
				"Pressure",
				{},
				{
					"includeZero": False,
					"suffix": " mb"
				},
				lambda x: round(x*.01, 2),
			),
			(
				"Altitude",
				{},
				{
					"includeZero": False,
					"suffix": "Ft"
				},
				lambda x: round(x*3.28084, 2)
			),
			(
				"Humidity",
				{},
				{
					"includeZero": False,
					"suffix": "%"
				},
				None
			),
			(
				"Heat Index",
				{},
				{
					"includeZero": False,
					"suffix": "&deg;F"
				},
				lambda x: round((x*1.8)+32, 2),
			)
		)

	def query_result():
		resultsInTimeframe = sqlHandler.query("SELECT COUNT(rowid), MIN(time), MAX(rowid), MIN(rowid) FROM weather WHERE time > ?", time.time()-CHART_SCALE)

		if not resultsInTimeframe:
			return []

		count, minTime, maxRowid, minRowid = resultsInTimeframe[0]
		step = (maxRowid-minRowid)//DATAPOINTS

		# If DATAPOINTS is greater than the available data then just display all using step = 1
		if step == 0:
			step = 1

		rowIds = [x for x in range(minRowid, maxRowid) if not x % step]

		query = "SELECT temperature, pressure, altitude, humidity, heatIndex, time FROM weather WHERE rowid IN (%s) ORDER BY time DESC"
		return sqlHandler.query(query % ", ".join("?"*len(rowIds)), *rowIds)

		#return sqlHandler.query("SELECT temperature, pressure, altitude, humidity, heatIndex, time FROM weather WHERE ROWID % ? = 0 AND time > ? ORDER BY time DESC LIMIT ?", manualStep, beginTime, DATAPOINTS)

	def handle_poll():
		sqlHandler.flush()

		while True:
			try:
				client = server.replyQueue.get(False)
			except queue.Empty:
				break

			queryResult = query_result()

			data = resultsHeader

			for index in (0, 1, 3, 4):
				chart = ChartDefinition(index, chartInfo[index][0], chartInfo[index][1], chartInfo[index][2], queryResult, chartInfo[index][3])
				data += generateChartHtml(chart)

			data += resultsMid

			current = sqlHandler.query("SELECT temperature, pressure, humidity, heatIndex FROM weather ORDER BY rowid DESC LIMIT 1")[0]
			currentInfoHtml = '<h2 style="margin-top: -1em;">%s</h2><h4 style="margin-top: -1em;">Currently: %s&deg;F - Humidity: %s%% - Pressure: %smb</h4>' % (time.strftime("%H:%M"), round((current[0]*1.8)+32, 2), current[2], round(current[1]*.01, 2))

			# Add the date and time header
			data += '<div class="block, center"><h1>%s</h1>%s</div>' % (time.strftime("%A, %B %d"), currentInfoHtml)



			data += ''
			print(current)

			# If we had no results then state so
			if not queryResult:
				data += '<div class="block, center""><br /><br /><br /><br /><br /><br /><br /><h1 style="font-weight: bold; color: red;">No Readings Have Been Logged</h1></div>'

			# Else show the data
			else:
				for index in (0, 1, 3, 4):
					data += '<div id="chartContainer%s" style="width: 90%%; margin-left: auto; margin-right: auto; height: 160px;display: block;"></div><br />\n' % index

			data += resultsFooter

			client.respond(data, "text/html")
			client.server.disconnect_client(client)

			server.replyQueue.task_done()

	server.sqlHandler = sqlHandler
	server.clientHandlerFunction = clientHandler
	server.pollHandlerFunction = handle_poll
	server.start()
	server.stop()
