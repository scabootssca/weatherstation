import sqlite3
import time

CHART_SCALE = 60*60*12
MAX_RESULTS = 200

sqlConnection = sqlite3.connect("weather.db")
sqlCursor = sqlConnection.cursor()


def test_query():
	resultsInTimeframe = sqlCursor.execute("SELECT COUNT(rowid), MIN(time), MAX(rowid), MIN(rowid) FROM weather WHERE time > ?", (time.time()-CHART_SCALE, )).fetchall()

	if not resultsInTimeframe:
		return []

	print(resultsInTimeframe)

	count, minTime, maxRowid, minRowid = resultsInTimeframe[0]
	step = (maxRowid-minRowid)//MAX_RESULTS

	if step == 0:
		step = 1

	rowIds = [x for x in range(minRowid, maxRowid) if not x % step]

	result = sqlCursor.execute("SELECT time FROM weather WHERE rowid IN (%s)" % ", ".join("?"*len(rowIds)), rowIds)

	#print(rowIds)

	#print(step)

	# sqlCursor.execute("""
	# 	SELECT time FROM weather JOIN (
	# 		SELECT rowid
	# 		WHERE time > ?", (time.time()-CHART_SCALE, )).fetchall()
	# 	""")


test_query()
