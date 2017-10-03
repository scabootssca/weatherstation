import socket
import select
import threading
import time

CLIENT_BUFFER_SIZE = 4096
HTTP_VERSION = "1.1"
PROGNAME = "crapshoot"


def date_time_string(timestamp=None):
	"""Return the current date and time formatted for a message header."""
	if timestamp is None:
		timestamp = time.time()
	weekdayname = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']

	monthname = [None, 'Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec']
	year, month, day, hh, mm, ss, wd, y, z = time.gmtime(timestamp)
	s = "%s, %02d %3s %4d %02d:%02d:%02d GMT" % (weekdayname[wd], day, monthname[month], year, hh, mm, ss)
	return s


def atof(a):
	try:
		return float(a)
	except ValueError:
		return


class Client:
	def __init__(self, server, info):
		self.server = server
		self.socket = info[0]
		self.address, self.port = info[1]
		self.connected = True

	def handle(self):
		result = self.server.clientHandlerFunction(self)

		if result is not True:
			self.server.disconnect_client(self)

	def __send(self, data):
		if type(data) != bytes:
			data = bytes(data, "UTF-8")

		self.socket.send(data)

	def __send_header(self, key, value):
		self.__send("%s: %s\n" % (key, value))

	def respond(self, content, contentType='text/html'):
		contentLength = len(content)

		self.__send("HTTP/%s 200 OK\n" % HTTP_VERSION)
		self.__send_header('Server', PROGNAME)
		self.__send_header('Date', date_time_string())
		self.__send_header('Content-Type', contentType)
		self.__send_header('Content-Length', contentLength)
		self.__send_header('Last-Modified', date_time_string(0))
		self.__send("\n")
		self.__send(content)

	def recieve(self):
		rcvBuffer = ""

		while self.connected:
			availableForReading, _, _ = select.select([self.socket], [], [], 0.5)

			if self.socket in availableForReading:
				chunk = self.socket.recv(CLIENT_BUFFER_SIZE).decode("UTF-8")
				rcvBuffer += chunk
			else:
				break

		return rcvBuffer


class Server:
	def __init__(self, port=8080):
		self.port = port

		self.running = False
		self.pollInterval = 0.5

		self.serverSocket = None
		self.ip = None

		self.pollHandlerFunction = None
		self.clientHandlerFunction = None

		self.clients = []

	def disconnect_client(self, client):
		client.connected = False
		client.socket.close()

		try:
			self.clients.remove(client)
		except ValueError:
			pass

	def __serve_forever(self):
		self.running = True
		self.serverSocket.listen(5)

		while self.running:
			availableForReading, _, _ = select.select([self.serverSocket], [], [], self.pollInterval)

			if not self.running:
				break

			if self.serverSocket in availableForReading:
				client = Client(self, self.serverSocket.accept())
				self.clients.append(client)

				#print("[%s] Client: %s" % (time.strftime("%d/%m/%Y %H:%M:%S"), client.address))

				clientThread = threading.Thread(target=client.handle)
				clientThread.start()

			# For providing a function to execute upon each pollw
			if self.pollHandlerFunction:
				self.pollHandlerFunction()

	def start(self):
		self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.serverSocket.bind(('', self.port))

		self.ip = socket.getaddrinfo(socket.gethostname(), self.port, 0, 0, socket.SOL_TCP)[-1][-1][0]

		print("Server Running on %s:%s" % (self.ip, self.port))

		self.__serve_forever()

	def stop(self):
		if self.running:
			self.running = False
			self.serverSocket.close()
