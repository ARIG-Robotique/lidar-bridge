import socket
import json
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# LiDAR connection settings
LIDAR_IP = 'localhost'  # Replace with your LiDAR's IP
LIDAR_PORT = 8686           # Replace with your LiDAR's port
REQUEST_JSON = {
	"action": "GRAB_DATA"   # Replace with the correct request your LiDAR expects
}

def connect_to_lidar(ip, port):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((ip, port))
	return s

def get_lidar_data(sock):
	# Send JSON request
	request_str = json.dumps(REQUEST_JSON) + '\n'
	request_str = ''.join([line.strip() for line in request_str.splitlines()])
	
	sock.sendall(request_str.encode())

	# Receive response
	data = b""
	while not data.endswith(b'\n'):
		chunk = sock.recv(4096)
		if not chunk:
			break
		data += chunk

	try:
		response = json.loads(data.decode())
		return response
	except json.JSONDecodeError:
		print("Invalid JSON response")
		return []

def polar_to_cartesian(angle_deg, distance):
	angle_rad = math.radians(angle_deg)
	x = distance * math.cos(angle_rad)
	y = distance * math.sin(angle_rad)
	return x, y

# Connect to LiDAR once
sock = connect_to_lidar(LIDAR_IP, LIDAR_PORT)

points_last = list()

def update():
	global points_last
	object = get_lidar_data(sock)
	#print(object)
	points = [polar_to_cartesian(obj['angleDeg'], obj['distanceMm']) for obj in object["data"]["scan"]]
	for i1, p1 in enumerate(points):
		for i2, p2 in enumerate(points_last):
			if p1 == p2:
				print(f"Point at {i1} is the same as point at {i2}")
	points_last = points
	xs, ys = zip(*points)

	plt.clf()
	plt.scatter(xs, ys)
	plt.pause(.1)

while 1:
	update()

plt.show()

# Close connection when done
sock.close()
