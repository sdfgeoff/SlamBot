from collections import deque
import json
from typing import NamedTuple
from mathutils import Vector, Matrix, Euler, Quaternion
import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200


class ChassisSample(NamedTuple):
	gyro: Vector
	accelerometer: Vector
	time: int
	distance_left_mm: float
	distance_right_mm: float


class ChassisConnection:
	def __init__(self):
		self.serial = serial.Serial(port=PORT, baudrate=BAUD, timeout=0)
		self.serialBuffer: bytes = b''
		self.samples: deque[ChassisSample] = deque(maxlen=1000)

	def poll(self):
		self.serialBuffer += self.serial.read(1000000)
		while b'\r\n' in self.serialBuffer:
			before, after = self.serialBuffer.split(b'\r\n', maxsplit=1)
			self.serialBuffer = after
			try:
				raw_packet = json.loads(before)
				self.samples.append(ChassisSample(
					gyro=Vector((
						raw_packet['rx'],
						raw_packet['ry'],
						raw_packet['rz'],
					)),
					accelerometer=Vector((
						raw_packet['ax'],
						raw_packet['ay'],
						raw_packet['az'],
					)),
					time=raw_packet['t'],
					distance_left_mm=raw_packet['dl'],
					distance_right_mm=raw_packet['dr']
				))
			except Exception as e:
				print("Json Decode Error:", e)
