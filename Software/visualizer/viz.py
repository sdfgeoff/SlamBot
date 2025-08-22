from collections import deque
from typing import NamedTuple
import bge
from mathutils import Vector, Matrix, Euler, Quaternion
import serial
import json
import math

PORT = '/dev/ttyUSB0'
BAUD = 115200

ACCELEROMETER_ANGLE_GAIN = 1.0


GRAVITY = Vector([0,0,9.8])

class ChassisSample(NamedTuple):
	gyro: Vector
	accelerometer: Vector
	time: int
	distance_left_mm: float
	distance_right_mm: float




class LidarVizualizer:
	def __init__(self, obj):
		self.obj = obj
		self.distance_mm = 0.0

	def update(self):
		bge.render.drawLine(
			self.obj.worldPosition,
			self.obj.worldPosition + self.obj.getAxisVect([0,0,self.distance_mm/1000]),
			[1,1,0,1]
		)




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

def get_children_by_property(obj, property):
	return [c for c in obj.childrenRecursive if property in c]


class Simulator:
	def __init__(self, obj):
		self.obj = obj
		self.chassis = ChassisConnection()
		self.lidar_left = LidarVizualizer(get_children_by_property(self.obj, 'LidarLeft')[0])
		self.lidar_right = LidarVizualizer(get_children_by_property(self.obj, 'LidarRight')[0])

		self.gyro_bias = Vector([0,0,0])
		self.accelerometer_bias = Vector([0,0,0])

		self.angular_velocity_estimate = Vector([0,0,0])
		self.previous_orientation = self.obj.worldOrientation
		self.linear_velocity_worldspace = Vector([0,0,0])

		self.i = 0
		

	def run(self):
		dt = 1 / bge.logic.getAverageFrameRate()
		self.chassis.poll()
		if self.chassis.samples:
			self.i += 1

			latest_packet = self.chassis.samples[-1]
			self.lidar_left.distance_mm = latest_packet.distance_left_mm
			self.lidar_right.distance_mm = latest_packet.distance_right_mm
			self.lidar_left.update()
			self.lidar_right.update()

			angular_velocity_local =  latest_packet.gyro - self.gyro_bias
			angular_velocity_world = Euler(angular_velocity_local, 'XYZ').to_quaternion()
			angular_velocity_world.angle *= dt
			self.obj.worldOrientation = self.obj.worldOrientation @ angular_velocity_world.to_matrix()

			gravity_vector_measured = self.obj.worldOrientation @ (latest_packet.accelerometer - self.accelerometer_bias).normalized()
			gravity_vector_from_pose_estimate = GRAVITY.normalized()

			axis = gravity_vector_measured.cross(gravity_vector_from_pose_estimate)
			axis = self.obj.worldOrientation.inverted() @ axis 
			if self.i < 10:
				gain = 1
			else:
				gain = dt * ACCELEROMETER_ANGLE_GAIN
			corrective_quat = Quaternion(axis.normalized(), math.asin(axis.length * gain))

			self.obj.worldOrientation = self.obj.worldOrientation @ corrective_quat.to_matrix()

			acceleration = self.obj.worldOrientation @ (latest_packet.accelerometer - self.accelerometer_bias) - GRAVITY
			if self.i < 60:
				#self.accelerometer_bias = latest_packet.accelerometer - self.obj.worldOrientation.inverted() @ GRAVITY
			#    print(latest_packet.accelerometer, self.obj.worldOrientation.inverted() @ GRAVITY)

				pass
			else:
				self.linear_velocity_worldspace += acceleration * dt
				#self.obj.worldPosition += self.linear_velocity_worldspace * dt

			print('bias', self.accelerometer_bias)


def start():
	cont = bge.logic.getCurrentController()
	obj = cont.owner
	if "sim" not in obj:
		obj["sim"] = Simulator(obj)
	else:
		obj["sim"].run()
	
