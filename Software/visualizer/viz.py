from collections import deque
from typing import NamedTuple
import bge
from chassis import ChassisCommand, ChassisConnection, ChassisSample
from imu import GRAVITY
from mathutils import Vector, Matrix, Euler, Quaternion
import math
import itertools

from missions.calibration_missions import CalibrateGyro
from util import get_deviation_and_average



ACCELEROMETER_ANGLE_GAIN = 1.0



class Pose(NamedTuple):
    worldTransform: Matrix
    worldLinearVelocity: Vector


class LidarVizualizer:
    def __init__(self, obj):
        self.obj = obj
        self.distance_mm = 0.0

        self.cones = []
        self.i = 0

    def update(self):
        bge.render.drawLine(
            self.obj.worldPosition,
            self.obj.worldPosition + self.obj.getAxisVect([0,0,self.distance_mm/1000]),
            [1,1,0,1]
        )

        self.i += 1

        if len(self.cones) < 100:
            cone = self.obj.scene.addObject("Cone")
            self.cones.append(cone)
        else:
            cone = self.cones[self.i % len(self.cones)]
        cone.worldOrientation = self.obj.worldOrientation
        cone.worldPosition = self.obj.worldPosition

        rise = math.tan(math.radians(25/2)) * self.distance_mm / 1000
        cone.localScale = [rise, rise, self.distance_mm / 1000]





def get_children_by_property(obj, property):
    return [c for c in obj.childrenRecursive if property in c]



def estimate_new_pose_from_imu(previous_pose: Pose, sample: ChassisSample, gyro_bias: Vector, accelerometer_bias: Vector, dt: float) -> Pose:
    ori = previous_pose.worldTransform.to_3x3()
    translation = previous_pose.worldTransform.translation.copy()
    linear_velocity = previous_pose.worldLinearVelocity.copy()

    angular_velocity_local =  sample.gyro - gyro_bias
    angular_velocity_world = Euler(angular_velocity_local, 'XYZ').to_quaternion()
    angular_velocity_world.angle *= dt
    ori = ori @ angular_velocity_world.to_matrix()

    gravity_vector_measured = ori @ (sample.accelerometer - accelerometer_bias).normalized()
    gravity_vector_from_pose_estimate = GRAVITY.normalized()

    axis = gravity_vector_measured.cross(gravity_vector_from_pose_estimate)
    axis = ori.inverted() @ axis 
    corrective_quat = Quaternion(axis.normalized(), math.asin(axis.length * ACCELEROMETER_ANGLE_GAIN * dt))

    ori = ori @ corrective_quat.to_matrix()

    acceleration = ori @ (sample.accelerometer - accelerometer_bias) - GRAVITY
    linear_velocity += acceleration * dt
    # translation += linear_velocity * dt

    return Pose(
        worldTransform=Matrix.LocRotScale(translation, ori, Vector([1,1,1])),
        worldLinearVelocity=linear_velocity
    )



# class LidarEvent(NamedTuple):
#     time: float
#     distance_left: float
#     distance_right: float

# class ImuEvent(NamedTuple):
#     time: float
#     accelerometer: Vector
#     gyro: Vector
#     dt: float

# class PoseEstimateEvent(NamedTuple):
#     time: float
#     world_transform: Matrix
#     world_linear_velocity: Vector
#     world_angular_velocity: Vector


# class OdometryEvent(NamedTuple):
#     time: float
#     linear_velocity: Vector
#     angular_velocity: Vector


# Event = LidarEvent | ImuEvent | PoseEstimateEvent | OdometryEvent

# def get_pose_estimate_at_time(events: Event, time: float):
#     pass


class Simulator:
    def __init__(self, obj):
        self.obj = obj
        self.chassis = ChassisConnection()
        self.lidar_left = LidarVizualizer(get_children_by_property(self.obj, 'LidarLeft')[0])
        self.lidar_right = LidarVizualizer(get_children_by_property(self.obj, 'LidarRight')[0])
        self.imu_transform = get_children_by_property(self.obj, 'IMU')[0].localTransform

        self.gyro_bias = Vector([0,0,0])
        self.accelerometer_bias = Vector([0,0,0])
        self.angular_velocity_estimate = Vector([0,0,0])
        self.previous_orientation = self.obj.worldOrientation
        self.linear_velocity_worldspace = Vector([0,0,0])

        self.i = 0

        self.mission = CalibrateGyro(self.chassis)

        self.chassis.on_sample.append(self.update_pose_estimate_from_sample)
        self.pose_estimate_history: deque[Matrix] = deque(maxlen=100)
        self.prev_sample_time = None


    def update_pose_estimate_from_sample(self, new_sample: ChassisSample):
        self.lidar_left.distance_mm = new_sample.distance_left_mm
        self.lidar_right.distance_mm = new_sample.distance_right_mm

        if self.prev_sample_time is None:
            self.prev_sample_time = new_sample.time_ms
            return
        
        dt = (new_sample.time_ms - self.prev_sample_time) / 1000
        self.prev_sample_time = new_sample.time_ms

        previous_pose = Pose(
            worldTransform=self.obj.worldTransform @ self.imu_transform,
            worldLinearVelocity=self.linear_velocity_worldspace,
        )
        newPose = estimate_new_pose_from_imu(
            previous_pose=previous_pose,
            sample=new_sample,
            gyro_bias=self.gyro_bias,
            accelerometer_bias=self.accelerometer_bias,
            dt=dt
        )
        
        self.obj.worldTransform = newPose.worldTransform @ self.imu_transform.inverted()
        self.linear_velocity_worldspace = newPose.worldLinearVelocity

        

    def run(self):
        dt = 1 / bge.logic.getAverageFrameRate()
        self.chassis.poll()

        
        if not self.chassis.samples:
            print("No Connection")
            return



        if self.mission:
            self.mission.run(dt)
            if self.mission.is_complete:
                self.gyro_bias = self.mission.gyro_calibration
                self.accelerometer_bias = self.mission.accelerometer_calibration
                self.mission = None

        else:
            self.lidar_left.update()
            self.lidar_right.update()

            self.chassis.command(ChassisCommand(
                speed=0,
                direction=20
            ))


def start():
    cont = bge.logic.getCurrentController()
    obj = cont.owner
    if "sim" not in obj:
        obj["sim"] = Simulator(obj)
    else:
        obj["sim"].run()
    
