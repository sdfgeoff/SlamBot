
import itertools
from chassis import ChassisConnection
from imu import GRAVITY
from missions.mission import Mission
from util import get_deviation_and_average
from mathutils import Vector


class CalibrateGyro(Mission):
    STABLE_SAMPLES = 20
    CALIBRATION_SAMPLES = 60

    def __init__(self, chassis: ChassisConnection):
        super().__init__()
        self.chassis = chassis

        self.gyro_calibration = None
        self.accelerometer_calibration = None

    def run(self, dt):
        # self.chassis.drive(0,0)
        if len(self.chassis.samples) < self.STABLE_SAMPLES + self.CALIBRATION_SAMPLES:
            return
        
        shaking_detection_samples = [s.gyro for s in itertools.islice(self.chassis.samples, len(self.chassis.samples)-(self.STABLE_SAMPLES + self.CALIBRATION_SAMPLES), len(self.chassis.samples))]
        shaking_deviation, _shaking_average = get_deviation_and_average(shaking_detection_samples)

        if shaking_deviation > 1e-5:
            print("Device still moving. Waiting before calibrating gyro")
            return

        calibration_gyro_samples = [s.gyro for s in itertools.islice(self.chassis.samples, len(self.chassis.samples)-(self.CALIBRATION_SAMPLES), len(self.chassis.samples))]
        _calibration_gyro_deviation, calibration_gyro_average = get_deviation_and_average(calibration_gyro_samples)

        calibration_accelerometer_samples = [s.accelerometer for s in itertools.islice(self.chassis.samples, len(self.chassis.samples)-(self.CALIBRATION_SAMPLES), len(self.chassis.samples))]
        _calibration_accelerometer_deviation, calibration_accelerometer_average = get_deviation_and_average(calibration_accelerometer_samples)


        gravity = calibration_accelerometer_average.normalized() * GRAVITY.length
        self.accelerometer_calibration = calibration_accelerometer_average - gravity

        self.gyro_calibration = calibration_gyro_average
        self.is_complete = True
