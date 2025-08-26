from collections import deque
import json
from multiprocessing import Pool, Process
from queue import Empty, Queue
from threading import Thread
import requests
from typing import Callable, List, NamedTuple
from mathutils import Vector, Matrix, Euler, Quaternion
import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200


class ChassisSample(NamedTuple):
    gyro: Vector
    accelerometer: Vector
    time_ms: int
    distance_left_mm: float
    distance_right_mm: float

class ChassisCommand(NamedTuple):
    speed: int
    direction: int


class UartChassisConnection:
    def __init__(self):
        self.serial = serial.Serial(port=PORT, baudrate=BAUD, timeout=0)
        self.serialBuffer: bytes = b''
        self.samples: deque[ChassisSample] = deque(maxlen=1000)
        
        self.on_sample: List[Callable[[ChassisSample], None]] = []

    def poll(self):
        self.serialBuffer += self.serial.read(1000000)
        while b'\r\n' in self.serialBuffer:
            before, after = self.serialBuffer.split(b'\r\n', maxsplit=1)
            self.serialBuffer = after
            try:
                raw_packet = json.loads(before)
                sample = ChassisSample(
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
                    time_ms=raw_packet['t'],
                    distance_left_mm=raw_packet['dl'],
                    distance_right_mm=raw_packet['dr']
                )
                

            except Exception as e:
                print("Json Decode Error:", e)
            else:
                self.samples.append(self.on_sample)
                [f(sample) for f in self.on_sample]


URL = "http://192.168.18.52/"
def server_thread(from_server: Queue, to_server: Queue, keep_running):
    print("Starting Networking Thread")
    session = requests.Session()


    def get_command_or_none() -> ChassisCommand:
        try:
            latest: ChassisCommand = to_server.get_nowait()
        except Empty:
            return None
        else:
            return latest


    while keep_running():
        resp = session.get(URL)
        try:
            raw_packet = resp.json()
            sample = ChassisSample(
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
                time_ms=raw_packet['t'],
                distance_left_mm=raw_packet['dl'],
                distance_right_mm=raw_packet['dr']
            )
        except Exception as e:
            print("Network Error:", e)
        else:
            from_server.put(sample)
        
        latest = get_command_or_none()
        more = True
        while more:
            new = get_command_or_none()
            if new is not None:
                latest = new
            else:
                more = False
        to_server_packet = latest
        if to_server_packet is not None:
            print("Posting", to_server_packet)
            resp = session.post(URL+f"control?speed={to_server_packet.speed}&direction={to_server_packet.direction}")
        




class NetworkChassisConnection:
    def __init__(self):
        self.on_sample = []
        self.samples: deque[ChassisSample] = deque(maxlen=1000)
        self.from_server: Queue[ChassisSample] = Queue()
        self.to_server: Queue[ChassisCommand] = Queue()
        self.keep_running = True
        self.process = Thread(target=server_thread, args=(self.from_server, self.to_server, lambda: self.keep_running))
        self.process.start()

        global on_end_game
        on_end_game.append(self.shutdown)

    def poll_internal(self):
        try:
            sample = self.from_server.get_nowait()
        except Empty:
            return False
        else:
            self.samples.append(sample)
            [f(sample) for f in self.on_sample]
            return True
        
    def poll(self):
        while self.poll_internal():
            pass

    def command(self, command: ChassisCommand):
        self.to_server.put(command)

    def shutdown(self):
        self.keep_running = False
        self.process.join()
    

ChassisConnection = NetworkChassisConnection




class RunOnPowerOff(object):
    '''Runs a function when the scenario ends. Put it in a game object
    property'''
    def __init__(self, function):
        self.function = function


    def __del__(self):
        self.function()


on_end_game = []

def end_game():
    print("end game")
    [e() for e in on_end_game]

import bge
obj = bge.logic.getCurrentScene().objects["Camera"]
obj['RUNONPOWEROFF'] = RunOnPowerOff(end_game)