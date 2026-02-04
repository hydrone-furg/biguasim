import numpy as np
from biguasim.shmem import Shmem
from threading import Thread
import time

class SharedMemoryReceiver:
    def __init__(self, uuid: str, agent:str="auv0"):
        self.uuid = uuid
        self.agent = agent
        self.position = np.zeros(3)
        self.rpy = np.zeros(3)
        self.pos_mem = Shmem(f"{agent}_LocationSensor_sensor_data", (3,), np.float32, uuid)
        self.rot_mem = Shmem(f"{agent}_RotationSensor_sensor_data", (3,), np.float32, uuid)
        # self.sensors_mem = Shmem(f"{agent}_")
        self.running = True
        self.thread = Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self.running:
            self.position = np.copy(self.pos_mem.np_array)
            self.rpy = np.copy(self.rot_mem.np_array)
            time.sleep(0.05)
