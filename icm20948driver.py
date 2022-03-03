import time
from icm20948 import ICM20948
import threading
import numpy as np


class IMU:
    def __init__(self, sensor_id=None):
        self.dev = None
        self.frame = None
        self.grabbed = False
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False
        self.sensorID = sensor_id
        self.isOpened = False
        self.postprocess = None

    def readAll(self):
        x, y, z = self.dev.read_magnetometer_data()
        ax, ay, az, gx, gy, gz = self.dev.read_accelerometer_gyro_data()
        grabbed = 1 if x or y or ax or ay else 0
        return grabbed, [x, y, z, ax, ay, az, gx, gy, gz]

    def open(self):
        try:
            self.dev = ICM20948()
            x, y, z = self.dev.read_magnetometer_data()
            ax, ay, az, gx, gy, gz = self.dev.read_accelerometer_gyro_data()
        except:
            print("Unable to open IMU")
        if x and ax and gx:
            self.isOpened = True

    def start(self):
        if self.isOpened:
            self.running = True
            self.read_thread = threading.Thread(target=self.update)
            self.read_thread.start()
        return self

    def update(self):
        while self.running:
            try:
                grabbed, frame = self.readAll()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")

    def read(self):
        with self.read_lock:
            time_stamp = time.time()
            frame = self.frame
            grabbed = self.grabbed
        return grabbed, frame, time_stamp

