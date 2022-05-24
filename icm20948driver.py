import threading
import time
import csv
import os
import queue

from icm20948 import ICM20948


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
        self.sensorIsReady = False
        self.postprocess = None
        self.logObj = None

    def readAll(self):
        x, y, z = self.dev.read_magnetometer_data()
        ax, ay, az, gx, gy, gz = self.dev.read_accelerometer_gyro_data()
        grabbed = 1 if x or y or ax or ay else 0
        measurements = [x, y, z, ax, ay, az, gx, gy, gz]
        measurements = [round(m, 3) for m in measurements]
        return grabbed, measurements

    def open(self):
        try:
            self.dev = ICM20948()
            x, y, z = self.dev.read_magnetometer_data()
            ax, ay, az, gx, gy, gz = self.dev.read_accelerometer_gyro_data()
        except:
            print("Unable to open IMU")
        if x and ax and gx:
            self.sensorIsReady = True
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
                print("Could not read data from IMU")
            except OSError:
                self.dev = ICM20948()
            time.sleep(.005)

    def read(self):
        with self.read_lock:
            time_stamp = time.time()
            frame = self.frame
            grabbed = self.grabbed
            self.frame = None
            self.grabbed = 0
        return grabbed, frame, time_stamp

    def log(self, tempObj):
        if tempObj.data_is_ok:
            self.logger.writerow([
                tempObj.index,
                tempObj.time,
                tempObj.data[0],
                tempObj.data[1],
                tempObj.data[2],
                tempObj.data[3],
                tempObj.data[4],
                tempObj.data[5],
                tempObj.data[6],
                tempObj.data[7],
                tempObj.data[8],
            ])
        else:
            self.logger.writerow([
                tempObj.index,
                tempObj.time,
                "", "", "", "", "", "", "", "", "",
            ])

    def logP(self, log_buffer):
        log_file = open(f'{os.getpid()}.csv', 'w', newline='')
        file_header = ["index", "time", "x", "y", "z", "ax", "ay", "az", "gx", "gy", "gz"]
        logger = csv.writer(log_file)
        logger.writerow(file_header)
        while self.logObj.running.value or not log_buffer.empty():
            try:
                tempObj = log_buffer.get(0)
                data = tempObj.data if tempObj.data_is_ok else []
                logger.writerow([
                    tempObj.index,
                    tempObj.time] + data)
            except RuntimeError:
                print('[ERROR] Could not save an frame')
            except queue.Empty:
                pass
        else:
            log_file.close()
            os.rename(f'{os.getpid()}.csv', f'{tempObj.source}.csv')
            print(f"[OK] Process id: {self.logObj.name} has joined")

    def stop(self):
        self.running = False
        self.read_thread.join()
        self.read_thread = None
        print(f'[OK]{self.sensorID} is closed')