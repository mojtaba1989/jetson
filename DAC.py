import cv2
import threading
import numpy as np
import time
from time import gmtime, strftime, localtime
import platform
import os
import csv
import sys
import matplotlib.pyplot as plt

from imx21983driver import CSI_Camera, gstreamer_pipeline
from awr1642driver  import awr1642


class sensor_read:
    def __init__(self, configFileName):
        self.dirPath = os.getcwd()
        self.dirName = None
        self.configFileName = configFileName
        self.config = [line.rstrip('\r\n') for line in open(
            self.configFileName)]
        self.configParameters = {}
        self.left_radar = None
        self.right_radar = None
        self.left_camera = None
        self.right_camera = None
        self.IMU = None
        self.writer = None
        self.radarLeftObjList = []
        self.radarRightObjList = []
        self.cameraLeftObjList = []
        self.cameraRightObjList = []
        self.sensorList = []
        self.display = 0
        self.save = 0
        self.index = 0
        self.sampling = None
        self.blankImg = cv2.imread('NoSignal.jpg', cv2.IMREAD_UNCHANGED)
        self.blankImgshape = self.blankImg.shape

    class objCreate:
        def __init__(self, index, data, data_is_ok=0):
            self.time = time.time()
            self.index = index
            self.data = data
            self.data_is_ok = data_is_ok

    def append(self, obj, sensor_id):
        if sensor_id == "left_radar":
            self.radarLeftObjList.append(obj)
        elif sensor_id == "right_radar":
            self.radarRightObjList.append(obj)
        elif sensor_id == "left_camera":
            self.cameraLeftObjList.append(obj)
        elif sensor_id == "right_camera":
            self.cameraRightObjList.append(obj)

    def setDirectories(self):
        try:
            self.dirName = strftime("%d-%b-%Y-%H-%M", localtime())
            os.makedirs(self.dirName)
            self.dirPath += '/' + self.dirName
            os.chdir(self.dirPath)
            if self.left_camera:
                os.makedirs('./Figures-left-camera/')
            if self.right_camera:
                os.makedirs('./Figure-right-camera/')
        except OSError:
            self.closeAll()
            sys.exit('[Error] Cannot make directories')

    def setup(self):
        for i in self.config:
            splitWords = i.split(" ")
            if "camera" in splitWords[0] and int(splitWords[1]):
                if not int(splitWords[2]):
                    self.left_camera = CSI_Camera('left_camera')
                    self.left_camera.open(
                        gstreamer_pipeline(
                            sensor_id=int(splitWords[3]),
                            capture_width=int(splitWords[4]),
                            capture_height=int(splitWords[5]),
                            display_width=int(splitWords[6]),
                            display_height=int(splitWords[7]),
                            framerate=self.sampling if self.sampling
                            else int(splitWords[8]),
                            flip_method=int(splitWords[9]),
                        )
                    )
                    self.sensorList.append(self.left_camera)
                    self.blankImg = cv2.resize(
                        self.blankImg,
                        (int(splitWords[6]),
                         int(splitWords[7])),
                        interpolation=cv2.INTER_AREA
                    )
                    self.blankImgshape = self.blankImg.shape
                else:
                    self.right_camera = CSI_Camera('right_camera')
                    self.right_camera.open(
                        gstreamer_pipeline(
                            sensor_id=int(splitWords[3]),
                            capture_width=int(splitWords[4]),
                            capture_height=int(splitWords[5]),
                            display_width=int(splitWords[6]),
                            display_height=int(splitWords[7]),
                            framerate=self.sampling if self.sampling
                            else int(splitWords[8]),
                            flip_method=int(splitWords[9]),
                        )
                    )
                    self.sensorList.append(self.right_camera)
            elif "radar" in splitWords[0] and int(splitWords[1]):
                if not int(splitWords[2]):
                    self.left_radar = driver = awr1642(
                        splitWords[5],
                        splitWords[3],
                        splitWords[4],
                        'left_radar'
                    )
                    self.left_radar.sensorSetup()
                    if self.sampling:
                        self.left_radar.setSampleRate(self.sampling)
                    self.left_radar.open()
                    self.sensorList.append(self.left_radar)
                else:
                    self.right_radar = driver = awr1642(
                        splitWords[5],
                        splitWords[3],
                        splitWords[4],
                        'right_radar'
                    )
                    self.right_radar.sensorSetup()
                    if self.sampling:
                        self.right_radar.setSampleRate(self.sampling)
                    self.right_radar.open()
                    self.sensorList.append(self.right_radar)
            elif "output" in splitWords[0]:
                self.save = int(splitWords[1])
                self.display = int(splitWords[2])
            elif "sampling" in splitWords[0] \
                    and int(splitWords[1]):
                self.sampling = float(splitWords[2])

    def closeAll(self):
        for obj in [self.left_radar, self.right_radar]:
            try:
                obj.close()
            except Exception as error:
                print (error)
                continue
        for obj in [self.left_camera, self.right_camera]:
            try:
                obj.stop()
                obj.release()
            except Exception as error:
                print (error)
                continue

        try:
            cv2.destroyAllWindows()
        except:
            pass

    def loopCycleControl(self, start_time):
        if time.time()-start_time < 1/self.sampling:
            time.sleep(1/self.sampling +
                       start_time -time.time())
        elif time.time()-start_time > 1/self.sampling:
            print("[WARNING] High sampling rate")

    def readAll(self):
        for sensor in self.sensorList:
            grabbed, data = sensor.read()
            self.append(self.objCreate(
                self.index,
                data, grabbed),
                sensor.sensorID)

    def saveAll(self):
        if self.save:
            radar_header = \
                ["index", "time", "x", "y", "range", "peakVal", "doppler"]
            camera_header = \
                ["index", "time", "imageName"]
            if self.right_radar:
                print("[SAVE] Right RADAR: Creating CSV File", )
                with open(self.dirPath+'/'+self.dirName+'right-radar.csv','w', newline='')\
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(radar_header)
                    for object in self.radarRightObjList:
                        print("Saving %d out of %d" %(object.index,
                                                      self.radarRightObjList.__len__()))
                        if object.data_is_ok:
                            writer.writerow([
                                object.time,
                                object.index,
                                object.data["x"],
                                object.data["y"],
                                object.data["peakVal"],
                                object.data["doppler"],
                            ])
                        else:
                            writer.writerow([
                                object.time,
                                object.index,
                                "", "", "", "",
                            ])
                        print("", end="\r")
                    print("\nSuccessful")
            if self.left_radar:
                print("[SAVE] Left RADAR: Creating CSV File", )
                with open(self.dirPath+'/'+self.dirName+'left-radar.csv', 'w', newline='')\
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(radar_header)
                    for object in self.radarLeftObjList:
                        print("Saving %d out of %d" % (object.index,
                                                       self.radarLeftObjList.__len__()))
                        if object.data_is_ok:
                            writer.writerow([
                                object.time,
                                object.index,
                                object.data["x"],
                                object.data["y"],
                                object.data["peakVal"],
                                object.data["doppler"],
                            ])
                        else:
                            writer.writerow([
                                object.time,
                                object.index,
                                "", "", "", "",
                            ])
                        print("", end="\r")
                    print("\nSuccessful")
            if self.left_camera:
                print("[SAVE] Left Camera: Creating CSV File", )
                with open(self.dirPath+'/'+self.dirName+'left-camera.csv', 'w', newline='')\
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(camera_header)
                    for object in self.cameraLeftObjList:
                        print("Saving %d out of %d" % (object.index,
                                                       self.cameraLeftObjList.__len__()), end="")
                        filename = "/Figures-left-camera/img_left_%d.jpg" % object.index
                        if object.data_is_ok:
                            writer.writerow([
                                object.time,
                                object.index,
                                filename
                            ])
                            cv2.imwrite(filename, object.data)
                        else:
                            writer.writerow([
                                object.time,
                                object.index,
                                "",
                            ])
                        print("", end="\r")
                    print("\nSuccessful")
            if self.right_camera:
                print("[SAVE] Right Camera: Creating CSV File", )
                with open(self.dirPath+'/'+self.dirName+'right-camera.csv', 'w', newline='') \
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(camera_header)
                    for object in self.cameraRightObjList:
                        print("Saving %d out of %d" % (object.index,
                                                       self.cameraRightObjList.__len__()), end="")
                        filename = "/Figures-right-camera/img_right_%d.jpg" % object.index
                        if object.data_is_ok:
                            writer.writerow([
                                object.time,
                                object.index,
                                filename
                            ])
                            cv2.imwrite(filename, object.data)
                        else:
                            writer.writerow([
                                object.time,
                                object.index,
                                "",
                            ])
                        print("", end="\r")
                    print("\nSuccessful")

    def displayAll(self):
        left_image = self.cameraLeftObjList[-1].data if self.cameraLeftObjList and self.cameraLeftObjList[-1].data_is_ok else self.blankImg
        right_image = self.cameraRightObjList[-1].data if self.cameraRightObjList and self.cameraRightObjList[-1].data_is_ok else self.blankImg
        if self.radarLeftObjList and self.radarLeftObjList[-1].data_is_ok:
            fig = plt.figure()
            plt.scatter(
                self.radarLeftObjList[-1].data["x"],
                self.radarLeftObjList[-1].data["y"],
                s=200,
                c=self.radarLeftObjList[-1].data["peakVal"],
                cmap='gray')
            fig.canvas.draw()
            left_radar = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            left_radar = left_radar.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            left_radar = cv2.cvtColor(left_radar, cv2.COLOR_RGB2BGR)
            left_radar = cv2.resize(left_radar,
                                    (self.blankImgshape[1], self.blankImgshape[0]),
                                    interpolation=cv2.INTER_AREA)
        else:
            left_radar = self.blankImg
        if self.radarLeftObjList and self.radarRightObjList[-1].data_is_ok:
            fig = plt.figure()
            plt.scatter(
                self.radarRightObjList[-1].data["x"],
                self.radarRightObjList[-1].data["y"],
                s=200,
                c=self.radarRightObjList[-1].data["peakVal"],
                cmap='gray')
            fig.canvas.draw()
            right_radar = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
            right_radar = right_radar.reshape(fig.canvas.get_width_height()[::-1] + (3,))
            right_radar = cv2.cvtColor(right_radar, cv2.COLOR_RGB2BGR)
            right_radar = cv2.resize(right_radar,
                                    (self.blankImgshape[1], self.blankImgshape[0]),
                                    interpolation=cv2.INTER_AREA)
        else:
            right_radar = self.blankImg
        camera_images = np.hstack((left_image, right_image))
        radar_images = np.hstack((left_radar, right_radar))
        final_image = np.vstack((camera_images, radar_images))
        cv2.imshow("Data Acquisition ", final_image, cv2.WINDOW_NORMAL)

    def run(self):
        if self.save:
            self.setDirectories()
        for sensor in self.sensorList:
            sensor.start()
            print(sensor, sensor.sensorID, sensor.isOpened)
        if self.sensorList \
                and all([sensor.isOpened for sensor in self.sensorList]):
            if not self.display:
                try:
                    while True:
                        start_time = time.time()
                        self.readAll()
                        self.loopCycleControl(start_time)
                        self.index += 1
                except KeyboardInterrupt:
                    self.closeAll()
                    self.saveAll() if self.save else None
            else:
                try:
                    while True:
                        start_time = time.time()
                        self.readAll()
                        self.displayAll()
                        self.loopCycleControl(start_time)
                        self.index += 1
                except KeyboardInterrupt:
                    self.closeAll()
                    cv2.destroyAllWindows()
                    self.saveAll() if self.save else None
        else:
            print("[ERROR] Not all sensors have been opened correctly")
            self.closeALL()

if __name__ == "__main__":
    target = sensor_read("dacProfile.cfg")
    target.setup()
    target.run()


