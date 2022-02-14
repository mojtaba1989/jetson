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
                    self.blankImg = cv2.resize(
                        self.blankImg,
                        (int(splitWords[6]),
                         int(splitWords[7])),
                        interpolation=cv2.INTER_AREA
                    )
                    self.blankImgshape = self.blankImg.shape
            elif "radar" in splitWords[0] and int(splitWords[1]):
                if not int(splitWords[2]):
                    self.left_radar = awr1642(
                        splitWords[5],
                        splitWords[3],
                        splitWords[4],
                        'left_radar'
                    )
                    self.left_radar.sensorSetup()
                    time.sleep(.01)
                    if self.sampling:
                        self.left_radar.setSampleRate(self.sampling)
                        time.sleep(.01)
                    self.left_radar.open()
                    time.sleep(.01)
                    self.sensorList.append(self.left_radar)
                else:
                    self.right_radar = awr1642(
                        splitWords[5],
                        splitWords[3],
                        splitWords[4],
                        'right_radar'
                    )
                    self.right_radar.sensorSetup()
                    time.sleep(.01)
                    if self.sampling:
                        self.right_radar.setSampleRate(self.sampling)
                        time.sleep(.01)
                    self.right_radar.open()
                    time.sleep(.01)
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

    def loopCycleControl(self, start_time, sleep_enable=1):
        if sleep_enable:
            if time.time()-start_time < 1/self.sampling:
                time.sleep(1/self.sampling +
                           start_time -time.time())
            elif time.time()-start_time > 1/self.sampling:
                print("[WARNING] High sampling rate")
        else:
            if time.time()-start_time < 1/self.sampling:
                return int((1/self.sampling + start_time - time.time()) * 1000)
            elif time.time()-start_time > 1/self.sampling:
                print("[WARNING] High sampling rate")
                return 10

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
                                                      self.radarRightObjList.__len__()), end="")
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
                                                       self.radarLeftObjList.__len__()), end="")
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
                        filename = "Figure-left-camera/img_left_%d.jpg" % object.index
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
                        filename = "Figure-right-camera/img_right_%d.jpg" % object.index
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

    def displayAll(self, start_time, fig, sc1, sc2):
        window_title = "Data Acquisition "
        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
        left_image = self.cameraLeftObjList[-1].data if self.cameraLeftObjList and self.cameraLeftObjList[-1].data_is_ok else self.blankImg
        right_image = self.cameraRightObjList[-1].data if self.cameraRightObjList and self.cameraRightObjList[-1].data_is_ok else self.blankImg
        if self.radarLeftObjList and self.radarLeftObjList[-1].data_is_ok:
            x = self.radarLeftObjList[-1].data["x"]
            y = self.radarLeftObjList[-1].data["y"]
            sc1.set_offsets(np.c_[x,y])
            fig1.canvas.draw_idle()
        else:
            left_radar = self.blankImg
        if self.radarRightObjList and self.radarRightObjList[-1].data_is_ok:
            x = self.radarRightObjList[-1].data["x"]
            y = self.radarRightObjList[-1].data["y"]
            sc2.set_offsets(np.c_[x,y])
            fig2.canvas.draw_idle()
        else:
            right_radar = self.blankImg
        camera_images = np.hstack((left_image, right_image))
        final_image = np.vstack((camera_images, radar_images))
        cv2.imshow(window_title, cv2.resize(final_image, (800,300)))
        keyCode = cv2.waitKey(self.loopCycleControl(start_time, sleep_enable=0))

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
                plt.ion()
                fig1, axes1 = plt.subplots()
                fig2, axes2 = plt.subplots()
                x, y = [],[]
                sc1 = axes1.scatter(x,y)
                sc2 = axes2.scatter(x,y)
                plt.draw()
                try:
                    while True:
                        start_time = time.time()
                        self.readAll()
                        self.displayAll(start_time, fig1, fig2, sc1, sc2)
                        self.index += 1
                except KeyboardInterrupt:
                    self.closeAll()
                    cv2.destroyAllWindows()
                    self.saveAll() if self.save else None
        else:
            print("[ERROR] Not all sensors have been opened correctly")
            self.closeAll()

if __name__ == "__main__":
    target = sensor_read("dacProfile.cfg")
    target.setup()
    target.run()


