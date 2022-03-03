import csv
import os
import sys
import time
from time import strftime, localtime

import cv2
import matplotlib.pyplot as plt
import numpy as np

from awr1642driver import awr1642
from icm20948driver import IMU as imu
from imx21983driver import CSI_Camera, gstreamer_pipeline

print('salam dada')


def undistort_func(dimension):
    mtx_left = np.array([[1.22443969e+03, 0.00000000e+00, 4.56407162e+02],
                         [0.00000000e+00, 1.22491671e+03, 2.01622856e+02],
                         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist_left = np.array([[-0.04521501, 0.92680653, -0.00826225, -0.00314583, -2.74432789]])

    mtx_right = np.array([[1.24066436e+03, 0.00000000e+00, 4.62903179e+02],
                          [0.00000000e+00, 1.23937692e+03, 1.70165070e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist_right = np.array([[-0.00970787,  0.59812137, -0.01065168, -0.00521563, -1.62548014]])
    h,  w = dimension[:2]
    newcameramtx_l, roi_l = cv.getOptimalNewCameraMatrix(mtx_left, dist_left, (w, h), 1, (w, h))
    newcameramtx_r, roi_r = cv.getOptimalNewCameraMatrix(mtx_right, dist_right, (w, h), 1, (w, h))
    return mtx_left, dist_left, newcameramtx_l, roi_l, mtx_right, dist_right, newcameramtx_r, roi_r

def heatmap(xr, yr, zr, xlim, ylim, xc=np.nan, yc=np.nan, xbinnum=100, ybinnum=100):

    x_edges = np.linspace(xlim[0], xlim[1], xbinnum)
    y_edges = np.linspace(ylim[0], ylim[1], ybinnum)

    try:
        valid_list = np.logical_and(
            np.logical_and(xr >= xlim[0], xr <= xlim[1]),
            np.logical_and(yr >= ylim[0], yr <= ylim[1]))

        xr = xr[valid_list]
        yr = yr[valid_list]
        zr = zr[valid_list]

        indx = np.digitize(xr, x_edges)
        indy = np.digitize(yr, y_edges)

        xr = x_edges[indx - 1]
        yr = y_edges[indy - 1]

        indx = np.digitize(xc, x_edges)
        indy = np.digitize(yc, y_edges)

        xc = x_edges[indx - 1]
        yc = y_edges[indy - 1]

        tab = np.zeros([xbinnum, ybinnum])

        for i in range(len(xr)):
            tab[np.where(x_edges == xr[i]), np.where(y_edges == yr[i])] = + zr[i]

        try:
            for i in range(len(xc)):
                tab[np.where(x_edges == xc[i]), np.where(y_edges == yc[i])] = + 1
        except:
            pass

        tab = tab.reshape((xbinnum, ybinnum, 1)).astype(np.uint8)
        img = cv2.cvtColor(tab, cv2.COLOR_GRAY2BGR)

        return img
    except:
        pass

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
        self.imu = None
        self.writer = None
        self.radarLeftObjList = []
        self.radarRightObjList = []
        self.cameraLeftObjList = []
        self.cameraRightObjList = []
        self.imuObjList = []
        self.sensorList = []
        self.display = 0
        self.displayMethod = 'heatmap'
        self.save = 0
        self.index = 0
        self.sampling = None
        self.blankImg = cv2.imread('NoSignal.jpg', cv2.IMREAD_UNCHANGED)
        self.blankImgshape = self.blankImg.shape
        self.Left_Stereo_Map_x = None
        self.Left_Stereo_Map_y = None
        self.Right_Stereo_Map_x = None
        self.Right_Stereo_Map_y = None
        self.numDisparities = None
        self.blockSize = None
        self.preFilterType = None
        self.preFilterSize = None
        self.preFilterCap = None
        self.textureThreshold = None
        self.uniquenessRatio = None
        self.speckleRange = None
        self.speckleWindowSize = None
        self.disp12MaxDiff = None
        self.minDisparity = None
        self.stereo = None
        self.getCamPars('params_py.xml', 'depth_pars.xml')

    class objCreate:
        def __init__(self, time, index, data, data_is_ok=0):
            self.time = time
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
        elif sensor_id == 'imu':
            self.imuObjList.append(obj)

    def setDirectories(self):
        try:
            self.dirName = strftime("%d-%b-%Y-%H-%M", localtime())
            os.makedirs(self.dirName)
            self.dirPath += '/' + self.dirName
            os.chdir(self.dirPath)
            if self.left_camera:
                os.makedirs('./Figure-left-camera/')
            if self.right_camera:
                os.makedirs('./Figure-right-camera/')
        except OSError:
            self.closeAll()
            sys.exit('[Error] Cannot make directories')

    def setup(self):
        try:
            os.system('sudo service nvargus-daemon stop')
            os.system('sudo service nvargus-daemon start')
        except:
            pass
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
                try:
                    self.displayMethod = 'scatter' if int(splitWords[3]) else 'heatmap'
                except:
                    pass

            elif "sampling" in splitWords[0] and int(splitWords[1]):
                self.sampling = float(splitWords[2])

            elif "imu" in splitWords[0] and int(splitWords[1]):
                self.imu = imu("imu")
                self.imu.open()
                self.sensorList.append(self.imu)

    def closeAll(self):
        for obj in [self.left_radar, self.right_radar]:
            try:
                obj.close()
            except Exception as error:
                print(error)
                continue
        for obj in [self.left_camera, self.right_camera]:
            try:
                obj.stop()
                obj.release()
            except Exception as error:
                print(error)
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
                print("\r[INFO] Actual rate:%d" % int(1/(time.time()-start_time)), end="")
            elif time.time()-start_time > 1/self.sampling:
                print("\r[WARNING] High sampling rate - Actual rate:%d" % int(1/(time.time()-start_time)), end="")
                time.sleep(0.001)
                print("", end='\r')
        else:
            if time.time()-start_time < 1/self.sampling:
                cv2.waitKey((1/self.sampling + start_time - time.time()) * 1000)
                print("\r[INFO] Actual rate:%d" % int(1/(time.time()-start_time)), end="")
            elif time.time()-start_time > 1/self.sampling:
                print("[WARNING] High sampling rate- Actual rate:%d" % int(1/(time.time()-start_time)), end="")
                time.sleep(0.001)
                print("", end='\r')
                cv2.waitKey(1)

    def readAll(self):
        for sensor in self.sensorList:
            grabbed, data, time_stamp = sensor.read()
            self.append(self.objCreate(
                time_stamp,
                self.index,
                data, grabbed),
                sensor.sensorID)

    def saveAll(self):
        if self.save:
            radar_header = \
                ["index", "time", "x", "y", "range", "peakVal", "doppler"]
            camera_header = \
                ["index", "time", "imageName"]
            imu_header = ["index", "time", "x", "y", "z", "ax", "ay", "az", "gx", "gy", "gz"]
            if self.right_radar:
                print("[SAVE] Right RADAR: Creating CSV File", )
                with open(self.dirPath+'/'+self.dirName+'right-radar.csv','w', newline='') \
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(radar_header)
                    for object in self.radarRightObjList:
                        print("Saving %d out of %d" %(object.index,
                                                      self.radarRightObjList.__len__()), end="")
                        if object.data_is_ok:
                            writer.writerow([
                                object.index,
                                object.time,
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
                with open(self.dirPath+'/'+self.dirName+'left-radar.csv', 'w', newline='') \
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(radar_header)
                    for object in self.radarLeftObjList:
                        print("Saving %d out of %d" % (object.index,
                                                       self.radarLeftObjList.__len__()), end="")
                        if object.data_is_ok:
                            writer.writerow([
                                object.index,
                                object.time,
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
                with open(self.dirPath+'/'+self.dirName+'left-camera.csv', 'w', newline='') \
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(camera_header)
                    for object in self.cameraLeftObjList:
                        print("Saving %d out of %d" % (object.index,
                                                       self.cameraLeftObjList.__len__()), end="")
                        filename = "Figure-left-camera/img_left_%d.jpg" % object.index
                        if object.data_is_ok:
                            writer.writerow([
                                object.index,
                                object.time,
                                filename
                            ])
                            cv2.imwrite(filename, object.data)
                        else:
                            writer.writerow([
                                object.index,
                                object.time,
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
                                object.index,
                                object.time,
                                filename
                            ])
                            cv2.imwrite(filename, object.data)
                        else:
                            writer.writerow([
                                object.index,
                                object.time,
                                "",
                            ])
                        print("", end="\r")
                    print("\nSuccessful")
            if self.imu:
                print("[SAVE] imu: Creating CSV File", )
                with open(self.dirPath+'/'+self.dirName+'imu.csv', 'w', newline='') \
                        as outputfile:
                    writer = csv.writer(outputfile)
                    writer.writerow(imu_header)
                    for object in self.imuObjList:
                        print("Saving %d out of %d" % (object.index,
                                                       self.radarRightObjList.__len__()), end="")
                        if object.data_is_ok:
                            writer.writerow([
                                object.index,
                                object.time,
                                object.data[0],
                                object.data[1],
                                object.data[2],
                                object.data[3],
                                object.data[4],
                                object.data[5],
                                object.data[6],
                                object.data[7],
                                object.data[8],
                            ])
                        else:
                            writer.writerow([
                                object.time,
                                object.index,
                                "", "", "", "", "", "", "", "", "",
                            ])
                        print("", end="\r")
                    print("\nSuccessful")

    def displayAll(self, start_time, window_title):
        if self.left_camera:
            left_image = self.cameraLeftObjList[-1].data
        else:
            left_image = self.blankImg
        if self.right_camera:
            right_image = self.cameraRightObjList[-1].data.copy()
        else:
            right_image = self.blankImg
        # disparity = self.stereo.compute(cv2.cvtColor(
        #     left_image,cv2.COLOR_BGR2GRAY),
        #     cv2.cvtColor(right_image,cv2.COLOR_BGR2GRAY)
        # )
        left_image = self.camCalib(left_image, 'left')
        right_image = self.camCalib(right_image, 'right')
        disparity = self.stereo.compute(
            left_image[:, :, 2],
            right_image[:, :, 2]
        )
        left_image = cv2.cvtColor(disparity, cv2.COLOR_GRAY2BGR)
        if self.left_radar:
            x = self.radarLeftObjList[-1].data["x"]
            y = self.radarLeftObjList[-1].data["y"]
            z = self.radarLeftObjList[-1].data["peakVal"]
            if self.displayMethod == 'scatter':
                plt.scatter(x, y, s=50, c=z, cmap='gray')
                plt.xlim([-5, 5])
                plt.ylim([0, 5])
                plt.savefig('imgl.png')
                plt.close()
                img = cv2.imread('imgl.png')
            else:
                img = heatmap(x, y, z, (-2, 2), (0, 4))
            left_radar = cv2.resize(img,
                             (self.blankImgshape[1], self.blankImgshape[0]),
                             interpolation=cv2.INTER_AREA)
        else:
            left_radar = self.blankImg
        if self.right_radar:
            x = self.radarRightObjList[-1].data["x"]
            y = self.radarRightObjList[-1].data["y"]
            z = self.radarRightObjList[-1].data["peakVal"]
            if self.displayMethod == 'scatter':
                plt.scatter(x, y, s=50, c=z, cmap='gray')
                plt.xlim([-5, 5])
                plt.ylim([0, 5])
                plt.savefig('imgl.png')
                plt.close()
                img = cv2.imread('imgl.png')
            else:
                img = heatmap(x, y, z, (-2, 2), (0, 4))
            right_radar = cv2.resize(img,
                                    (self.blankImgshape[1], self.blankImgshape[0]),
                                    interpolation=cv2.INTER_AREA)
        else:
            right_radar = self.blankImg
        camera_images = np.hstack((left_image, right_image))
        radar_images = np.hstack((left_radar, right_radar))
        final_image = np.vstack((camera_images, radar_images))
        cv2.imshow(window_title, cv2.resize(final_image, (800,600)))
        self.loopCycleControl(start_time, sleep_enable=0)

    def run(self):
        if self.save:
            self.setDirectories()
        for sensor in self.sensorList:
            sensor.start()
            print(sensor, sensor.sensorID, sensor.isOpened)
        if self.sensorList \
                and all([sensor.isOpened for sensor in self.sensorList]):
            print('[INFO] Recording...')
            if not self.display:
                try:
                    while True:
                        start_time = time.time()
                        self.readAll()
                        self.loopCycleControl(start_time)
                        self.index += 1
                except KeyboardInterrupt:
                    print("\n Capture distrupted")
                    self.closeAll()
                    if self.save:
                        self.saveAll()
            else:
                window_title = "Data Acquisition "
                cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
                try:
                    while True:
                        start_time = time.time()
                        self.readAll()
                        self.displayAll(start_time, window_title)
                        self.index += 1
                except KeyboardInterrupt:
                    print("\n Capture distrupted")
                    time.sleep(3)
                    self.closeAll()
                    cv2.destroyAllWindows()
                    time.sleep(3)
                    if self.save: self.saveAll()
        else:
            print("[ERROR] Not all sensors have been opened correctly")
            self.closeAll()

    def getCamPars(self, path_to_calib_xml, path_to_depth_xml):
        cv_file = cv2.FileStorage(path_to_calib_xml, cv2.FILE_STORAGE_READ)
        self.Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
        self.Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
        self.Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
        self.Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
        cv_file.release()
        cv_file = cv2.FileStorage(path_to_depth_xml, cv2.FILE_STORAGE_READ)
        self.numDisparities = int(cv_file.getNode("numDisparities").real())
        self.blockSize = int(cv_file.getNode("blockSize").real())
        self.preFilterType = int(cv_file.getNode("preFilterType").real())
        self.preFilterSize = int(cv_file.getNode("preFilterSize").real())
        self.preFilterCap = int(cv_file.getNode("preFilterCap").real())
        self.textureThreshold = int(cv_file.getNode("textureThreshold").real())
        self.uniquenessRatio = int(cv_file.getNode("uniquenessRatio").real())
        self.speckleRange = int(cv_file.getNode("speckleRange").real())
        self.speckleWindowSize = int(cv_file.getNode("speckleWindowSize").real())
        self.disp12MaxDiff = int(cv_file.getNode("disp12MaxDiff").real())
        self.minDisparity = int(cv_file.getNode("minDisparity").real())
        cv_file.release()
        self.stereo = cv2.StereoBM_create()
        self.stereo.setNumDisparities(self.numDisparities)
        self.stereo.setBlockSize(self.blockSize)
        self.stereo.setPreFilterType(self.preFilterType)
        self.stereo.setPreFilterSize(self.preFilterSize)
        self.stereo.setPreFilterCap(self.preFilterCap)
        self.stereo.setTextureThreshold(self.textureThreshold)
        self.stereo.setUniquenessRatio(self.uniquenessRatio)
        self.stereo.setSpeckleRange(self.speckleRange)
        self.stereo.setSpeckleWindowSize(self.speckleWindowSize)
        self.stereo.setDisp12MaxDiff(self.disp12MaxDiff)
        self.stereo.setMinDisparity(self.minDisparity)

    def camCalib(self, img, sensor_id):
        if 'left' in sensor_id:
            img_out = cv2.remap(
                img,
                self.Left_Stereo_Map_x,
                self.Left_Stereo_Map_y,
                cv2.INTER_LANCZOS4,
                cv2.BORDER_CONSTANT,
                0,
            )
        elif 'right' in sensor_id:
            img_out = cv2.remap(
                img,
                self.Right_Stereo_Map_x,
                self.Right_Stereo_Map_y,
                cv2.INTER_LANCZOS4,
                cv2.BORDER_CONSTANT,
                0,
            )
        return img_out

    def left_camera_postprocess(self):
        while self.left_camera.running:
            try:
                if self.left_camera.postprocess.new_data and not self.left_camera.postprocess.processed:
                    with self.left_camera.postprocess.read_lock:
                        img = self.left_camera.postprocess.data.copy()
                    img = self.camCalib(img, 'left')
                    with self.left_camera.postprocess.read_lock:
                        self.left_camera_output = img
                        self.left_camera.postprocess.processed = True
                        self.left_camera.postprocess.new_data = False
            except:
                print('[ERROR] Left camera post processing failed')

    def right_camera_postprocess(self):
        while self.right_camera.running:
            try:
                if self.right_camera.postprocess.new_data and not self.right_camera.postprocess.processed:
                    with self.right_camera.postprocess.read_lock:
                        img = self.right_camera.postprocess.data.copy()
                    img = self.camCalib(img, 'right')
                    with self.right_camera.postprocess.read_lock:
                        self.right_camera_output= img # post porcessed
                        self.right_camera.postprocess.processed = True
                        self.right_camera.postprocess.new_data = False
            except:
                print('[ERROR] Right camera post processing failed')

    def left_radar_postprocess(self):
        while self.left_radar.running:
            if self.left_radar.postprocess.new_data and not self.left_radar.postprocess.processed:
                # process on new data
                with self.left_radar.postprocess.read_lock:
                    data = self.left_radar.postprocess.data.copy()
                x = data["x"]
                y = data["y"]
                z = data["peakVal"]
                if self.displayMethod == 'scatter':
                    plt.scatter(x, y, s=50, c=z, cmap='gray')
                    plt.xlim([-5, 5])
                    plt.ylim([0, 5])
                    plt.savefig('imgl.png')
                    plt.close()
                    img = cv2.imread('imgl.png')
                else:
                    img = heatmap(x, y, z, (-2, 2), (0, 4))
                img = cv2.resize(img,
                                        (self.blankImgshape[1], self.blankImgshape[0]),
                                        interpolation=cv2.INTER_AREA)
                with self.left_radar.postprocess.read_lock:
                    self.left_radar_output = img # post porcessed
                self.left_radar.postprocess.processed = True
                self.left_radar.postprocess.new_data = False

    def right_radar_postprocess(self):
        while self.right_radar.running:
            if self.right_radar.postprocess.new_data and not self.right_radar.postprocess.processed:
                # process on new data
                with self.right_radar.postprocess.read_lock:
                    data = self.right_radar.postprocess.data.copy()
                x = data["x"]
                y = data["y"]
                z = data["peakVal"]
                if self.displayMethod == 'scatter':
                    plt.scatter(x, y, s=50, c=z, cmap='gray')
                    plt.xlim([-5, 5])
                    plt.ylim([0, 5])
                    plt.savefig('imgr.png')
                    plt.close()
                    img = cv2.imread('imgr.png')
                else:
                    img = heatmap(x, y, z, (-2, 2), (0, 4))
                img = cv2.resize(img,
                                 (self.blankImgshape[1], self.blankImgshape[0]),
                                 interpolation=cv2.INTER_AREA)
                with self.right_radar.postprocess.read_lock:
                    self.right_radar_output = img # post porcessed
                self.right_radar.postprocess.processed = True
                self.right_radar.postprocess.new_data = False


if __name__ == "__main__":
    target = sensor_read("dacProfile.cfg")
    target.setup()
    target.run()

