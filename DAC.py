import copy
import csv
import logging
import multiprocessing
import os
import queue
import shutil
import signal
import sys
import time
from multiprocessing import Queue, Value
from time import strftime, localtime

import cv2
import matplotlib.pyplot as plt
import numpy as np

from awr1642driver import awr1642
from icm20948driver import IMU as imu
from imx21983driver import CSI_Camera, gstreamer_pipeline


def get_ip():
    import subprocess
    lst_txt = subprocess.check_output('ifconfig', shell=True).decode("utf-8").splitlines()
    while True:
        temp = lst_txt.pop(0)
        if 'wlan' in temp:
            temp = lst_txt.pop(0).split(' ')
            break
    return temp[9]

class DelayedKeyboardInterrupt:

    def __enter__(self):
        self.signal_received = False
        self.old_handler = signal.signal(signal.SIGINT, self.handler)

    def handler(self, sig, frame):
        self.signal_received = (sig, frame)
        logging.debug('SIGINT received. Delaying KeyboardInterrupt.')

    def __exit__(self, type, value, traceback):
        signal.signal(signal.SIGINT, self.old_handler)
        if self.signal_received:
            self.old_handler(*self.signal_received)

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
    newcameramtx_l, roi_l = cv2.getOptimalNewCameraMatrix(mtx_left, dist_left, (w, h), 1, (w, h))
    newcameramtx_r, roi_r = cv2.getOptimalNewCameraMatrix(mtx_right, dist_right, (w, h), 1, (w, h))
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
        # configuration
        print('[INFO] Loading configuration...')
        self.dirPath = os.getcwd()
        self.dirName = None
        self.configFileName = configFileName
        self.config = [line.rstrip('\r\n') for line in open(self.configFileName)]
        self.configParameters = {}
        self.sampling = None
        self.is_running = False

        # sensor objects
        self.left_radar = None
        self.right_radar = None
        self.left_camera = None
        self.right_camera = None
        self.imu = None
        self.writer = None
        self.sensorList = []

        # sensor data object lists - CSV
        self.csvBuffer = []

        # display setup
        self.display = 0
        self.displayMethod = 'heatmap'
        self.blankImg = cv2.imread('NoSignal.jpg', cv2.IMREAD_UNCHANGED)
        self.blankImgShape = self.blankImg.shape

        # save method setup
        self.saveCSV = 0
        self.saveIMG = 0
        self.index = 0
        self.imgBuffer = Queue()
        self.leftRadarWriter = None
        self.rightRadarWriter = None
        self.leftCameraWriter = None
        self.rightCameraWriter = None
        self.imuWriter = None
        self.leftRadarFile = None
        self.rightRadarFile = None
        self.leftCameraFile = None
        self.rightCameraFile = None
        self.imuFile = None
        self.zip = None
        self.ip_address = None

        # image postprocessing
        self.camPars = self.stereoCalib()
        self.getCamPars('params_py.xml', 'depth_pars.xml')
        print('[OK] Loading configuration is done')

    class objCreate:
        def __init__(self, time, index, data, data_is_ok=0, source=None):
            self.time = time
            self.index = index
            self.data = data
            self.data_is_ok = data_is_ok
            self.isSaved = False
            self.isProcessed = False
            self.source = source

    class addProcessor:
        def __init__(self, name, target, args, num_of_cores=2):
            self.read_thread    = None
            self.name           = name
            self.running        = Value('i', 0)
            self.target         = target
            self.args           = args
            self.num_of_cores   = num_of_cores
            self.jobs           = []

        def start(self):
            self.running.value = 1
            for i in range(self.num_of_cores):
                process = multiprocessing.Process(target=self.target, args=self.args)
                self.jobs.append(process)
            for job in self.jobs:
                job.start()
            print(f'[INFO] Parallel {self.name} status: {bool(self.running)}')

        def stop(self):
            self.running.value = 0
            for job in self.jobs:
                if job.is_alive():
                    job.join()

    class stereoCalib:
        def __init__(self):
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

    def append(self, obj):
        self.csvBuffer.append(obj)
        if 'camera' in obj.source:
            self.imgBuffer.put(copy.copy(obj))


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
                    self.blankImgShape = self.blankImg.shape
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
                    self.blankImgShape = self.blankImg.shape
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
            elif "imu" in splitWords[0] and int(splitWords[1]):
                self.imu = imu("imu")
                self.imu.open()
                self.sensorList.append(self.imu)
            elif "output" in splitWords[0]:
                self.saveIMG = int(splitWords[1])
                self.saveCSV = int(splitWords[1])
                if self.saveCSV:
                    self.setDirectories()
                    radar_header = ["index", "time", "x", "y", "range", "peakVal", "doppler"]
                    camera_header = ["index", "time", "imageName"]
                    imu_header = ["index", "time", "x", "y", "z", "ax", "ay", "az", "gx", "gy", "gz"]
                    if self.left_radar:
                        self.leftRadarFile = open(self.dirPath+'/'+self.dirName+'left-radar.csv', 'w', newline='')
                        self.leftRadarWriter = csv.writer(self.leftRadarFile)
                        self.leftRadarWriter.writerow(radar_header)
                    if self.right_radar:
                        self.rightRadarFile = open(self.dirPath+'/'+self.dirName+'right-radar.csv', 'w', newline='')
                        self.rightRadarWriter = csv.writer(self.rightRadarFile)
                        self.rightRadarWriter.writerow(radar_header)
                    if self.left_camera:
                        self.leftCameraFile = open(self.dirPath+'/'+self.dirName+'left-camera.csv', 'w', newline='')
                        self.leftCameraWriter = csv.writer(self.leftCameraFile)
                        self.leftCameraWriter.writerow(camera_header)
                    if self.right_camera:
                        self.rightCameraFile = open(self.dirPath+'/'+self.dirName+'right-camera.csv', 'w', newline='')
                        self.rightCameraWriter = csv.writer(self.rightCameraFile)
                        self.rightCameraWriter.writerow(camera_header)
                    if self.imu:
                        self.imuFile = open(self.dirPath+'/'+self.dirName+'imu.csv', 'w', newline='')
                        self.imuWriter = csv.writer(self.imuFile)
                        self.imuWriter.writerow(imu_header)
                    self.saveIMG = self.addProcessor('saving', target=self.imgWriter, args=(self.imgBuffer,), num_of_cores=4)
                self.display = int(splitWords[2])
                if self.display:
                    self.displayMethod = 'scatter' if int(splitWords[3]) else 'heatmap'
                self.zip = int(splitWords[4])
                self.ip_address = int(splitWords[5])
            elif "sampling" in splitWords[0] and int(splitWords[1]):
                self.sampling = float(splitWords[2])

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
        if self.saveIMG:
            self.saveIMG.running.value = 0
        self.saveIMG = None
        if self.leftRadarFile:
            self.leftRadarFile.close()
        if self.rightRadarFile:
            self.rightRadarFile.close()
        if self.leftCameraFile:
            self.leftCameraFile.close()
        if self.rightCameraFile:
            self.rightCameraFile.close()
        if self.imuFile:
            self.imuFile.close()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        if self.zip:
            os.chdir('..')
            shutil.make_archive(self.dirName, 'zip', self.dirName)
        if self.ip_address:
            print(f"scp jetson@{get_ip()}:{os.getcwd()}/{self.dirName}.zip ~")



    def loopCycleControl(self, start_time, sleep_enable=1):
        if sleep_enable:
            if time.time()-start_time < 1/self.sampling:
                time.sleep(1/self.sampling +
                           start_time -time.time())
                print(f"\r[INFO] Actual rate:{int(1/(time.time()-start_time))} - Buffer length: {self.imgBuffer.qsize()}              ", end="")
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
            tempObj = self.objCreate(
                time_stamp,
                self.index,
                data,
                grabbed,
                sensor.sensorID)
            self.append(tempObj)

    def csvWriter(self):
        try:
            while True:
                tempObj = self.csvBuffer.pop(0)
                if 'radar' in tempObj.source:
                    writer = self.leftRadarWriter if 'left' in tempObj.source else self.rightRadarWriter
                    if tempObj.data_is_ok:
                        writer.writerow([
                            tempObj.index,
                            tempObj.time,
                            tempObj.data["x"],
                            tempObj.data["y"],
                            tempObj.data["peakVal"],
                            tempObj.data["doppler"],
                        ])
                    else:
                        writer.writerow([
                            tempObj.index,
                            tempObj.time,
                            "", "", "", ""])
                elif 'camera' in tempObj.source:
                    writer = self.leftCameraWriter if 'left' in tempObj.source else self.rightCameraWriter
                    filename = "Figure-left-camera/img_left_%d.jpg" % tempObj.index if 'left' in tempObj.source else "Figure-right-camera/img_right_%d.jpg" % tempObj.index
                    if tempObj.data_is_ok:
                        writer.writerow([
                            tempObj.index,
                            tempObj.time,
                            filename
                        ])
                    else:
                        writer.writerow([
                            tempObj.index,
                            tempObj.time,
                            "",
                        ])
                elif 'imu' in tempObj.source:
                    writer = self.imuWriter
                    if tempObj.data_is_ok:
                        writer.writerow([
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
                        writer.writerow([
                            tempObj.index,
                            tempObj.time,
                            "", "", "", "", "", "", "", "", "",
                        ])
        except IndexError:
            pass
        except KeyboardInterrupt:
            pass

    def imgWriter(self, data_buffer):
        while self.saveIMG.running.value or not data_buffer.empty():
            if not self.saveIMG.running.value:
                print(f'\r[INFO] Remained images to save : {data_buffer.qsize()}', end='')
            try:
                tempObj = data_buffer.get(0)
                if tempObj.data_is_ok:
                    filename = f"Figure-left-camera/img_left_{tempObj.index}.jpg" if 'left' in tempObj.source else f"Figure-right-camera/img_right_{tempObj.index}.jpg"
                    cv2.imwrite(filename, tempObj.data)
            except RuntimeError:
                print('[ERROR] Could not save an image')
            except queue.Empty:
                pass
        else:
            print(f"[INFO] process id: {os.getpid()} has joined\n")

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
                                    (self.blankImgShape[1], self.blankImgShape[0]),
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
                                     (self.blankImgShape[1], self.blankImgShape[0]),
                                     interpolation=cv2.INTER_AREA)
        else:
            right_radar = self.blankImg
        camera_images = np.hstack((left_image, right_image))
        radar_images = np.hstack((left_radar, right_radar))
        final_image = np.vstack((camera_images, radar_images))
        cv2.imshow(window_title, cv2.resize(final_image, (800,600)))
        self.loopCycleControl(start_time, sleep_enable=0)

    def run(self):
        if self.saveIMG:
            with DelayedKeyboardInterrupt():
                self.saveIMG.start()
        for sensor in self.sensorList:
            sensor.start()
            print(sensor, sensor.sensorID, sensor.isOpened)
        if self.sensorList \
                and all([sensor.isOpened for sensor in self.sensorList]):
            print('[INFO] Recording...')
            self.is_running = True
            if not self.display:
                while self.is_running:
                    try:
                        start_time = time.time()
                        self.readAll()
                        self.csvWriter()
                        self.loopCycleControl(start_time)
                        self.index += 1
                    except KeyboardInterrupt:
                        self.is_running = False
                        pass
                else:
                    print("\n Capture disrupted")
                    print(f"[INFO] Estimated buffer size :{self.imgBuffer.qsize()}\n", end="")
                    self.closeAll()
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
                    print("\n Capture disrupted")
                    time.sleep(3)
                    self.closeAll()
                    cv2.destroyAllWindows()
                    time.sleep(3)
        else:
            print("[ERROR] Not all sensors have been opened correctly")
            self.closeAll()

    def getCamPars(self, path_to_calib_xml, path_to_depth_xml):
        cv_file = cv2.FileStorage(path_to_calib_xml, cv2.FILE_STORAGE_READ)
        self.camPars.Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
        self.camPars.Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
        self.camPars.Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
        self.camPars.Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
        cv_file.release()
        cv_file = cv2.FileStorage(path_to_depth_xml, cv2.FILE_STORAGE_READ)
        self.camPars.numDisparities = int(cv_file.getNode("numDisparities").real())
        self.camPars.blockSize = int(cv_file.getNode("blockSize").real())
        self.camPars.preFilterType = int(cv_file.getNode("preFilterType").real())
        self.camPars.preFilterSize = int(cv_file.getNode("preFilterSize").real())
        self.camPars.preFilterCap = int(cv_file.getNode("preFilterCap").real())
        self.camPars.textureThreshold = int(cv_file.getNode("textureThreshold").real())
        self.camPars.uniquenessRatio = int(cv_file.getNode("uniquenessRatio").real())
        self.camPars.speckleRange = int(cv_file.getNode("speckleRange").real())
        self.camPars.speckleWindowSize = int(cv_file.getNode("speckleWindowSize").real())
        self.camPars.disp12MaxDiff = int(cv_file.getNode("disp12MaxDiff").real())
        self.camPars.minDisparity = int(cv_file.getNode("minDisparity").real())
        cv_file.release()
        self.stereo = cv2.StereoBM_create()
        self.stereo.setNumDisparities(self.camPars.numDisparities)
        self.stereo.setBlockSize(self.camPars.blockSize)
        self.stereo.setPreFilterType(self.camPars.preFilterType)
        self.stereo.setPreFilterSize(self.camPars.preFilterSize)
        self.stereo.setPreFilterCap(self.camPars.preFilterCap)
        self.stereo.setTextureThreshold(self.camPars.textureThreshold)
        self.stereo.setUniquenessRatio(self.camPars.uniquenessRatio)
        self.stereo.setSpeckleRange(self.camPars.speckleRange)
        self.stereo.setSpeckleWindowSize(self.camPars.speckleWindowSize)
        self.stereo.setDisp12MaxDiff(self.camPars.disp12MaxDiff)
        self.stereo.setMinDisparity(self.camPars.minDisparity)

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
                                 (self.blankImgShape[1], self.blankImgShape[0]),
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
                                 (self.blankImgShape[1], self.blankImgShape[0]),
                                 interpolation=cv2.INTER_AREA)
                with self.right_radar.postprocess.read_lock:
                    self.right_radar_output = img # post porcessed
                self.right_radar.postprocess.processed = True
                self.right_radar.postprocess.new_data = False

if __name__ == "__main__":
    target = sensor_read("dacProfile.cfg")
    target.setup()
    target.run()

