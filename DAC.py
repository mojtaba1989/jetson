import copy
import logging
import multiprocessing
import os
import queue
import shutil
import signal
import sys
import time
from multiprocessing import Queue, Value, Array
from time import strftime, localtime
import threading

import cv2
# import matplotlib.pyplot as plt
import numpy as np


from awr1642driver import awr1642
from icm20948driver import IMU as imu
from imx21983driver import CSI_Camera, gstreamer_pipeline
from exceptions import *

class HiddenPrints:
    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout.close()
        sys.stdout = self._original_stdout

def blockPrinting(func):
    def func_wrapper(*args, **kwargs):
        # block all printing to the console
        sys.stdout = open(os.devnull, 'w')
        # call the method in question
        value = func(*args, **kwargs)
        # enable all printing to the console
        sys.stdout = sys.__stdout__
        # pass the return value of the method back
        return value

    return func_wrapper


def get_ip(broadcast=False):
    import subprocess
    lst_txt = subprocess.check_output('ifconfig', shell=True).decode("utf-8").splitlines()
    try:
        while True:
            temp = lst_txt.pop(0)
            if 'wlan' in temp:
                while True:
                    temp = lst_txt.pop(0)
                    if 'inet' in temp:
                        temp = temp[temp.find('inet'):].split(' ')
                        while True:
                            ip = temp.pop(0)
                            if '.' in ip:
                                raise StopIteration
    except StopIteration:
        if broadcast:
            ip = ip.split('.')
            ip[-1] = '255'
            ip = '.'.join(ip)
        return ip

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
        self.maxWait = None
        self.is_running = False

        # sensor objects
        self.sensorList = []

        # display setup
        self.display = 0
        self.displayMethod = 'heatmap'
        self.blankImg = cv2.imread('NoSignal.jpg', cv2.IMREAD_UNCHANGED)
        self.blankImgShape = self.blankImg.shape

        # save method setup
        self.saveCSV = 0
        self.saveIMG = 0
        self.index = 0
        self.zip = None

        # image postprocessing
        self.camPars = None

        # Multiprocessor lists:
        self.multiProcessorList = []
        self.multiThreadingList = []
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
        def __init__(self, name=None, target_function=None, args=None, num_of_cores=2):
            self.name           = name
            self.running        = Value('i', 0)
            self.buffer         = Queue()
            self.target         = target_function
            self.args           = args
            self.num_of_cores   = num_of_cores
            self.jobs           = []

        def start(self):
            self.running.value = 1
            for i in range(self.num_of_cores):
                process = multiprocessing.Process(target=self.target, args=self.args)
                process.daemon = True
                self.jobs.append(process)
            for job in self.jobs:
                job.start()
            print(f'[INFO] Parallel {self.name} status: {bool(self.running)}')

        def stop(self):
            self.running.value = 0
            for job in self.jobs:
                if job.is_alive():
                    job.join()


    class addThread:
        def __init__(self, name, target):
            self.read_thread    = None
            self.read_lock      = threading.Lock()
            self.name           = name
            self.running        = False
            self.target         = target

        def start(self):
            self.running = True
            self.read_thread = threading.Thread(target=self.target)
            print(f'[INFO] Parallel {self.name} status: {self.running}')
            self.read_thread.start()

        def stop(self):
            self.running = False
            self.read_thread.join()
            self.read_thread = None

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

    def setDirectories(self, image=False):
        try:
            self.dirName = strftime("%d-%b-%Y-%H-%M", localtime())
            os.makedirs(self.dirName)
            self.dirPath += '/' + self.dirName
            os.chdir(self.dirPath)
            if image:
                for sensor in self.sensorList:
                    if 'camera' in sensor.sensorID:
                        os.makedirs(f'{sensor.sensorID}')
        except OSError:
            self.closeAll()
            sys.exit('[Error] Cannot make directories')

    @blockPrinting
    def addCamera(self, config):
        if int(config[1]):
            sensor_id = f'camera{int(config[2])}'
            sensor_obj = CSI_Camera(sensor_id)
            with HiddenPrints():
                sensor_obj.open(
                    gstreamer_pipeline(
                        sensor_id=int(config[3]),
                        capture_width=int(config[4]),
                        capture_height=int(config[5]),
                        display_width=int(config[6]),
                        display_height=int(config[7]),
                        framerate=self.sampling if self.sampling else int(config[8]),
                        flip_method=int(config[9]),
                    )
                )
            return sensor_obj
        else:
            return None

    def addRadar(self, config):
        if int(config[1]):
            sensor_id = f'radar{int(config[2])}'
            sensor_obj = awr1642(
                config[5],
                config[3],
                config[4],
                sensor_id=sensor_id
            )
            sensor_obj.sensorSetup()
            time.sleep(.01)
            sensor_obj.open()
            time.sleep(.01)
            return sensor_obj
        else:
            return None

    def addIMU(self, config):
        if int(config[1]):
            sensor_id = f'imu{int(config[2])}'
            sensor_obj = imu(sensor_id=sensor_id)
            sensor_obj.open()
            return sensor_obj
        else:
            return None

    def setup(self, demo_visualizer=False):
        # refresh nvargus port
        try:
            os.system('sudo service nvargus-daemon stop')
            os.system('sudo service nvargus-daemon start')
        except:
            pass
        # register sensors
        for i in self.config:
            splitWords = i.split(" ")
            if "camera" in splitWords[0]:
                temp = self.addCamera(splitWords)
                if temp:
                    self.sensorList.append(temp)
                    self.blankImg = cv2.resize(
                        self.blankImg,
                        (int(splitWords[6]),
                         int(splitWords[7])),
                        interpolation=cv2.INTER_AREA
                    )
                    self.blankImgShape = (self.blankImg.shape[1], self.blankImg.shape[0])
                    temp.blankImg = self.blankImg
            elif "radar" in splitWords[0] and not demo_visualizer:
                temp = self.addRadar(splitWords)
                if temp:
                    self.sensorList.append(temp)
            elif "imu" in splitWords[0] and not demo_visualizer:
                temp = self.addIMU(splitWords)
                if temp:
                    self.sensorList.append(temp)
            elif "output" in splitWords[0] and not demo_visualizer:
                if int(splitWords[1]):
                    self.setDirectories(image=int(splitWords[3]))
                    for sensor in self.sensorList:
                        sensor.logObj = self.addProcessor()
                        sensor.logObj.name = f'logger_dev_{sensor.sensorID}'
                        sensor.logObj.target = sensor.logP
                        sensor.logObj.args = (sensor.logObj.buffer, )
                        sensor.logObj.num_of_cores = 1
                        self.multiProcessorList.append(sensor.logObj)
                        if 'camera' in sensor.sensorID:
                            processor_name = f'img_writer_dev_{sensor.sensorID}'
                            if int(splitWords[3]):
                                sensor.writerObj = self.addProcessor()
                                sensor.writerObj.name = processor_name
                                sensor.writerObj.target = sensor.image_writer
                                sensor.writerObj.args = (sensor.writerObj.buffer, )
                                sensor.writerObj.num_of_cores = int(splitWords[4])
                            else:
                                cap_size = Array('i', self.blankImgShape)
                                sensor.writerObj = self.addProcessor()
                                sensor.writerObj.name = processor_name
                                sensor.writerObj.target = sensor.video_writer
                                sensor.writerObj.args = (sensor.writerObj.buffer, cap_size)
                                sensor.writerObj.num_of_cores = int(splitWords[4])
                            self.multiProcessorList.append(sensor.writerObj)
                    self.zip = int(splitWords[2])
            elif "sampling" in splitWords[0] and int(splitWords[1]):
                if not demo_visualizer:
                    self.sampling = float(splitWords[2])
                    self.maxWait = 1/self.sampling
                else:
                    self.sampling = 30
                    self.maxWait = 1/self.sampling

    def closeAll(self):
        for sensor in self.sensorList:
            sensor.stop()
        for thread in self.multiThreadingList:
            thread.stop()
        for processor in self.multiProcessorList:
            processor.stop()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        if self.zip:
            print('[Waiting] Zipping ....')
            os.chdir('..')
            shutil.make_archive(self.dirName, 'zip', self.dirName)
            print('[OK] Zip file is ready')
            print('[INFO] Command to download the zip file via SSH: (replace ~ with destination)')
            print(f"scp jetson@{get_ip()}:{os.getcwd()}/{self.dirName}.zip ~")

    def loopCycleControl(self, start_time, sleep_enable=1):
        dT = time.time()-start_time
        if sleep_enable:
            if dT < self.maxWait:
                time.sleep(self.maxWait-dT)
            buffer_size = sum([p.args[0].qsize() for p in self.multiProcessorList])
            print(f"\r[INFO] Actual rate:{min([1/dT, self.sampling]):5.2f} - Buffer length: {buffer_size:03d}", end="")
            if buffer_size > 200:
                raise LargeBuffer
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
            sensor.logObj.buffer.put(tempObj)
            if 'camera' in sensor.sensorID:
                sensor.writerObj.buffer.put(copy.copy(tempObj))

    def csvWriter(self):
        while self.is_running:
            try:
                for sensor in self.sensorList:
                    with self.saveCSV.read_lock:
                        sensor.write(sensor.output_buffer.pop(0))
            except IndexError:
                pass
        else:
            print(f"[OK] Thread id: {self.saveCSV.name} has joined")



    def imgWriter(self, data_buffer):
        while self.saveIMG.running.value or not data_buffer.empty():
            try:
                tempObj = data_buffer.get(0)
                if tempObj.data_is_ok:
                    filename = f"Image-{tempObj.source}/img_{tempObj.source}_{tempObj.index}.jpg"
                    cv2.imwrite(filename, tempObj.data)
            except RuntimeError:
                print('[ERROR] Could not save an video')
            except queue.Empty:
                pass
        else:
            print(f"[OK] process id: {os.getpid()} has joined")

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
                plt.stop()
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
                plt.stop()
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
        self.is_running = True
        if all([sensor.sensorIsReady for sensor in self.sensorList]):
            with DelayedKeyboardInterrupt():
                for processor in self.multiProcessorList:
                    processor.start()
            for thread in self.multiThreadingList:
                thread.start()
            for sensor in self.sensorList:
                sensor.start()
                print(sensor, sensor.sensorID, sensor.isOpened)
            if self.sensorList \
                    and all([sensor.isOpened for sensor in self.sensorList]):
                print('[INFO] Recording...')
                while self.is_running:
                    try:
                        start_time = time.time()
                        self.readAll()
                        self.loopCycleControl(start_time)
                        self.index += 1
                    except KeyboardInterrupt:
                        self.is_running = False
                        pass
                    except LargeBuffer:
                        pass
                else:
                    print("\nCapture disrupted")
                    buffersize = sum([p.args[0].qsize() for p in self.multiProcessorList])
                    print(f"[INFO] Estimated buffer size :{buffersize}")
                    self.closeAll()
            else:
                print("[ERROR] Not all sensors have been opened correctly")
                self.closeAll()
        else:
            print("[ERROR] Not all sensors have been opened correctly")
            for sensor in self.sensorList:
                print(sensor, sensor.sensorID, sensor.sensorIsReady)

    def run_demo(self):
        self.camPars = self.stereoCalib()
        self.getCamPars('params_py.xml', 'depth_pars.xml')
        (width, height) = (480, 270)
        self.is_running = True
        if all([sensor.sensorIsReady for sensor in self.sensorList]):
            with DelayedKeyboardInterrupt():
                for processor in self.multiProcessorList:
                    processor.start()
            for thread in self.multiThreadingList:
                thread.start()
            for sensor in self.sensorList:
                sensor.start()
                print(sensor, sensor.sensorID, sensor.isOpened)
            gst_out = f"appsrc ! video/x-raw, format=BGR ! queue ! videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! omxh264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! queue ! application/x-rtp, media=video, encoding-name=H264 ! udpsink host={get_ip(broadcast=True)} port=5000"
            print(f'[INFO] GSTREAMER {gst_out}')
            streamer = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, float(25), (2 * width, height))
            print('[INFO] Socket has been created...')
            if self.sensorList \
                    and all([sensor.isOpened for sensor in self.sensorList]):
                print('[INFO] Streaming...')
                frame = cv2.resize(self.blankImg, (width, height))
                while self.is_running:
                    tstart = time.time()
                    try:
                        for sensor in self.sensorList:
                            if 'camera0' in sensor.sensorID:
                                grabbed0, data0, time_stamp = sensor.read()
                                if not grabbed0:
                                    data0 = self.blankImg
                            elif 'camera1' in sensor.sensorID:
                                grabbed1, data1, time_stamp = sensor.read()
                                if not grabbed1:
                                    data1 = self.blankImg
                        frame = np.hstack((data0, data1))
                        streamer.write(cv2.resize(frame, (2 * width, height)))
                        time.sleep(max([0, 1/25-time.time()+tstart]))
                    except KeyboardInterrupt:
                        self.is_running = False
                        streamer.release()
                        pass
                else:
                    print("\nStreaming Interrupt")
                    self.closeAll()
            else:
                print("[ERROR] Not all sensors have been opened correctly")
                self.closeAll()
        else:
            print("[ERROR] Not all sensors have been opened correctly")
            for sensor in self.sensorList:
                print(sensor, sensor.sensorID, sensor.sensorIsReady)

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

if __name__ == "__main__":
    demo = False
    for idx, arg in enumerate(sys.argv):
        if arg in ['--demo', '-d']:
            demo = True
    target = sensor_read("dacProfile.cfg")
    if not demo:
        target.setup()
        target.run()
    else:
        target.setup(demo_visualizer=True)
        target.run_demo()



