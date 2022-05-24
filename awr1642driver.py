import os
import platform
import struct
import threading
import time

import numpy as np
import serial
import csv
import queue

def pow2roundup(x):
    y = 1
    while x > y:
        y *= 2
    return y

def dict_update(dict_main, dict_new):
    try:
        if dict_new:
            for key in dict_main.keys():
                dict_main[key] += dict_new[key]
            return dict_main
    except (KeyError, NameError):
        pass
def flush_det_obj():
    out_dict = {'x': [], 'y': [], 'peakVal': [], 'doppler': []}
    return out_dict

class awr1642:
    def __init__(self, configFileName, CLIport_num, Dataport_num, sensor_id=None):
        self.CLIportNum  = CLIport_num
        self.DataportNum = Dataport_num
        self.CLIport = {}
        self.Dataport = {}
        self.configFileName = configFileName
        self.configParameters = {}
        self.magicOK = 0
        self.dataOK = 0
        self.frameNumber = 0
        self.detObj = flush_det_obj()
        self.config = [line.rstrip('\r\n') for line in open(self.configFileName)]
        self.byteBuffer = bytes([])
        self.byteBufferLength = 0
        self.magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
        self.magicWordB = bytearray(self.magicWord)
        self.MMWDEMO_UART_MSG_DETECTED_POINTS = 1
        self.running = False
        self.failedSample = 0
        self.sensorIsReady = False
        self.sensorID = sensor_id
        self.isOpened = False
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.postprocess = None
        self.logger = None
        self.logObj = None
        self.file_header = ["index", "time", "x", "y", "peakVal", "doppler"]
        self.log_buffer = []

    def sendConfig(self):
        for i in self.config:
            if not i == 'sensorStart':
                self.CLIport.write((i + '\n').encode())
            time.sleep(0.01)

    def serrialConfig(self):
        self.CLIport  = {}
        self.Dataport = {}

        if platform.system() == "Windows":
            self.CLIport  = serial.Serial('COM' + self.CLIportNum, 115200, timeout=1)
            self.Dataport = serial.Serial('COM' + self.DataportNum, 921600, timeout=10)

        elif platform.system() == "Darwin":
            self.CLIport = serial.Serial('/dev/tty.usbmodem' + self.CLIportNum, 115200)
            self.Dataport = serial.Serial('/dev/tty.usbmodem' + self.DataportNum, 921600, timeout=1)
        elif platform.system() == "Linux":
            try:
                self.CLIport = serial.Serial('/dev/tty' + self.CLIportNum, 115200)
                self.Dataport = serial.Serial('/dev/tty' + self.DataportNum, 921600, timeout=1)
            except PermissionError:
                os.system('sudo chmod 766 /dev/tty' + self.CLIportNum)
                os.system('sudo chmod 766 /dev/tty' + self.DataportNum)
                print('salam')
                self.CLIport = serial.Serial('/dev/tty' + self.CLIportNum, 115200)
                self.Dataport = serial.Serial('/dev/tty' + self.DataportNum, 921600, timeout=1)

    def parseConfigFile(self):
        # Initialize an empty dictionary to store the configuration parameters
        for i in self.config:
            splitWords = i.split(" ")

            # Hard code the number of antennas, change if other configuration is used
            numTxAnt = 2

            # Get the information about the profile configuration
            if "profileCfg" in splitWords[0]:
                startFreq = int(splitWords[2])
                idleTime = int(splitWords[3])
                rampEndTime = float(splitWords[5])
                freqSlopeConst = int(splitWords[8])
                numAdcSamples = int(splitWords[10])
                numAdcSamplesRoundTo2 = 1

                while numAdcSamples > numAdcSamplesRoundTo2:
                    numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2

                digOutSampleRate = int(splitWords[11])

            # Get the information about the frame configuration
            elif "frameCfg" in splitWords[0]:

                chirpStartIdx = int(splitWords[1])
                chirpEndIdx = int(splitWords[2])
                numLoops = int(splitWords[3])
                numFrames = int(splitWords[4])
                framePeriodicity = float(splitWords[5])

        # Combine the read data to obtain the configuration parameters
        numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
        self.configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
        self.configParameters["numRangeBins"] = numAdcSamplesRoundTo2
        self.configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * numAdcSamples)
        self.configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
                2 * freqSlopeConst * 1e12 * self.configParameters["numRangeBins"])
        self.configParameters["dopplerResolutionMps"] = 3e8 / (
                2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * self.configParameters["numDopplerBins"] * numTxAnt)
        self.configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
        self.configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
        self.configParameters["framePeriodicity"] = framePeriodicity

    def fastParseData(self):
        magicOK = 0  # Checks if magic number has been read
        dataOK = 0  # Checks if the data has been read correctly
        detObj = flush_det_obj()
        if self.failedSample > 5:
            with self.read_lock:
                self.CLIport.write(('sensorStop\n').encode())
                self.Dataport.reset_output_buffer()
                self.Dataport.reset_input_buffer()
                time.sleep(.01)
                self.CLIport.write(('sensorStart\n').encode())
                self.failedSample = 0
                time.sleep(.01)
        else:
            readBuffer = self.Dataport.read(self.Dataport.in_waiting)
            self.byteBuffer += readBuffer

            if len(self.byteBuffer) > 16:
                loc = self.byteBuffer.find(self.magicWordB)
                if loc >= 0:
                    self.byteBuffer = self.byteBuffer[loc:]
                    self.byteBufferLength = len(self.byteBuffer)
                    totalPacketLen = struct.unpack('<I', self.byteBuffer[12:16])[0]
                    if self.byteBufferLength and self.byteBufferLength >= totalPacketLen:
                        magicOK = 1

            if magicOK:
                version, totalPacketLen, platForm, frameNumber, timeCpuCycles, numpDetectObj, numTLVs, subFrameNumber = struct.unpack('<8I', self.byteBuffer[8:40])
                version = format(version, 'x')
                platForm = format(platForm, 'x')
                idX = 40
                for tlvIdx in range(numTLVs):
                    tlv_type, tlv_length = struct.unpack("<2I", self.byteBuffer[idX:idX+8])
                    idX += 8
                    if tlv_type == self.MMWDEMO_UART_MSG_DETECTED_POINTS:
                        tlv_numObj, tlv_xyzQFormat = struct.unpack("<2h", self.byteBuffer[idX:idX+4])
                        idX += 4
                        tlv_xyzQFormat = 1/(2 ** tlv_xyzQFormat)
                        dopplerIdx = np.zeros(tlv_numObj, dtype='int16')
                        peakVal = np.zeros(tlv_numObj, dtype='int16')
                        x = np.zeros(tlv_numObj, dtype='int16')
                        y = np.zeros(tlv_numObj, dtype='int16')
                        for odX in range(tlv_numObj):
                            rangeIdx, dopplerIdx[odX], peakVal[odX], x[odX], y[odX], z = struct.unpack("<Hhhhhh", self.byteBuffer[idX:idX+12])
                            idX += 12
                        dopplerIdx[dopplerIdx > 32767] = dopplerIdx[dopplerIdx > 32767] - 65535
                        x[x > 32767] = x[x > 32767] - 65535
                        y[y > 32767] = y[y > 32767] - 65535
                        dopplerVal = dopplerIdx * tlv_xyzQFormat
                        x = x * tlv_xyzQFormat
                        y = y * tlv_xyzQFormat
                        detObj = {
                            "x": list(x.round(decimals=2)),
                            "y": list(y.round(decimals=2)),
                            "peakVal": list(peakVal),
                            "doppler": list(dopplerVal.round(decimals=2)),
                        }
                        dataOK = 1
                    else:
                        idX += tlv_length
                if dataOK:
                    self.byteBuffer = self.byteBuffer[totalPacketLen:]
                    self.byteBufferLength = len(self.byteBuffer)
        return dataOK, detObj

    def stop(self):
        self.running = False
        self.CLIport.write(('sensorStop\n').encode())
        time.sleep(0.01)
        self.CLIport.close()
        time.sleep(0.01)
        self.Dataport.close()
        time.sleep(0.01)
        self.CLIport = {}
        self.Dataport = {}
        self.read_thread.join()
        self.read_thread = None
        print(f'[OK]{self.sensorID} is closed')

    def sensorSetup(self):
        self.serrialConfig()
        self.sendConfig()
        time.sleep(.01)

    def open(self, max_failed=4):
        self.byteBufferLength = 0
        self.CLIport.write(('sensorStart\n').encode())
        self.failedSample = 0
        tries = 5
        try:
            for i in range(tries):
                loopStartTime = time.time()
                self.dataOK, self.detObj = self.fastParseData()
                if not self.dataOK:
                    self.failedSample += 1
                time.sleep(.03 - (time.time() - loopStartTime))
            self.CLIport.write(('sensorStop\n').encode())
            print(f"[INFO] Sensor {self.CLIportNum} test result: Failed rate ({self.failedSample} out of {tries})")
            if self.failedSample > max_failed:
                print("[ERROR] Link is unstable")
            else:
                self.sensorIsReady = True
                print ("[INFO] Sensor is ready - Port:" + self.CLIportNum)
        except KeyboardInterrupt:
            self.CLIport.write(('sensorStop\n').encode())
            print("Unable to open radar - Port:" + self.CLIportNum)

    def start(self):
        if self.running:
            print('Radar streaming is already running')
            return None
        if self.sensorIsReady:
            self.Dataport.reset_output_buffer()
            self.Dataport.reset_input_buffer()
            self.byteBufferLength = 0
            self.CLIport.write(('sensorStart\n').encode())
            time.sleep(.01)
            self.running = True
            self.isOpened = True
            self.failedSample = 0
            self.read_thread = threading.Thread(target=self.update)
            self.read_thread.start()
        return self

    def update(self):
        while self.running:
            try:
                dataOK, detObj = self.fastParseData()
                with self.read_lock:
                    if dataOK:
                        self.dataOK = dataOK
                        self.detObj = dict_update(self.detObj, detObj)
            except RuntimeError:
                print("Could not read point cloud from radar")

    def read(self):
        with self.read_lock:
            time_stamp = time.time()
            dataOK = self.dataOK
            detObj = self.detObj
            self.failedSample = 0 if dataOK else self.failedSample + 1
            self.dataOK = 0
            self.detObj = flush_det_obj()
        return dataOK, detObj, time_stamp

    def setDetectionThreashold(self, new_threshold):
        idx = [i for i, s in enumerate(self.config) if "cfarCfg -1 0" in s][0]
        temp = self.config[idx].split(" ")
        temp[8] = str(int(new_threshold))
        self.config[idx] = ' '.join(temp)
        time.sleep(.005)
        self.CLIport.write((self.config[idx] + '\n').encode())
        print(self.config[idx])

    def setNoiseAveragingWindow(self, numSamples):
        numSamplesRound2 = 1
        while numSamplesRound2 < numSamples:
            numSamplesRound2 = numSamplesRound2 * 2

        idx = [i for i, s in enumerate(self.config) if "cfarCfg -1 0" in s][0]
        temp = self.config[idx].split(" ")
        temp[4] = str(numSamplesRound2)
        temp[5] = str(int(numSamplesRound2/2))
        temp[6] = str(np.log2(numSamplesRound2).astype(int) + 1)
        self.config[idx] = ' '.join(temp)
        time.sleep(.01)
        self.CLIport.write((self.config[idx] + '\n').encode())
        print(self.config[idx])

    def setRangePeakGrouping(self, state='enable'):
        idx = [i for i, s in enumerate(self.config) if "peakGrouping" in s][0]
        temp = self.config[idx].split(" ")
        if state in ['enable', 'e']:
            temp[3] = str(int(1))
        else:
            temp[3] = str(int(0))
        self.config[idx] = ' '.join(temp)
        self.CLIport.write((self.config[idx] + '\n').encode())
        time.sleep(0.01)
        print(self.config[idx])

    def setMaxRange(self, range):
        self.CLIport.write(('sensorStop\n').encode())
        time.sleep(0.01)
        print('sensorStop\n')
        idx = [i for i, s in enumerate(self.config) if "frameCfg" in s][0]
        temp = self.config[idx].split(" ")
        frameTime = np.min([float(temp[5]) * 1e3, 50e3])
        idx = [i for i, s in enumerate(self.config) if "profileCfg" in s][0]
        temp = self.config[idx].split(" ")
        freqSlopeConst   = int(np.max([20, np.ceil(240/range/5) * 5]))
        rampEndTime      = np.round(4000 / freqSlopeConst, decimals=2)
        adcStartTime     = int(temp[4])
        txStartTime      = int(temp[9])
        idleTime         = int(np.round(frameTime/64-rampEndTime-adcStartTime-txStartTime, decimals=0))
        adcSamplingTime  = rampEndTime - adcStartTime - txStartTime
        digOutSampleRate = int(np.round(range * 2 * freqSlopeConst / .24, decimals=0))
        numAdcSamples    = int(np.floor(digOutSampleRate * adcSamplingTime / 4e3) * 4)
        temp[3]  = str(idleTime)
        temp[5]  = str(rampEndTime)
        temp[8]  = str(freqSlopeConst)
        temp[10] = str(numAdcSamples)
        temp[11] = str(digOutSampleRate)
        self.config[idx] = ' '.join(temp)
        self.CLIport.write((self.config[idx] + '\n').encode())
        print(self.config[idx])
        self.parseConfigFile()

    def setSampleRate(self, rate):
        self.CLIport.write(('sensorStop\n').encode())
        time.sleep(0.01)
        print('sensorStop\n')
        idx = [i for i, s in enumerate(self.config) if "frameCfg" in s][0]
        temp = self.config[idx].split(" ")
        temp[5] = str(np.round(1e3 / rate, decimals=2))
        self.configParameters["framePeriodicity"] = float(temp[5])
        self.config[idx] = ' '.join(temp)
        self.CLIport.write((self.config[idx] + '\n').encode())
        print(self.config[idx])

    def optimize(self, range_interval, eval_range):
        detection_range = np.linspace(range_interval[0], range_interval[1], 10)
        score = np.zeros(10)

        for i in range(10):
            self.byteBuffer = np.zeros(2 ** 15, dtype='uint8')
            self.byteBufferLength = 0
            self.setMaxRange(detection_range[i])
            time.sleep(.01)
            self.CLIport.write(('sensorStart\n').encode())
            time.sleep(.01)
            n = 90
            radar = np.zeros((n, n, 20))
            for itr in range(20):
                loopStartTime = time.time()
                self.update()
                while not self.dataOK and time.time() - loopStartTime > self.configParameters[
                    "framePeriodicity"] / 1000:
                    self.update()
                    time.sleep(.01)
                if not self.dataOK:
                    self.Dataport.reset_output_buffer()
                    self.Dataport.reset_input_buffer()
                    self.byteBuffer = np.zeros(2 ** 15, dtype='uint8')
                    self.byteBufferLength = 0
                    time.sleep(.01)
                else:
                    print(self.dataOK, self.detObj)
                    radar = self.heat_map(radar, self.detObj["x"], self.detObj["y"], self.detObj["peakVal"],
                                          [-eval_range, eval_range], [0, eval_range], xbinnum=n, ybinnum=n)
                time.sleep(np.max([0,
                                   self.configParameters["framePeriodicity"] / 1000 - (time.time() - loopStartTime)]))
            score[i] = np.mean(np.mean(np.mean(radar)))
            self.Dataport.reset_output_buffer()
            self.Dataport.reset_input_buffer()
            self.CLIport.write(('sensorStop\n').encode())
            time.sleep(.01)
        print('range', detection_range, 'score', score)

    def run(self, end=np.nan):
        self.Dataport.reset_output_buffer()
        self.Dataport.reset_input_buffer()
        self.byteBuffer = np.zeros(2 ** 15, dtype='uint8')
        self.byteBufferLength = 0
        self.CLIport.write(('sensorStart\n').encode())
        time.sleep(.01)
        try:
            itt = 0
            while True:
                if not np.isnan(end):
                    if itt <= end:
                        itt += 1
                    else:
                        break
                loopStartTime = time.time()
                self.update()
                while not self.dataOK and time.time()-loopStartTime > self.configParameters["framePeriodicity"]/1000:
                    self.update()
                    time.sleep(.01)
                if not self.dataOK:
                    self.Dataport.reset_output_buffer()
                    self.Dataport.reset_input_buffer()
                    self.byteBuffer = np.zeros(2 ** 15, dtype='uint8')
                    self.byteBufferLength = 0
                    time.sleep(.01)
                print(self.dataOK, self.detObj)
                time.sleep(self.configParameters["framePeriodicity"]/1000 - (time.time() - loopStartTime))
        except KeyboardInterrupt:
            self.Dataport.reset_output_buffer()
            self.Dataport.reset_input_buffer()
            self.CLIport.write(('sensorStop\n').encode())

    def heat_map(self, tabold, xr, yr, zr, xlim, ylim, xc=np.nan, yc=np.nan, xbinnum=100, ybinnum=100):

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

            tabold = np.append(tab.reshape(xbinnum, ybinnum, 1), tabold, axis=-1)
            tabold = np.delete(tabold, -1, axis=-1)

            return tabold
        except:
            pass

    def log(self, tempObj):
        if tempObj.data_is_ok:
            self.logger.writerow([
                tempObj.index,
                tempObj.time,
                tempObj.data["x"],
                tempObj.data["y"],
                tempObj.data["peakVal"],
                tempObj.data["doppler"],
            ])
        else:
            self.logger.writerow([
                tempObj.index,
                tempObj.time,
                "", "", "", ""])

    def logP(self, log_buffer):
        log_file = open(f'{os.getpid()}.csv', 'w', newline='')
        file_header = ["index", "time", "x", "y", "peakVal", "doppler"]
        logger = csv.writer(log_file)
        logger.writerow(file_header)
        while self.logObj.running.value or not log_buffer.empty():
            try:
                tempObj = log_buffer.get(0)
                data = list(tempObj.data.values()) if tempObj.data_is_ok else []
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


class awr1642SRR:
    def __init__(self, CLIport_num, Dataport_num, sensor_id=None):
        self.CLIportNum  = CLIport_num
        self.DataportNum = Dataport_num
        self.CLIport = {}
        self.Dataport = {}
        self.configParameters = {}
        self.magicOK = 0
        self.dataOK = 0
        self.frameNumber = 0
        self.detObj = {}
        self.byteBuffer = bytes([])
        self.byteBufferLength = 0
        self.magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
        self.magicWordB = bytearray(self.magicWord)
        self.MMWDEMO_UART_MSG_DETECTED_POINTS = 1
        self.MMWDEMO_UART_MSG_CLUSTERS = 2
        self.MMWDEMO_UART_MSG_TRACKED_OBJ = 3
        self.MMWDEMO_UART_MSG_PARKING_ASSIST = 4
        self.running = False
        self.failedSample = 0
        self.sensorIsReady = False
        self.sensorID = sensor_id
        self.isOpened = False
        self.read_thread = None
        self.monitor = None
        self.read_lock = threading.Lock()
        self.postprocess = None
        self.writer = None
        self.file = None
        self.file_header = ["index", "time", "x", "y", "peakVal", "doppler"]
        self.output_buffer = []

    def RSSConfig(self):
        self.CLIport.write(('sensorStop\n').encode())
        time.sleep(.01)
        self.CLIport.write(('advFrameCfg\n').encode())
        time.sleep(3)

    def serrialConfig(self):
        self.CLIport  = {}
        self.Dataport = {}

        if platform.system() == "Windows":
            self.CLIport  = serial.Serial('COM' + self.CLIportNum, 115200, timeout=1)
            self.Dataport = serial.Serial('COM' + self.DataportNum, 921600, timeout=10)

        elif platform.system() == "Darwin":
            self.CLIport = serial.Serial('/dev/tty.usbmodem' + self.CLIportNum, 115200)
            self.Dataport = serial.Serial('/dev/tty.usbmodem' + self.DataportNum, 921600, timeout=1)
        elif platform.system() == "Linux":
            try:
                self.CLIport = serial.Serial('/dev/tty' + self.CLIportNum, 115200)
                self.Dataport = serial.Serial('/dev/tty' + self.DataportNum, 921600, timeout=1)
            except:
                os.system('sudo chmod 666 /dev/tty' + self.CLIportNum)
                os.system('sudo chmod 666 /dev/tty' + self.DataportNum)
                self.CLIport = serial.Serial('/dev/tty' + self.CLIportNum, 115200)
                self.Dataport = serial.Serial('/dev/tty' + self.DataportNum, 921600, timeout=1)

    def SRRParseData(self):
        magicOK = 0  # Checks if magic number has been read
        dataOK = 0  # Checks if the data has been read correctly
        detObj = {}

        readBuffer = self.Dataport.read(self.Dataport.in_waiting)
        self.byteBuffer += readBuffer

        if len(self.byteBuffer) > 16:
            loc = self.byteBuffer.find(self.magicWordB)
            if loc >= 0:
                self.byteBuffer = self.byteBuffer[loc:]
                self.byteBufferLength = len(self.byteBuffer)
                totalPacketLen = struct.unpack('<I', self.byteBuffer[12:16])[0]
                if self.byteBufferLength and self.byteBufferLength >= totalPacketLen:
                    magicOK = 1

        if magicOK:
            version, totalPacketLen, platForm, frameNumber, timeCpuCycles, numpDetectObj, numTLVs, subFrameNumber = struct.unpack('<8I', self.byteBuffer[8:40])
            version = format(version, 'x')
            platForm = format(platForm, 'x')
            idX = 40
            for tlvIdx in range(numTLVs):
                tlv_type, tlv_length = struct.unpack("<2I", self.byteBuffer[idX:idX+8])
                idX += 8
                if tlv_type == self.MMWDEMO_UART_MSG_DETECTED_POINTS:
                    tlv_numObj, tlv_xyzQFormat = struct.unpack("<2h", self.byteBuffer[idX:idX+4])
                    idX += 4
                    tlv_xyzQFormat = 2 ** tlv_xyzQFormat
                    dopplerIdx = np.zeros(tlv_numObj, dtype='int16')
                    peakVal = np.zeros(tlv_numObj, dtype='int16')
                    x = np.zeros(tlv_numObj, dtype='int16')
                    y = np.zeros(tlv_numObj, dtype='int16')
                    for odX in range(tlv_numObj):
                        dopplerIdx[odX], peakVal[odX], x[odX], y[odX] = struct.unpack("<hHhh", self.byteBuffer[idX:idX+8])
                        idX += 8
                    dopplerIdx[dopplerIdx > 32767] = dopplerIdx[dopplerIdx > 32767] - 65536
                    x[x > 32767] = x[x > 32767] - 65536
                    y[y > 32767] = y[y > 32767] - 65536
                    dopplerVal = dopplerIdx / tlv_xyzQFormat
                    x = x / tlv_xyzQFormat
                    y = y / tlv_xyzQFormat
                    detObj = {"numObj": tlv_numObj,
                              "doppler": dopplerVal,
                              "peakVal": peakVal,
                              "x": x,
                              "y": y,
                              }
                    dataOK = 1
                elif tlv_type == self.MMWDEMO_UART_MSG_CLUSTERS:
                    tlv_numObj, tlv_xyzQFormat = struct.unpack("<2h", self.byteBuffer[idX:idX+4])
                    idX += 4
                    tlv_xyzQFormat = 2 ** tlv_xyzQFormat
                    xsize = np.zeros(tlv_numObj, dtype='int16')
                    ysize = np.zeros(tlv_numObj, dtype='int16')
                    x = np.zeros(tlv_numObj, dtype='int16')
                    y = np.zeros(tlv_numObj, dtype='int16')
                    for odX in range(tlv_numObj):
                        x[odX], y[odX], xsize[odX], ysize[odX] = struct.unpack("<4h", self.byteBuffer[idX:idX+8])
                        idX += 8
                    x[x > 32767] = x[x > 32767] - 65536
                    y[y > 32767] = y[y > 32767] - 65536
                    x = x / tlv_xyzQFormat
                    y = y / tlv_xyzQFormat
                    xsize = xsize / tlv_xyzQFormat
                    ysize = ysize / tlv_xyzQFormat

                elif tlv_type == self.MMWDEMO_UART_MSG_TRACKED_OBJ:
                    tlv_numObj, tlv_xyzQFormat = struct.unpack("<2h", self.byteBuffer[idX:idX+4])
                    idX += 4
                    tlv_xyzQFormat = 2 ** tlv_xyzQFormat
                    xsize = np.zeros(tlv_numObj, dtype='int16')
                    ysize = np.zeros(tlv_numObj, dtype='int16')
                    x = np.zeros(tlv_numObj, dtype='int16')
                    y = np.zeros(tlv_numObj, dtype='int16')
                    xd = np.zeros(tlv_numObj, dtype='int16')
                    yd = np.zeros(tlv_numObj, dtype='int16')
                    for odX in range(tlv_numObj):
                        x[odX], y[odX], xd[odX], yd[odX], xsize[odX], ysize[odX] = struct.unpack("<6h", self.byteBuffer[idX:idX+12])
                        idX += 12
                    x[x > 32767] = x[x > 32767] - 65536
                    y[y > 32767] = y[y > 32767] - 65536
                    xd[xd > 32767] = xd[xd > 32767] - 65536
                    yd[yd > 32767] = yd[yd > 32767] - 65536
                    x = x / tlv_xyzQFormat
                    y = y / tlv_xyzQFormat
                    xd = xd / tlv_xyzQFormat
                    yd = yd / tlv_xyzQFormat
                    xsize = xsize / tlv_xyzQFormat
                    ysize = ysize / tlv_xyzQFormat

                elif tlv_type == self.MMWDEMO_UART_MSG_PARKING_ASSIST:
                    idX += tlv_length
                else:
                    idX += tlv_length
            if dataOK:
                self.byteBuffer = self.byteBuffer[totalPacketLen:]
                self.byteBufferLength = len(self.byteBuffer)
        return dataOK, detObj

    def stop(self):
        self.running = False
        self.CLIport.write(('sensorStop\n').encode())
        time.sleep(0.01)
        self.CLIport.close()
        time.sleep(0.01)
        self.Dataport.close()
        time.sleep(0.01)
        self.CLIport = {}
        self.Dataport = {}
        self.read_thread.join()
        self.read_thread = None
        self.monitor.join()
        print(f'[OK]{self.sensorID} is closed')

    def sensorSetup(self):
        self.serrialConfig()
        self.RSSConfig()
        time.sleep(.01)

    def open(self, max_failed=8):
        self.Dataport.reset_output_buffer()
        self.Dataport.reset_input_buffer()
        self.byteBufferLength = 0
        self.CLIport.write(('sensorStart\n').encode())
        self.failedSample = 0
        try:
            for i in range(10):
                loopStartTime = time.time()
                self.dataOK, self.detObj = self.SRRParseData()
                print(time.time()-loopStartTime)
                if not self.dataOK:
                    self.failedSample += 1
                time.sleep(.03 - (time.time() - loopStartTime))
            self.Dataport.reset_output_buffer()
            self.Dataport.reset_input_buffer()
            self.CLIport.write(('sensorStop\n').encode())
            print(f"[INFO] Sensor {self.CLIportNum} test result: Failed rate ({self.failedSample} out of 10)")
            if self.failedSample > max_failed:
                print("[ERROR] Link is unstable")
            else:
                self.sensorIsReady = True
                print ("[INFO] Sensor is ready - Port:" + self.CLIportNum)
        except KeyboardInterrupt:
            self.Dataport.reset_output_buffer()
            self.Dataport.reset_input_buffer()
            self.CLIport.write(('sensorStop\n').encode())
            print("Unable to open radar - Port:" + self.CLIportNum)

    def start(self):
        if self.running:
            print('Radar streaming is already running')
            return None
        if self.sensorIsReady:
            self.Dataport.reset_output_buffer()
            self.Dataport.reset_input_buffer()
            self.byteBufferLength = 0
            self.CLIport.write(('sensorStart\n').encode())
            time.sleep(.01)
            self.running = True
            self.isOpened = True
            self.failedSample = 0
            self.read_thread = threading.Thread(target=self.update)
            self.monitor = threading.Thread(target=self.monitorFun)
            self.read_thread.start()
            self.monitor.start()
        return self

    def update(self):
        while self.running:
            try:
                dataOK, detObj = self.SRRParseData()
                with self.read_lock:
                    if dataOK:
                        self.dataOK = dataOK
                        self.detObj = detObj
            except RuntimeError:
                print("Could not read point cloud from radar")

    def monitorFun(self):
        while self.running:
            try:
                if self.failedSample > 5:
                    with self.read_lock:
                        self.CLIport.write(('sensorStop\n').encode())
                        self.Dataport.reset_output_buffer()
                        self.Dataport.reset_input_buffer()
                        time.sleep(.01)
                        self.CLIport.write(('sensorStart\n').encode())
                        self.failedSample = 0
            except RuntimeError:
                pass

    def read(self):
        with self.read_lock:
            time_stamp = time.time()
            dataOK = self.dataOK
            detObj = self.detObj
            self.failedSample = 0 if dataOK else self.failedSample + 1
            self.dataOK = 0
            self.detObj = {}
        return dataOK, detObj, time_stamp

    def write(self, tempObj):
        if tempObj.data_is_ok:
            self.writer.writerow([
                tempObj.index,
                tempObj.time,
                tempObj.data["x"],
                tempObj.data["y"],
                tempObj.data["peakVal"],
                tempObj.data["doppler"],
            ])
        else:
            self.writer.writerow([
                tempObj.index,
                tempObj.time,
                "", "", "", ""])

class cli_generator:
    class dataPath_init:
        def __init__(self):
            self.numTxAzimAnt = 1
            self.numTxElevAnt = 0
            self.numRxAnt = 4
            self.numTxAnt = self.numTxAzimAnt + self.numTxElevAnt
            self.numChirpsPerFrame = None
            self.numDopplerBins = None
            self.numRangeBins = None
            self.rangeResolutionMeters = None
            self.rangeIdxToMeters = None
            self.dopplerResolutionMps = None
    class channelCfg_init:
        def __init__(self):
            self.txChannelEn = None
            self.rxChannelEn = None
    class profileCfg_init:
        def __init__(self):
            self.startFreq = 76.0
            self.idleTime = 3.0
            self.rampEndTime = 56.0
            self.freqSlopeConst = 8.0
            self.numAdcSamples = 256
            self.digOutSampleRate = 5000
    class framCfg_init:
        def __init__(self):
            self.chirpStartIdx = 0
            self.chirpEndIdx = 63
            self.numLoops = 1
            self.numFrames = 0
            self.framePeriodicity = 30
    class guiMonitor_init:
        def __init__(self):
            self.stat = 1
            self.numFigures = 1
    def __init__(self):
        self.channelCfg = self.channelCfg_init()
        self.dataPath = self.dataPath_init()
        self.profileCfg = self.profileCfg_init()
        self.frameCfg = self.framCfg_init()
        self.guiMonitor = self.guiMonitor_init()
        self.TOTAL_PAYLOAD_SIZE_BYTES = 32
        self.MAX_NUM_OBJECTS = 200
        self.OBJ_STRUCT_SIZE_BYTES = 8
        self.MAX_NUM_CLUSTERS = 24
        self.CLUSTER_STRUCT_SIZE_BYTES = 8
        self.MAX_NUM_TRACKERS = 24
        self.TRACKER_STRUCT_SIZE_BYTES = 12
        self.STATS_SIZE_BYTES = 16
        self.platformType = None
        self.STATS_SIZE_BYTES = None
        self.dataPath.numChirpsPerFrame = (self.frameCfg.chirpEndIdx - self.frameCfg.chirpStartIdx + 1) * self.frameCfg.numLoops
        self.dataPath.numDopplerBins = self.dataPath.numChirpsPerFrame / self.dataPath.numTxAnt
        self.dataPath.numRangeBins = pow2roundup(self.profileCfg.numAdcSamples)
        self.dataPath.rangeResolutionMeters = 3e8 * self.profileCfg.digOutSampleRate * 1e3 / (2 * self.profileCfg.freqSlopeConst * 1e12 * self.profileCfg.numAdcSamples)
        self.dataPath.rangeIdxToMeters = 3e8 * self.profileCfg.digOutSampleRate * 1e3 / (2 * self.profileCfg.freqSlopeConst * 1e12 * self.dataPath.numRangeBins)
        self.dataPath.dopplerResolutionMps = 3e8 / (2*self.profileCfg.startFreq*1e9 * (self.profileCfg.idleTime + self.profileCfg.rampEndTime) * 1e-6 * self.dataPath.numDopplerBins * self.dataPath.numTxAnt)

def generate_params_for_SRR():
    out = []
    out.append(cli_generator())
    P = cli_generator()
    P.channelCfg.txChannelEn = 3
    P.dataPath.numTxAzimAnt = 2
    P.dataPath.numTxElevAnt = 0
    P.channelCfg.rxChannelEn = 15
    P.dataPath.numRxAnt = 4
    P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt
    P.profileCfg.startFreq = 77.0
    P.profileCfg.idleTime = 7.00
    P.profileCfg.rampEndTime = 87.28
    P.profileCfg.freqSlopeConst = 42.0
    P.profileCfg.numAdcSamples = 512
    P.profileCfg.digOutSampleRate = 6222
    P.frameCfg.chirpStartIdx = 0
    P.frameCfg.chirpEndIdx = 1
    P.frameCfg.numLoops = 32
    P.frameCfg.numFrames = 0
    P.frameCfg.framePeriodicity = 30
    out.append(P)
    return P

# if __name__ == '__main__':
#     driver = awr1642("profile.cfg", "ACM0", "ACM1")
#     driver.sensorSetup()
#     # driver.optimize((5, 15) ,10)
#     # driver.setMaxRange(15)
#     # time.sleep(.01)
#     driver.run(20)
#     # driver.setNoiseAveragingWindow(32)
#     # time.sleep(1)
#     # driver.setRangePeakGrouping('dis')
#     # driver.run()
#     # # time.sleep(1)
#     # driver.setRangePeakGrouping('enable')
#     # time.sleep(5)
#     # driver.run()
#     driver.close()
