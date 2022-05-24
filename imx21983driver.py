# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# A simple code snippet
# Using two  CSI cameras (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit with two CSI ports (Jetson Nano, Jetson Xavier NX) via OpenCV
# Drivers for the camera and OpenCV are included in the base image in JetPack 4.3+

# This script will open a window and place the camera stream from each camera in a window
# arranged horizontally.
# The camera streams are each read in their own thread, as when done sequentially there
# is a noticeable lag
import csv
import threading
import time

import cv2
import numpy as np
import os
import queue
import random


class CSI_Camera:

    def __init__(self, sensor_id=None):
        self.video_capture = None
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
        self.writerObj = None


    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()
            self.sensorIsReady = self.grabbed

        except:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
            self.isOpened = self.video_capture.isOpened()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        print(f'[OK]{self.sensorID} is closed')

    def updateCamera(self):
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
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
            self.frame = None
            self.grabbed = 0
        return grabbed, frame, time_stamp

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        if self.read_thread != None:
            self.read_thread.join()

    def log(self, tempObj):
        filename = f"frame_{self.sensorID}_{tempObj.index}"
        if tempObj.data_is_ok:
            self.logger.writerow([
                tempObj.index,
                tempObj.time,
                filename
            ])
        else:
            self.logger.writerow([
                tempObj.index,
                tempObj.time,
                "",
            ])

    def logP(self, log_buffer):
        log_file = open(f'{os.getpid()}.csv', 'w', newline='')
        file_header = ["index", "time", "imageName"]
        logger = csv.writer(log_file)
        logger.writerow(file_header)
        while self.logObj.running.value or not log_buffer.empty():
            try:
                tempObj = log_buffer.get(0)
                filename = f"{tempObj.source}/{tempObj.source}_{tempObj.index}.jpg"
                logger.writerow([
                    tempObj.index,
                    tempObj.time,
                    filename if tempObj.data_is_ok else None
                ])
            except RuntimeError:
                print('[ERROR] Could not save an frame')
            except queue.Empty:
                pass
        else:
            log_file.close()
            os.rename(f'{os.getpid()}.csv', f'{tempObj.source}.csv')
            print(f"[OK] Process id: {self.logObj.name} has joined")

    def video_writer(self, data_buffer, cap_size):
        pWriter = cv2.VideoWriter(f'{os.getpid()}.mp4',
                        0x7634706d,
                        30,
                        cap_size,
                        )
        with open(f'{os.getpid()}.txt', 'w') as f:
            while self.writerObj.running.value or not data_buffer.empty():
                try:
                    if random.random() < .9:
                        tempObj = data_buffer.get(0)
                        if tempObj.data_is_ok:
                            pWriter.write(tempObj.data)
                            f.write(f'{tempObj.source}_{tempObj.index}\n')
                except RuntimeError:
                    print('[ERROR] Could not save an frame')
                except queue.Empty:
                    time.sleep(.01)
            else:
                pWriter.release()
                print(f"[OK] Process id: {os.getpid()} has joined")

    def image_writer(self, data_buffer):
        while self.writerObj.running.value or not data_buffer.empty():
            try:
                if random.random() < .9:
                    tempObj = data_buffer.get(0)
                    if tempObj.data_is_ok:
                        filename = f"{tempObj.source}/{tempObj.source}_{tempObj.index}.jpg"
                        cv2.imwrite(filename, tempObj.data)
            except RuntimeError:
                print('[ERROR] Could not save an frame')
            except queue.Empty:
                time.sleep(.01)
        else:
            print(f"[OK] process id: {os.getpid()} has joined")



""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080
"""


def gstreamer_pipeline(
        sensor_id=0,
        capture_width=1920,
        capture_height=1080,
        display_width=1920,
        display_height=1080,
        framerate=30,
        flip_method=0,
):
    return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )


def run_cameras():
    window_title = "Dual CSI Cameras"
    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=0,
            capture_width=1920,
            capture_height=1080,
            flip_method=0,
            display_width=960,
            display_height=540,
        )
    )
    left_camera.start()

    right_camera = CSI_Camera()
    right_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=1920,
            capture_height=1080,
            flip_method=0,
            display_width=960,
            display_height=540,
        )
    )
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
                # Use numpy to place images next to each other
                camera_images = np.hstack((left_image, right_image))
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, camera_images)
                else:
                    break

                # This also acts as
                keyCode = cv2.waitKey(30) & 0xFF
                # Stop the program on the ESC key
                if keyCode == 27:
                    break
        finally:

            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open both cameras")
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()




# if __name__ == "__main__":
#     run_cameras()
