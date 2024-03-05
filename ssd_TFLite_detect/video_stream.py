import cv2
import time
from threading import Thread
import multiprocessing
from datetime import datetime

class VideoStream:
    """Camera object that controls video streaming"""
    def __init__(self,resolution=(640,480),framerate=24,STREAM_URL=''):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(STREAM_URL)
        self.resolution = resolution
        self.framerate = framerate
        self.save_video = False
        self.reuslt_video = None
        self.start_time = time.time
        self.count_frame_save = 0
        self.capture_duration = 5
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

	# Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
	# Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream

            (self.grabbed, frame) = self.stream.read()
            if self.grabbed:
                # print("Error Read Frame")
                self.frame = frame
            else:
                # time.sleep(0.01)
                continue
                
            if self.save_video:
                self.reuslt_video.write(self.read())
                self.count_frame_save +=1
                # if (cv2.getTickCount() / cv2.getTickFrequency() - self.start_time) >self.capture_duration:
                if self.count_frame_save > self.capture_duration *15:
                    print("time ",cv2.getTickCount() / cv2.getTickFrequency() - self.start_time)
                    self.reuslt_video.release()
                    self.save_video = False
                    print("end save")
                
            # time.sleep(0.01)
# test lai

    def read(self):
	# Return the most recent frame
        self.frame = cv2.resize(self.frame,self.resolution)
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True

    def savevideo(self):

        if not self.save_video :
            print("start save video")
            self.capture_duration = 5
            self.reuslt_video = cv2.VideoWriter(self.genrate_video_filename(),
                                                cv2.VideoWriter_fourcc(*'MJPG'),
                                                self.framerate-10,
                                                self.resolution)
            self.start_time = cv2.getTickCount() / cv2.getTickFrequency()
            self.save_video = True
            self.count_frame_save = 0

    def genrate_video_filename(self):
        now = datetime.now()
        video_filename = 'Video_saved/'+ now.strftime("%Y%m%d_%H%M%S") + ".avi"
        return video_filename
    def increase_time(self,time = 3):
        self.capture_duration+=time