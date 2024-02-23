import cv2
import time
from threading import Thread
import multiprocessing

class VideoStream:
    """Camera object that controls video streaming"""
    def __init__(self,resolution=(640,480),framerate=30,STREAM_URL=''):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(STREAM_URL)
        self.resolution = resolution
            
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
                self.frame = frame
            time.sleep(0.01)


    def read(self):
	# Return the most recent frame
        self.frame = cv2.resize(self.frame,self.resolution)
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True
