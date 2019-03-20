from threading import Thread
#from multiprocessing import Process
import numpy
import time
import cv2

class VideoGet:
    '''
    Class that continuously gets frames from a VideoGet object
    with a dedicated thread.
    '''
    def __init__(self, src=0):
        # Create global variables.
        self.mode = "tape"
        self.cameraSource = 1
        self.camera1 = cv2.VideoCapture(src)
        self.camera1.set(3, 400)
        self.camera1.set(4, 300)
        self.camera2 = cv2.VideoCapture(1)
        self.camera2.set(3, 400)
        self.camera2.set(4, 300)
        (self.grabbed, self.frame) = self.camera1.read()
        self.resolution = [self.camera1.get(3), self.camera1.get(4)]
        self.stopped = False

    def start(self):
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        # Grab frame from video stream.
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
            	# Determine which camera source to grab from.
            	if self.cameraSource == 1:
            		# Adjust brightness for tracking modes.
	                if self.mode == "free":
	                    self.camera1.set(cv2.CAP_PROP_BRIGHTNESS, 0.4)
			            #self.stream.set(cv2.CAP_PROP_EXPOSURE, 0.01)
	                if self.mode == "tape":
	                    self.camera1.set(cv2.CAP_PROP_BRIGHTNESS, -100)
	                    #self.stream.set(cv2.CAP_PROP_EXPOSURE, 0.005)

            		# Get low camera.
            		(self.grabbed, self.frame) = self.camera1.read()
            	else:
            		# Adjust brightness for tracking modes.
	                if self.mode == "free":
	                    self.camera2.set(cv2.CAP_PROP_BRIGHTNESS, 0.4)
			            #self.stream.set(cv2.CAP_PROP_EXPOSURE, 0.01)
	                if self.mode == "tape":
	                    self.camera2.set(cv2.CAP_PROP_BRIGHTNESS, -100)
	                    #self.stream.set(cv2.CAP_PROP_EXPOSURE, 0.005)

            		# Get high camera.
            		(self.grabbed, self.frame) = self.camera2.read()

                # Add time delay when reading from video files.
                #time.sleep(0.5)

    def stop(self):
        self.stopped = True
