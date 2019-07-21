from threading import Thread
#from multiprocessing import Process
import numpy as np
import time
import cv2
import cv2 as cv
from CountsPerSec import CountsPerSec

class VideoProcess:
    '''
    Class that continuously shows a frame using a dedicated thread.
    '''
    def __init__(self, gui, frame=None, resolution=None, trackbarValues=None):
        # Create object pointer for speed tracking.
        self.cps = CountsPerSec().start()

        # Create global variables.
        self.gui = gui
        self.mode = "free"
        self.frame = frame
        self.area = [0,0,0,0]
        self.rotation = [0,0,0,0]
        self.objectDistance = 0
        self.trackingPoint =  0
        self.numberOfObjects = 2
        self.processedFrame = frame
        self.objectDetected = False
        self.resolution = resolution
        self.speed = self.cps.countsPerSec()
        self.trackbarValues = trackbarValues
        self.object1 = [[0,0], [0,0], [0,0], [0,0]]
        self.object2 = [[0,0], [0,0], [0,0], [0,0]]
        self.object3 = [[0,0], [0,0], [0,0], [0,0]]
        self.object4 = [[0,0], [0,0], [0,0], [0,0]]
        self.pointArray = [self.object1, self.object2, self.object3, self.object4]
        self.stopped = False

    def start(self):
        Thread(target=self.detectObject, args=()).start()
        return self

    def detectObject(self):
        # Create variables.
        setPosition = True
        sameAsBefore = 0

        if self.gui == "yes":
            # This is Mostly Useless
            def nothing(x):
                pass

            # Create GUI windows
            cv2.namedWindow("HueComp")
            cv2.namedWindow("SatComp")
            cv2.namedWindow("ValComp")
            cv2.namedWindow("Tracking")
            cv2.createTrackbar("hmin", "HueComp", int(self.trackbarValues[0][0]), 179, nothing)
            cv2.createTrackbar("hmax", "HueComp", int(self.trackbarValues[0][1]), 179, nothing)
            cv2.createTrackbar("smin", "SatComp", int(self.trackbarValues[1][0]), 255, nothing)
            cv2.createTrackbar("smax", "SatComp", int(self.trackbarValues[1][1]), 255, nothing)
            cv2.createTrackbar("vmin", "ValComp", int(self.trackbarValues[2][0]), 255, nothing)
            cv2.createTrackbar("vmax", "ValComp", int(self.trackbarValues[2][1]), 255, nothing)
            cv2.createTrackbar("minArea", "Tracking", 0, 5000, nothing)
            cv2.createTrackbar("maxArea", "Tracking", 0, 5000, nothing)

        while not self.stopped:
            # Refresh camera frame.
            frame = self.frame.copy()

            # Take input from camera and split into three windows.
            kernel = np.ones((5,5),np.uint8)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hue,sat,val = cv2.split(hsv)

            # Read and change trackbar values.
            if self.gui == "yes":
                # Determine if trackbars need to be set.
                if self.mode == "free" and sameAsBefore == 0:
                    setPosition = True
                    sameAsBefore = 1

                if self.mode =="tape" and sameAsBefore == 1:
                    setPosition = True
                    sameAsBefore = 0

                # Set trackbar positions when mode changes.
                if setPosition == True:
                    cv2.setTrackbarPos("hmin", "HueComp", int(self.trackbarValues[0][0]))
                    cv2.setTrackbarPos("hmax", "HueComp", int(self.trackbarValues[0][1]))
                    cv2.setTrackbarPos("smin", "SatComp", int(self.trackbarValues[1][0]))
                    cv2.setTrackbarPos("smax", "SatComp", int(self.trackbarValues[1][1]))
                    cv2.setTrackbarPos("vmin", "ValComp", int(self.trackbarValues[2][0]))
                    cv2.setTrackbarPos("vmax", "ValComp", int(self.trackbarValues[2][1]))
                    cv2.setTrackbarPos("minArea", "Tracking", int(self.trackbarValues[3][0]))
                    cv2.setTrackbarPos("maxArea", "Tracking", int(self.trackbarValues[3][1]))
                    setPosition = False

                # Get trackbar positions.
                hmn = cv2.getTrackbarPos("hmin","HueComp")
                hmx = cv2.getTrackbarPos("hmax","HueComp")
                smn = cv2.getTrackbarPos("smin","SatComp")
                smx = cv2.getTrackbarPos("smax","SatComp")
                vmn = cv2.getTrackbarPos("vmin","ValComp")
                vmx = cv2.getTrackbarPos("vmax","ValComp")
                areaThresh = [cv2.getTrackbarPos("minArea", "Tracking"), cv2.getTrackbarPos("maxArea", "Tracking")]
            else:
                hmn = int(self.trackbarValues[0][0])
                hmx = int(self.trackbarValues[0][1])
                smn = int(self.trackbarValues[1][0])
                smx = int(self.trackbarValues[1][1])
                vmn = int(self.trackbarValues[2][0])
                vmx = int(self.trackbarValues[2][1])
                areaThresh = [int(self.trackbarValues[3][0]), int(self.trackbarValues[3][1])]

            # Apply thresholding to windows.
            hthresh = cv2.inRange(np.array(hue),np.array(hmn),np.array(hmx))
            sthresh = cv2.inRange(np.array(sat),np.array(smn),np.array(smx))
            vthresh = cv2.inRange(np.array(val),np.array(vmn),np.array(vmx))

            # AND together hue, sat, and val.
            combined = cv2.bitwise_and(hthresh,cv2.bitwise_and(sthresh,vthresh))
            # Remove small blobs in combined image.
            clean = cv2.medianBlur(combined, 5)
            
            # Fuse the image and add slight blur to improve tracking.
            #dilation = cv2.dilate(clean, kernel, iterations = 1)
            closed = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)
            blurred = cv2.GaussianBlur(closed,(5,5),0)

            # Combine all HSV windows to 'closing'
            #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            # Close any gaps to complete rectangle and output in new window 'closed'
            #closed = cv2.morphologyEx(closing, cv2.MORPH_CLOSE, kernel)
            # Create a wireframe from 'closed' and output as 'edged'
            #edged = cv2.Canny(closed, 10, 240)

            # Set new threshold.
            _, bin = cv2.threshold(blurred, 40, 255, 0)

            # Find contours of the second image.
            bin, contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # Read contours and draw the four points of rectangle.
            if len(contours) != 0:
                # Output that program has detected object and reset object values.
                self.objectDetected = True
                for object in self.pointArray:
                    object[0][0] = (self.resolution[0] / 2) - 4
                    object[0][1] = (self.resolution[1] / 2) - 4
                    object[1][0] = (self.resolution[0] / 2) - 4
                    object[1][1] = (self.resolution[1] / 2) + 4
                    object[2][0] = (self.resolution[0] / 2) + 4
                    object[2][1] = (self.resolution[1] / 2) - 4
                    object[3][0] = (self.resolution[0] / 2) + 4
                    object[3][1] = (self.resolution[1] / 2) + 4

                # Draw filtered contours in white.
                cv2.drawContours(frame, contours, -1, ( 255, 255, 210), 1)

                # Filter out all contours except for the biggest numberOfObjects.
                contours = sorted(contours, key = cv2.contourArea, reverse = True)[:int(self.numberOfObjects)]

                # Find contours within an area range.
                objectCounter = 0
                for cnts in contours:
                    # Find contour area for thresholding and increment counter.
                    area = cv2.contourArea(cnts)
                    objectCounter += 1
                    if (area < areaThresh[1]) and (area > areaThresh[0]):
                        # Find the four points of the rectangle.
                        rect = cv2.minAreaRect(cnts)
                        box = cv2.boxPoints(rect)
                        
                        # Store points, rotation, and area of object for calculations.
                        if objectCounter == 1:
                            self.object1 = box
                            self.pointArray[0] = self.object1
                            self.rotation[0] = rect[2]
                            self.area[0] = area
                        if objectCounter == 2:
                            self.object2 = box
                            self.pointArray[1] = self.object2
                            self.rotation[1] = rect[2]
                            self.area[1] = area
                        if objectCounter == 3:
                            self.object3 = box
                            self.pointArray[2] = self.object3
                            self.rotation[2] = rect[2]
                            self.area[2] = area
                        if objectCounter == 4:
                            self.object4 = box
                            self.pointArray[3] = self.object4
                            self.rotation[3] = rect[2]
                            self.area[3] = area

                        # Draw tracking overlay for gui enabled.
                        if self.gui == "yes":
                            # Create temp variables.
                            nameStepper = 0
                            font = cv2.FONT_HERSHEY_SIMPLEX

                            # Draw object detected text.
                            if self.trackingPoint == 0:
                                cv2.putText(frame, "Distance: ???", (int(self.resolution[0]) - 130, int(self.resolution[1]) - 10), font, 1, (255,255,255), 1, cv2.LINE_AA)
                            else:
                                cv2.putText(frame, str(self.objectDistance), (int(self.resolution[0]) - 130, int(self.resolution[1]) - 10), font, 1, (255,255,255), 1, cv2.LINE_AA)
                                cv2.line(frame, (int(self.trackingPoint), int(box[2][1])), (int(self.trackingPoint), int(box[0][1])), (0,255,0), 2)

                            # Draw circles and numbers.
                            for p in box:
                                nameStepper = nameStepper + 1
                                # Draw blue circles and numbers.
                                pt = int(p[0]), int(p[1])
                                pt2 = int(p[0]) - 50, int(p[1]) - 50
                                cv2.circle(frame, pt, 5, (255, 0, 0), 1)
                                cv2.putText(frame, str(nameStepper), pt, font, 1, (0, 0, 255), 1, cv2.LINE_AA)
                                cv2.putText(frame, str(objectCounter), pt2, font, 1, (0, 0, 255), 1, cv2.LINE_AA)

                        # Draw tracking overlay with gui disabled.
                        if self.gui == "no":
                            # Create temp variables.
                            nameStepper = 0
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            
                            # Draw object detected text.
                            if self.trackingPoint == 0:
                                cv2.putText(frame, "Distance: ???", (int(self.resolution[0]) - 130, int(self.resolution[1]) - 10), font, 1, (255,255,255), 1, cv2.LINE_AA)
                            else:
                                cv2.putText(frame, str(self.objectDistance), (int(self.resolution[0]) - 130, int(self.resolution[1]) - 10), font, 1, (255,255,255), 1, cv2.LINE_AA)
                                cv2.line(frame, (int(self.trackingPoint), int(box[2][1])), (int(self.trackingPoint), int(box[0][1])), (0,255,0), 2)
                            
                            # Draw enclosing circle.
                            (x, y), radius = cv2.minEnclosingCircle(cnts)
                            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 1)
                            for p in box:
                                # Draw blue circles.
                                pt = int(p[0]), int(p[1])
                                cv2.circle(frame, pt, 5, (255, 0, 0), 1)

            else:
                # Default values if nothing is detected.
                for object in self.pointArray:
                    object[0][0] = (self.resolution[0] / 2) - 4
                    object[0][1] = (self.resolution[1] / 2) - 4
                    object[1][0] = (self.resolution[0] / 2) - 4
                    object[1][1] = (self.resolution[1] / 2) + 4
                    object[2][0] = (self.resolution[0] / 2) + 4
                    object[2][1] = (self.resolution[1] / 2) - 4
                    object[3][0] = (self.resolution[0] / 2) + 4
                    object[3][1] = (self.resolution[1] / 2) + 4
                # Output that the program does not detect an object.
                self.objectDetected = False

            # Enable/Disable GUI.
            if self.gui == "yes":
                # Write image output to windows.
                cv2.imshow("HueComp", hthresh)
                cv2.imshow("SatComp", sthresh)
                cv2.imshow("ValComp", vthresh)
                cv2.imshow("Combined", combined)
                cv2.imshow("Clean", clean)
                #cv2.imshow("Dilation", dilation)
                cv2.imshow("Closed", closed)
                cv2.imshow("Blurred", blurred)
                #cv2.imshow("Edged", edged)
                cv2.imshow("Tracking", frame)

            # Send the final frame for streaming.
            self.processedFrame = frame

            # Increment counts per second.
            self.cps.increment()
            self.speed = self.cps.countsPerSec()

            # Add small delay and if q is pressed quit.
            if cv2.waitKey(1) == ord("q"):
                self.stopped = True

            # Wait for other threads to catch up.
            time.sleep(0.02)

    def stop(self):
        self.stopped = True
