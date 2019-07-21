#from multiprocessing import Process
from threading import Thread
import logging
import numpy
import json
import math
import time
import cv2
from simple_pid import PID
from networktables import NetworkTables

class VisionNetworking:
    '''
    Class that continuously sends data to and from the VideoProcess class
    between the RoboRIO with a dedicated thread.
    '''
    def __init__(self, gui, server, pointArray):
        # Create global variables.
        self.gui = gui
        self.numberOfObjects = 2
        self.mode = "tape"
        self.speed = 0
        self.server = server
        self.cameraSource = 1
        self.trackingPoint = 0
        self.area = [0,0,0,0]
        self.resolution = [0,0]
        self.rotation = [0,0,0,0]
        self.objectWidth = 11.875
        self.focalLength = (141 * 32) / self.objectWidth
        self.distance = 100
        self.objectDetected = False
        self.pointArray = pointArray
        self.trackbarValues = [[0,0], [0,0], [0,0], [0, 0]]
        self.pointsAreFlopped = [False, False, False, False]
        self.stopped = False

        # Initialize NetworkTables
        ip = self.server
        NetworkTables.initialize(server = ip)
        self.NWTB = NetworkTables.getTable("SmartDashboard")

        # Send placeholder vision NetworkTable data.
        self.NWTB.putBoolean("VisionFreeMode", False)
        self.NWTB.putBoolean("VisionIsActive", False)
        self.NWTB.putNumber("VisionMinArea", 0)
        self.NWTB.putNumber("VisionMaxArea", 5000)
        self.NWTB.putNumber("VisionTolerance", 5)
        self.NWTB.putNumber("VisionProportional", 0.18)
        self.NWTB.putNumber("VisionIntegral", 0.1)
        self.NWTB.putNumber("VisionDerivative", 0.01)
        self.NWTB.putNumber("VisionSpeedForward", 0.20) #0.145
        self.NWTB.putNumber("VisionTurningSpeed", 0.11)
        self.NWTB.putNumber("VisionNumberOfObjects", 3)
        self.NWTB.putNumber("VisionCenterOffset", 0)
        self.NWTB.putNumber("VisionCameraSource", 1)
        self.NWTB.putNumber("VisionVirtualJoystickX", 0)
        self.NWTB.putNumber("VisionVirtualJoystickY", 0)
        self.NWTB.putString("VisionFilterSide", "center")
        self.minArea = self.NWTB.getEntry("VisionMinArea")
        self.maxArea = self.NWTB.getEntry("VisionMaxArea")
        self.freeModeEnabled = self.NWTB.getEntry("VisionFreeMode")
        self.visionIsActive = self.NWTB.getEntry("VisionIsActive")
        self.armActual = self.NWTB.getEntry("Arm Actual")
        self.centerOffset = self.NWTB.getEntry("VisionCenterOffset")
        self.cameraIndex = self.NWTB.getEntry("VisionCameraSource")
        self.tolerance = self.NWTB.getEntry("VisionTolerance")
        self.filterSide = self.NWTB.getEntry("VisionFilterSide")
        self.proportional = self.NWTB.getEntry("VisionProportional")
        self.integral = self.NWTB.getEntry("VisionIntegral")
        self.derivative = self.NWTB.getEntry("VisionDerivative")
        self.speedForward = self.NWTB.getEntry("VisionSpeedForward")
        self.turningSpeed = self.NWTB.getEntry("VisionTurningSpeed")
        self.amountOfObjectsToTrack = self.NWTB.getEntry("VisionNumberOfObjects")

        # Create and setup the PID object.
        self.pid = PID(self.proportional.value, self.integral.value, self.derivative.value, setpoint = 0)
        self.pid.output_limits = (-100, 100)        
        self.pid.sample_time = 0.02
        self.pid.auto_mode = True

        # Open values from files and set trackbar position.
        with open("/home/nvidia/Desktop/Values/trackbarValues.json") as data_file:
            self.trackbarData = json.load(data_file)
        (self.trackbarValues[0][0]) = self.trackbarData["freeMode"]["hmn"]
        (self.trackbarValues[0][1]) = self.trackbarData["freeMode"]["hmx"]
        (self.trackbarValues[1][0]) = self.trackbarData["freeMode"]["smn"]
        (self.trackbarValues[1][1]) = self.trackbarData["freeMode"]["smx"]
        (self.trackbarValues[2][0]) = self.trackbarData["freeMode"]["vmn"]
        (self.trackbarValues[2][1]) = self.trackbarData["freeMode"]["vmx"]
        (self.trackbarValues[3][0]) = self.trackbarData["rotation"]["min"]
        (self.trackbarValues[3][1]) = self.trackbarData["rotation"]["max"]

    def start(self):
        Thread(target=self.network, args=()).start()
        return self

    def network(self):
        # Create variables and object pointers.
        rotationSwitcher = 0
        modeSwitcher = 0
        oldTime = 0
        
        while not self.stopped:
            # Get NetworkTable data.
            self.numberOfObjects = self.amountOfObjectsToTrack.value
            self.trackbarValues[3][0] = self.minArea.value
            self.trackbarValues[3][1] = self.maxArea.value
            self.cameraSource = self.cameraIndex.value
            if (self.freeModeEnabled.value == True) and (modeSwitcher == 0):
                self.mode = "free"
                modeSwitcher = 1
            if (self.freeModeEnabled.value == False) and (modeSwitcher == 1):
                self.mode = "tape"
                modeSwitcher = 0

            # Do calculations for find objects location, size, and distance.
            objectCounter = 0
            for object in self.pointArray:
                # Increment object counter.                
                objectCounter += 1

                # Determine rotation of the rectangle.
                if (object[0][0]) < (object[2][0]):
                    rotationSwitcher = 0

                if (object[0][0]) > (object[2][0]):
                    rotationSwitcher = 1

                # Correct calculations depending on orientation.
                if rotationSwitcher == 0:
                    # Output values to NetworkTables for robot movement.
                    if objectCounter == 1:
                        self.pointsAreFlopped[0] = False
                    if objectCounter == 2:
                        self.pointsAreFlopped[1] = False
                    if objectCounter == 3:
                        self.pointsAreFlopped[2] = False
                    if objectCounter == 4:
                        self.pointsAreFlopped[3] = False

                if rotationSwitcher == 1:
                    # Output values to NetworkTables for robot movement.
                    if objectCounter == 1:
                        self.pointsAreFlopped[0] = True
                    if objectCounter == 2:
                        self.pointsAreFlopped[1] = True
                    if objectCounter == 3:
                        self.pointsAreFlopped[2] = True
                    if objectCounter == 4:
                        self.pointsAreFlopped[3] = True

            # If tracking more than 1 object find the converging point of the extended object.
            if (self.numberOfObjects > 1) and (self.mode == "tape"):
                trackingPoints = [[0,0], [0,0], [0,0]]
                distances = [[0,0], [1,0], [2,0], [3,0]]
                
                # Find the distances of the objects from the left of the screen.
                objectCounter = 0
                for object in self.pointArray:
                    if (object[0][0] == (self.resolution[0] / 2) - 4) and (object[1][0] == (self.resolution[0] / 2) - 4):
                        distances[objectCounter][1] = 0
                    else:
                        distances[objectCounter][1] = object[2][0]
                    
                    # Increment objectCounter
                    objectCounter += 1

                # Sorted the values in ascending order.
                distances = sorted(distances, key=lambda distances: distances[1])
                print distances

                # Find the distance of the object using the focal length of the camera and the object size.
                if (distances[0][1] != 0 and distances[1][1] != 0):
                    self.distance = (self.objectWidth * self.focalLength) / (distances[1][1] - distances[0][1])
                else:
                    if (distances[2][1] != 0 and distances[3][1] != 0):
                        self.distance = (self.objectWidth * self.focalLength) / (distances[3][1] - distances[2][1])

                print self.distance
                print (distances[3][1] - distances[2][1])

                # Find the intersection for all pairs of points.
                objectCounter = 0
                for x in range(1, len(self.pointArray)):
                    # Increment objectCounter.
                    objectCounter += 1

                    # Determine which objects to use for findIntersection.
                    if x == 1:
                        firstObject = distances[0][0]
                        secondObject = distances[1][0]
                    if x == 2:
                        firstObject = distances[1][0]
                        secondObject = distances[2][0]
                    if x == 3:
                        firstObject = distances[2][0]
                        secondObject = distances[3][0]

                    # Get the correct points of the objects.
                    if self.pointsAreFlopped[firstObject] == True:
                        x1 = self.pointArray[firstObject][1][0]
                        y1 = self.pointArray[firstObject][1][1]
                        x2 = self.pointArray[firstObject][2][0]
                        y2 = self.pointArray[firstObject][2][1]
                    else:
                        x1 = self.pointArray[firstObject][3][0]
                        y1 = self.pointArray[firstObject][3][1]
                        x2 = self.pointArray[firstObject][2][0]
                        y2 = self.pointArray[firstObject][2][1]
                    if self.pointsAreFlopped[secondObject] == True:
                        x3 = self.pointArray[secondObject][1][0]
                        y3 = self.pointArray[secondObject][1][1]
                        x4 = self.pointArray[secondObject][2][0]
                        y4 = self.pointArray[secondObject][2][1]
                    else:
                        x3 = self.pointArray[secondObject][3][0]
                        y3 = self.pointArray[secondObject][3][1]
                        x4 = self.pointArray[secondObject][2][0]
                        y4 = self.pointArray[secondObject][2][1]

                    # Do the calculations for finding the intersection point and then store it.
                    x, y = self.findIntersection(x1,y1,x2,y2,x3,y3,x4,y4)
                    trackingPoints[objectCounter - 1][0] = x
                    trackingPoints[objectCounter - 1][1] = y

                # Determine which point is clostest to center and above the objects.
                pointCounter = 0
                filterPoints = [[0,0], [0,0]]
                for point in trackingPoints:
                    # Filter out inverted tracking points.
                    if point[1] > self.pointArray[0][0][1]:
                        point[0] = 0
                        point[1] = self.resolution[0]
                    # If point is not inverted add it to the array.
                    if (point[0] != 0) and (point[1] != 0) and (pointCounter <= 1):
                        filterPoints[pointCounter][0] = point[0]
                        if self.filterSide.value == "center":
                            filterPoints[pointCounter][1] = abs(int(self.resolution[0] / 2) - point[0])
                        else:
                            filterPoints[pointCounter][1] = point[0]
                        # Increment pointCounter.
                        pointCounter += 1

                # Remove any numbers of infinity.
                for info in filterPoints:
                    if (math.isinf(info[0])) or (math.isinf(info[1])):
                        info[0] = 0
                        info[1] = self.resolution[0]
                    if (info[0] == 0):
                        if self.filterSide.value == "right":
                            info[1] = 0
                        else:
                            info[1] = self.resolution[0]

                # Find the intersection for all pairs of points.
                if (filterPoints[0][0] != 0) or (filterPoints[1][0] != 0):
                    # Find the point with the least amount of distance to the center and output number.
                    if self.filterSide.value == "right":
                        if filterPoints[0][1] > filterPoints[1][1]:
                            self.trackingPoint = filterPoints[0][0]
                            self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)
                        if filterPoints[1][1] > filterPoints[0][1]:
                            self.trackingPoint = filterPoints[1][0]
                            self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)
                    else:
                        if filterPoints[0][1] < filterPoints[1][1]:
                            self.trackingPoint = filterPoints[0][0]
                            self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)
                        if filterPoints[1][1] < filterPoints[0][1]:
                            self.trackingPoint = filterPoints[1][0]
                            self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)

                    # If nothing is detect reset trackingPoint and PID.
                    if (filterPoints[0][0] == 0) and (filterPoints[1][0] == 0):
                        self.trackingPoint = self.resolution[0] / 2
                        self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)
                        self.pid.set_auto_mode(False, last_output = 0)

            # Calculate the x and y joystick values for robot if in free mode.
            xCenter = 0
            if (self.mode == "free"):
                # Determine rotation of the rectangle.
                if (self.pointArray[0][0][0]) < (self.pointArray[0][2][0]):
                    # Calculate width of the object.
                    xSize = abs(abs((self.pointArray[0][2][0])) - abs((self.pointArray[0][0][0])))
                    # Find center point on x axis.
                    xCenter = abs((self.pointArray[0][0][0])) + xSize / 2
                if (self.pointArray[0][0][0]) > (self.pointArray[0][2][0]):
                    # Calculate the width of the object with reversed calculations.
                    xSize = abs(abs((self.pointArray[0][0][0])) - abs((self.pointArray[0][2][0])))
                    # Find center point on x axis.
                    xCenter = abs((self.pointArray[0][2][0])) + xSize / 2

                # Set trackingPoint equal to xCenter.
                self.trackingPoint = xCenter

                # Calculate joystick values.
                joystickX , joystickY = self.convertToJoystick(self.trackingPoint, self.tolerance, self.speedForward, self.turningSpeed, self.armActual)
                # Put values in NetworkTables
                self.NWTB.putNumber("VisionVirtualJoystickX", joystickX)
                self.NWTB.putNumber("VisionVirtualJoystickY", joystickY)

            # Calculate the x and y joystick values for robot if in tape mode.
            if (self.mode == "tape"):
                joystickX , joystickY = self.convertToJoystick(self.trackingPoint, self.tolerance, self.speedForward, self.turningSpeed, self.armActual)
                # Put values in NetworkTables
                self.NWTB.putNumber("VisionVirtualJoystickX", joystickX)
                self.NWTB.putNumber("VisionVirtualJoystickY", joystickY)

            # Check if the files or mode have changed and set trackbar values.
            if self.mode == "free":
                if (self.trackbarValues[0][0]) != self.trackbarData["freeMode"]["hmn"]:
                    (self.trackbarValues[0][0]) = self.trackbarData["freeMode"]["hmn"]
                if (self.trackbarValues[0][1]) != self.trackbarData["freeMode"]["hmx"]:
                    (self.trackbarValues[0][1]) = self.trackbarData["freeMode"]["hmx"]
                if (self.trackbarValues[1][0]) != self.trackbarData["freeMode"]["smn"]:
                    (self.trackbarValues[1][0]) = self.trackbarData["freeMode"]["smn"]
                if (self.trackbarValues[1][1]) != self.trackbarData["freeMode"]["smx"]:
                    (self.trackbarValues[1][1]) = self.trackbarData["freeMode"]["smx"]
                if (self.trackbarValues[2][0]) != self.trackbarData["freeMode"]["vmn"]:
                    (self.trackbarValues[2][0]) = self.trackbarData["freeMode"]["vmn"]
                if (self.trackbarValues[2][1]) != self.trackbarData["freeMode"]["vmx"]:
                    (self.trackbarValues[2][1]) = self.trackbarData["freeMode"]["vmx"]

            if self.mode == "tape":
                if (self.trackbarValues[0][0]) != self.trackbarData["tapeMode"]["hmn"]:
                    (self.trackbarValues[0][0]) = self.trackbarData["tapeMode"]["hmn"]
                if (self.trackbarValues[0][1]) != self.trackbarData["tapeMode"]["hmx"]:
                    (self.trackbarValues[0][1]) = self.trackbarData["tapeMode"]["hmx"]
                if (self.trackbarValues[1][0]) != self.trackbarData["tapeMode"]["smn"]:
                    (self.trackbarValues[1][0]) = self.trackbarData["tapeMode"]["smn"]
                if (self.trackbarValues[1][1]) != self.trackbarData["tapeMode"]["smx"]:
                    (self.trackbarValues[1][1]) = self.trackbarData["tapeMode"]["smx"]
                if (self.trackbarValues[2][0]) != self.trackbarData["tapeMode"]["vmn"]:
                    (self.trackbarValues[2][0]) = self.trackbarData["tapeMode"]["vmn"]
                if (self.trackbarValues[2][1]) != self.trackbarData["tapeMode"]["vmx"]:
                    (self.trackbarValues[2][1]) = self.trackbarData["tapeMode"]["vmx"]

            # If the object is lost, set a timeout and reset to center.
            #seconds = time.time()
            #if seconds - oldTime >= 0.02:
                #self.trackingPoint = self.resolution[0] / 2
                #oldTime = seconds

            # Print debug info if the gui is disabled.
            #if self.gui == "no":
                #print "Vision Camera Res: " + str(self.resolution[0]) + "x" + str(self.resolution[1])
                #print "Vision Mode: " + self.mode
                #print "Vision Area: " + str(self.area)
                #print "Vision Center: " + str(self.trackingPoint)
                #print "Vision Rotation: " + str(self.rotation)
                #print "Object Detected: " + str(self.objectDetected)
                #print "Vision Frequency: " + str(self.speed)
                #print "Vision JoystickXY: " + str(joystickX), str(joystickY)
                #print "Vision Point Array: " + str(self.pointArray[0]) + str(self.pointArray[1])
                #print "Vision Track-bar Values Array: " + str(self.trackbarValues)

            # Add delay so it doesn't flood the network.
            time.sleep(0.02)

    def findIntersection(self,x1,y1,x2,y2,x3,y3,x4,y4):
        # Find intersection points of two lines.
        if 0 in [x1,y2,x2,y2,x3,y3,x4,y4] or x2 == x1 or x4 == x3:
            intersectPointX = 0
            intersectPointY = 0
        else:
            # Find X.
            if (x1 < x3):
                intersectPointX = ((x3 - x1) / 2) + x1
            else:
                intersectPointX = ((x1 - x3) / 2) + x3
            
            # Find the equation of the line.
            m = (y2 - y1) / (x2 - x1)
            b = y1 - m * x1

            # Find Y.
            intersectPointY = (m * intersectPointX) + b
        return (intersectPointX, intersectPointY)
    
    def convertToJoystick(self, objectCenter, tolerance, speedForward, turningSpeed, armActual):
        # Create variables.
        cameraCenter = self.resolution[0] / 2
        tolerance = tolerance.value
        armActual = armActual.value
        speedForward = speedForward.value
        turningSpeed = turningSpeed.value
        joystickX = 0
        joystickY = 0
        setpointLeft = 0
        setpointRight = 0
        errorLeft = 0
        errorRight = 0
        
        # Offset the center of the camera. (If the camera is not perfectly center)
        cameraCenter += self.centerOffset.value        

        # Determine Setpoints depending on tolerance.
        setpointLeft = (cameraCenter - tolerance)
        setpointRight = (cameraCenter + tolerance)
		
        # Calculate Error.
        errorLeft = (setpointLeft - objectCenter)
        errorRight = (objectCenter - setpointRight)
        self.NWTB.putNumber("VisionErrorLeft", errorLeft)
        self.NWTB.putNumber("VisionErrorRight", errorRight)
        
        # Get the P, I, D values from NetworkTables.
        self.pid.tunings = (self.proportional.value, self.integral.value, self.derivative.value)

        # Disable PID if vision is not being used.
        if (self.visionIsActive.value == False):
            self.pid.set_auto_mode(False, last_output = 0)
            self.joystickX = 0
            self.joystickY = 0
        else:
            # Use PID to determine output depending on errorLeft and errorRight.
            if (objectCenter < setpointLeft):
                self.pid.auto_mode = True
                joystickX = self.pid(errorLeft)
                joystickX = joystickX * -0.01
            if (objectCenter > setpointRight):
                self.pid.auto_mode = True
                joystickX = self.pid(errorRight)
                joystickX = joystickX * 0.01
            if (objectCenter > (setpointLeft + self.centerOffset.value)) and (objectCenter < (setpointRight + self.centerOffset.value)):
                self.pid.set_auto_mode(False, last_output = 0)

        # If the bucket is up then use the bottom camera and closest distance.
        if (armActual < 65):
            self.NWTB.putNumber("VisionCameraSource", 2)
        # If the bucket is down then use the top camera and farthest distance.
        if (armActual > 65):
            self.NWTB.putNumber("VisionCameraSource", 1)

        # Set max speed for turning speed.
        if (joystickX <= turningSpeed * -1):
            joystickX = turningSpeed * -1
        if (joystickX >= turningSpeed):
            joystickX = turningSpeed
		
        # Set max forward speed.
        joystickY = speedForward * -1
        
        # Return the calculated joystick values.
        return joystickX, joystickY

    def stop(self):
        self.stopped = True
