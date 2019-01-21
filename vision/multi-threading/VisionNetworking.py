#from multiprocessing import Process
from threading import Thread
import logging
import numpy
import math
import time
import cv2
from networktables import NetworkTables

class VisionNetworking:
    '''
    Class that continuously sends data to and from the VideoProcess class
    between the RoboRIO with a dedicated thread.
    '''
    def __init__(self, gui, server, pointArray):
        # Create global variables.
        self.gui = gui
        self.numberOfObjects = 4
        self.mode = "free"
        self.speed = 0
        self.server = server
        self.trackingPoint = 0
        self.area = [0,0,0,0]
        self.resolution = [0,0]
        self.rotation = [0,0,0,0]
        self.objectDetected = False
        self.pointArray = pointArray
        self.trackbarValues = [[0,0], [0,0], [0,0], [0, 0]]
        self.pointsAreFlopped = [False, False, False, False]
        self.stopped = False

        # Initialize NetworkTables
        ip = self.server
        NetworkTables.initialize(server = ip)
        self.NWTB = NetworkTables.getTable("SmartDashboard")

        # Send vision NetworkTable data.
        self.NWTB.putBoolean("VisionFreeMode", True)
        self.NWTB.putNumber("VisionMinArea", 0)
        self.NWTB.putNumber("VisionMaxArea", 1000)
        self.NWTB.putNumber("VisionNumberOfObjects", 4)
        self.NWTB.putNumber("VisionHorizontalRes", self.resolution[0])
        self.minArea = self.NWTB.getEntry("VisionMinArea")
        self.maxArea = self.NWTB.getEntry("VisionMaxArea")
        self.freeModeEnabled = self.NWTB.getEntry("VisionFreeMode")
        self.amountOfObjectsToTrack = self.NWTB.getEntry("VisionNumberOfObjects")
        
        # Open values from files and set trackbar position.
        self.hmnReadFree = open("/home/pi/Desktop/Values/FreeMode/hmn.txt", "r")
        self.hmxReadFree = open("/home/pi/Desktop/Values/FreeMode/hmx.txt", "r")
        self.smnReadFree = open("/home/pi/Desktop/Values/FreeMode/smn.txt", "r")
        self.smxReadFree = open("/home/pi/Desktop/Values/FreeMode/smx.txt", "r")
        self.vmnReadFree = open("/home/pi/Desktop/Values/FreeMode/vmn.txt", "r")
        self.vmxReadFree = open("/home/pi/Desktop/Values/FreeMode/vmx.txt", "r")
        self.areaReadMin = open("/home/pi/Desktop/Values/Telemetry/areamin.txt", "r")
        self.areaReadMax = open("/home/pi/Desktop/Values/Telemetry/areamax.txt", "r")
        (self.trackbarValues[0][0]) = self.hmnReadFree.read()
        (self.trackbarValues[0][1]) = self.hmxReadFree.read()
        (self.trackbarValues[1][0]) = self.smnReadFree.read()
        (self.trackbarValues[1][1]) = self.smxReadFree.read()
        (self.trackbarValues[2][0]) = self.vmnReadFree.read()
        (self.trackbarValues[2][1]) = self.vmxReadFree.read()
        (self.trackbarValues[3][0]) = self.areaReadMin.read()
        (self.trackbarValues[3][1]) = self.areaReadMax.read()

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
            if self.freeModeEnabled.value == True and modeSwitcher == 0:
                self.mode = "free"
                modeSwitcher = 1
            if self.freeModeEnabled.value == False and modeSwitcher == 1:
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
                    # Calculate width of the object.
                    xSize = abs(abs((object[2][0])) - abs((object[0][0])))
                    # Find center point on x axis.
                    xCenter = abs((object[0][0])) + xSize / 2
                    # Output values to NetworkTables for robot movement.
                    if objectCounter == 1:
                        self.NWTB.putNumber("VisionObject1Size", xSize)
                        self.NWTB.putNumber("VisionObject1Center", xCenter)
                        self.pointsAreFlopped[0] = False
                    if objectCounter == 2:
                        self.NWTB.putNumber("VisionObject2Size", xSize)
                        self.NWTB.putNumber("VisionObject2Center", xCenter)
                        self.pointsAreFlopped[1] = False
                    if objectCounter == 3:
                        self.NWTB.putNumber("VisionObject3Size", xSize)
                        self.NWTB.putNumber("VisionObject3Center", xCenter)
                        self.pointsAreFlopped[2] = False
                    if objectCounter == 4:
                        self.NWTB.putNumber("VisionObject4Size", xSize)
                        self.NWTB.putNumber("VisionObject4Center", xCenter)
                        self.pointsAreFlopped[3] = False

                if rotationSwitcher == 1:
                    # Calculate the width of the object with reversed calculations.
                    xSize = abs(abs((object[0][0])) - abs((object[2][0])))
                    # Find center point on x axis.
                    xCenter = abs((object[2][0])) + xSize / 2
                    # Output values to NetworkTables for robot movement.
                    if objectCounter == 1:
                        self.NWTB.putNumber("VisionObject1Size", xSize)
                        self.NWTB.putNumber("VisionObject1Center", xCenter)
                        self.pointsAreFlopped[0] = True
                    if objectCounter == 2:
                        self.NWTB.putNumber("VisionObject2Size", xSize)
                        self.NWTB.putNumber("VisionObject2Center", xCenter)
                        self.pointsAreFlopped[1] = True
                    if objectCounter == 3:
                        self.NWTB.putNumber("VisionObject3Size", xSize)
                        self.NWTB.putNumber("VisionObject3Center", xCenter)
                        self.pointsAreFlopped[2] = True
                    if objectCounter == 4:
                        self.NWTB.putNumber("VisionObject4Size", xSize)
                        self.NWTB.putNumber("VisionObject4Center", xCenter)
                        self.pointsAreFlopped[3] = True

            # If tracking more than 1 object find the converging point of the extended object.
            if (self.numberOfObjects > 1):
                objectCounter = 0
                calculations = [[0,0], [0,0], [0,0], [0,0]]
                trackingPoints = [[0,0], [0,0], [0,0]]
                for points in self.pointArray:
                    # Increment object counter.
                    objectCounter += 1
                    # Find the objects outside extended lines.
                    if self.pointsAreFlopped[int(objectCounter) - 1] == True:
                        point1 = points[1]
                        point2 = points[2]
                        if point1[0] != point2[0]:
                            calculations[int(objectCounter) - 1][0] = (point1[1] - point2[1]) / (point1[0] - point2[0])
                            calculations[int(objectCounter) - 1][1] = point1[1] - (calculations[int(objectCounter) - 1][0] * point1[0])
                    if self.pointsAreFlopped[int(objectCounter) - 1] == False:
                        point1 = points[3]
                        point2 = points[2]
                        if point1[0] != point2[0]:
                            calculations[int(objectCounter) - 1][0] = (point1[1] - point2[1]) / (point1[0] - point2[0])
                            calculations[int(objectCounter) - 1][1] = point1[1] - (calculations[int(objectCounter) - 1][0] * point1[0])

                # Find the point of conversion for all the points.
                if calculations[0][0] != 0 and calculations[1][0] != 0:
                    trackingPoints[0][0] = (calculations[1][1] - calculations[0][1]) / (calculations[0][0] - calculations[1][0])
                    trackingPoints[0][1] = calculations[0][0] * trackingPoints[0][0] + calculations[0][1]
                if calculations[1][0] != 0 and calculations[2][0] != 0:
                    trackingPoints[1][0] = (calculations[2][1] - calculations[1][1]) / (calculations[1][0] - calculations[2][0])
                    trackingPoints[1][1] = calculations[1][0] * trackingPoints[1][0] + calculations[1][1]
                if calculations[2][0] != 0 and calculations[3][0] != 0:
                    trackingPoints[2][0] = (calculations[3][1] - calculations[2][1]) / (calculations[2][0] - calculations[3][0])
                    trackingPoints[2][1] = calculations[2][0] * trackingPoints[2][0] + calculations[2][1]

                # Filter out the converging points below the objects.
                pointCounter = 0
                pointInfo = [[0,0], [0,0]]
                for point in trackingPoints:
                    if point[1] > self.pointArray[0][0][1]:
                        point[0] = 0
                        point[1] = 0
                    # Write points distance and center of objects.
                    if point[0] != 0 and point[1] != 0:
                        pointInfo[int(pointCounter)][0] = abs(int(self.resolution[0]) - point[0])
                        pointInfo[int(pointCounter)][1] = point[0]
                        
                # Find the point with the least amount of distance to the center and output number.
                if int(pointInfo[0][0]) > int(pointInfo[1][0]):
                    self.trackingPoint = int(pointInfo[0][1])
                    self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)
                if int(pointInfo[1][0]) > int(pointInfo[0][0]):
                    self.trackingPoint = int(pointInfo[1][1])
                    self.NWTB.putNumber("VisionTrackingCenter", self.trackingPoint)

            # Check if the files or mode have changed and set trackbar values.
            if self.mode == "free":
                hmnReadFree = open("/home/pi/Desktop/Values/FreeMode/hmn.txt", "r")
                if (self.trackbarValues[0][0]) != hmnReadFree.read():
                    (self.trackbarValues[0][0]) = hmnReadFree.read()
                hmxReadFree = open("/home/pi/Desktop/Values/FreeMode/hmx.txt", "r")
                if (self.trackbarValues[0][1]) != hmxReadFree.read():
                    (self.trackbarValues[0][1]) = hmxReadFree.read()
                smnReadFree = open("/home/pi/Desktop/Values/FreeMode/smn.txt", "r")
                if (self.trackbarValues[1][0]) != smnReadFree.read():
                    (self.trackbarValues[1][0]) = smnReadFree.read()
                smxReadFree = open("/home/pi/Desktop/Values/FreeMode/smx.txt", "r")
                if (self.trackbarValues[1][1]) != smxReadFree.read():
                    (self.trackbarValues[1][1]) = smxReadFree.read()
                vmnReadFree = open("/home/pi/Desktop/Values/FreeMode/vmn.txt", "r")
                if (self.trackbarValues[2][0]) != vmnReadFree.read():
                    (self.trackbarValues[2][0]) = vmnReadFree.read()
                vmxReadFree = open("/home/pi/Desktop/Values/FreeMode/vmx.txt", "r")
                if (self.trackbarValues[2][1]) != vmxReadFree.read():
                    (self.trackbarValues[2][1]) = vmxReadFree.read()

            if self.mode == "tape":
                hmnReadTape = open("/home/pi/Desktop/Values/TapeMode/hmn.txt", "r")
                if (self.trackbarValues[0][0]) != hmnReadTape.read():
                    (self.trackbarValues[0][0]) = hmnReadTape.read()
                hmxReadTape = open("/home/pi/Desktop/Values/TapeMode/hmx.txt", "r")
                if (self.trackbarValues[0][1]) != hmxReadTape.read():
                    (self.trackbarValues[0][1]) = hmxReadTape.read()
                smnReadTape = open("/home/pi/Desktop/Values/TapeMode/smn.txt", "r")
                if (self.trackbarValues[1][0]) != smnReadTape.read():
                    (self.trackbarValues[1][0]) = smnReadTape.read()
                smxReadTape = open("/home/pi/Desktop/Values/TapeMode/smx.txt", "r")
                if (self.trackbarValues[1][1]) != smxReadTape.read():
                    (self.trackbarValues[1][1]) = smxReadTape.read()
                vmnReadTape = open("/home/pi/Desktop/Values/TapeMode/vmn.txt", "r")
                if (self.trackbarValues[2][0]) != vmnReadTape.read():
                    (self.trackbarValues[2][0]) = vmnReadTape.read()
                vmxReadTape = open("/home/pi/Desktop/Values/TapeMode/vmx.txt", "r")
                if (self.trackbarValues[2][1]) != vmxReadTape.read():
                    (self.trackbarValues[2][1]) = vmxReadTape.read()

            # Send telemtry data to webserver.
            seconds = int(time.time())
            if seconds % 2 == 0 and seconds != oldTime:
                telemetryMode = open("/home/pi/Desktop/Values/Telemetry/mode.txt", "w")
                telemetryMode.write(self.mode)
                telemetrySpeed = open("/home/pi/Desktop/Values/Telemetry/speed.txt", "w")
                telemetrySpeed.write(str(self.speed))
                telemetryPointArray = open("/home/pi/Desktop/Values/Telemetry/pointarray.txt", "w")
                telemetryPointArray.write(str(self.pointArray))
                telemetryTrackbars = open("/home/pi/Desktop/Values/Telemetry/trackbarvalues.txt", "w")
                telemetryTrackbars.write(str(self.trackbarValues))
                telemetryAreaMin = open("/home/pi/Desktop/Values/Telemetry/areamin.txt", "w")
                telemetryAreaMax = open("/home/pi/Desktop/Values/Telemetry/areamax.txt", "w")
                if self.gui == "no":
                    telemetryAreaMin.write(str(self.minArea.value))
                    telemetryAreaMax.write(str(self.maxArea.value))
                if self.gui == "yes":
                    telemetryAreaMin.write(str(self.trackbarValues[3][0]))
                    telemetryAreaMax.write(str(self.trackbarValues[3][1]))
                oldTime = seconds

            # Print debug info if the gui is disabled.
            if self.gui == "no":
                print "Vision Camera Res: " + str(self.resolution[0]) + "x" + str(self.resolution[1])
                print "Vision Mode: " + self.mode
                print "Vision Size: " + str(xSize)
                print "Vision Area: " + str(self.area)
                print "Vision Center: " + str(xCenter)
                print "Vision Rotation: " + str(self.rotation)
                print "Object Detected: " + str(self.objectDetected)
                print "Vision Frequency: " + str(self.speed)
                print "Vision Point Array: " + str(self.pointArray[0]) + str(self.pointArray[1])
                print "Vision Track-bar Values Array: " + str(self.trackbarValues)

            # Add delay so it doesn't flood the network.
            time.sleep(0.05)

    def stop(self):
        self.stopped = True
