#!/usr/bin/env python3
import rospy
from typing import List
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, vector_norm
from readers    import *
from enums      import *
from utils.utils      import *
import time
import math
import numpy as np
from utils.pathPlanner  import path_plan
from nav_msgs.msg import Odometry

currentWaypoint = 0

class Follower:
    def __init__(self,topic : str,heightFactor = 1):
        self.current_x = .0
        self.current_y = .0
        self.current_z = .0
        self.desired_x = .0
        self.desired_y = .0
        self.desired_z = .0
        self.id = topic
        self.heightFactor = heightFactor
        self.current_yaw = quaternion_from_euler(0, 0, 0)
        self.position_target_pub = rospy.Publisher(f'/{topic}/command/pose', PoseStamped, queue_size=10)
        self.path = Path()
        rospy.Subscriber(f"/{topic}/odometry_sensor1/position", PointStamped, self.position_callback)
        rospy.Subscriber(f'/{topic}/odometry_sensor1/odometry', Odometry, self.odom_cb)
        self.pathPublisher = rospy.Publisher(f'/{topic}/path', Path, queue_size=10)

    def position_callback(self, data):
        self.current_x = round2f(data.point.x)
        self.current_y = round2f(data.point.y)
        self.current_z = round2f(data.point.z)
        self.path.header = data.header
        
    
    def odom_cb(self, data):
        if(currentWaypoint > 0):
            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            self.path.poses.append(pose)
            self.pathPublisher.publish(self.path)

    def ready(self) -> bool:
        return euclidianDistance(
            self.current_x, self.desired_x,
            self.current_y, self.desired_y,
            self.current_z, self.desired_z
        ) < .2

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        self.position_target_pub.publish(self.set_orientation(yaw_degree))

    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    def get_location(self) -> dict:
        return [
            self.current_x,
            self.current_y,
            self.current_z
        ]

    def set_pose(self, x=0, y=0, z=2, bodyFlu = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if bodyFlu:
            # pose.header.frame_id = 'base_link'
            pose.header.frame_id = 'auto'

        else:
            pose.header.frame_id = 'map'
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.desired_x = x
        self.desired_y = y
        self.desired_z = z
        # pose.pose.orientation = Quaternion(*self.current_yaw)
        return pose


    def getHeightFactor(self) -> int:
        return self.heightFactor

class Commander:

    AVAILABLE_DRONES    = ("gs0","gs1","gs2","gs3","gs4","gs5","gs6","gs7","gs8")
    CONSTANT_Z = .8

    def __init__(self,instructions : list,looping = False):
        if not rospy.core.is_initialized():
            rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.loopDrones = looping
        self.numDrones = len(instructions)
        self.drones : List[Follower] = []
        for x in range(self.numDrones):
            self.drones.append(Follower(Commander.AVAILABLE_DRONES[x],x+1))
        self.currentWaypoint = 0
        self.distribuiteWaypoints(instructions)

    def nextPosition(self) -> int:
        global currentWaypoint
        if(self.currentWaypoint >= len(self.wayPoints[0])):
            return -1
        i = 0
        currentWaypoint = self.currentWaypoint
        if(self.currentWaypoint == 0):
            self.separeteCoords()
        else:
            for d in self.drones:
                if(d.id == Commander.AVAILABLE_DRONES[0]):
                    d.get_location()
                waypoints = self.wayPoints[i][self.currentWaypoint]
                if(len(waypoints) == 3):
                    d.move(waypoints[0], waypoints[1], waypoints[2])
                else:
                    d.move(waypoints[0], waypoints[1], Commander.CONSTANT_Z * d.getHeightFactor())
                i += 1
        self.currentWaypoint += 1
        if(self.currentWaypoint >= len(self.wayPoints[0]) and self.loopDrones):
            self.currentWaypoint = 0
        return self.currentWaypoint

    def getAllPositions(self):
        return [dronePosition.get_location() for dronePosition in self.drones]

    def waitForDrones(self) -> bool:
        for dron in self.drones:
            if not dron.ready():
                return True
        return False

    def waitForSelectedDrones(self, doNotCheck) -> bool:
        for i in range(len(self.drones)):
            if not i in doNotCheck and not self.drones[i].ready():
                return True
        return False

    def separeteCoords(self):
        points =  self.getAllPositions()
        collides = True
        #Gets posistions for drones in z axis so they dont collide
        while(collides):
            pointsChanged = False
            for i in range(len(self.drones)):
                for j in range(i+1,len(self.drones)):
                    distance = self.calculate2dDistance(points[i][0], points[j][0], points[i][1], points[j][1])
                    while(distance < 0.6):
                        points[i][0] = points[i][0] + 0.3 + distance
                        points[i][1] = points[i][1] + 0.3 + distance
                        pointsChanged = True
                        distance = self.calculate2dDistance(points[i][0], points[j][0], points[i][1], points[j][1])
            if(not pointsChanged): collides = False
        
        for i in range(len(self.drones)):
            self.drones[i].move(points[i][0],points[i][1], points[i][2])
        while(self.waitForDrones()):
            time.sleep(.5)
        for i in range(len(self.drones)):
            self.drones[i].move(points[i][0],points[i][1], self.wayPoints[i][self.currentWaypoint][2])
        while(self.waitForDrones()):
            time.sleep(.5)


        #Creates a queue for drones who might collide
        points =  self.getAllPositions()
        wait = []
        for i in range(len(self.drones)):
            desiredLocations = self.wayPoints[i][self.currentWaypoint]
            for j in range(i+1,len(self.drones)):
                #Checks that drones are in same or a very close z axis
                if(abs(points[i][2] -points[j][2]) < 0.6):
                    #Calculates the point in which the trayectories will collide
                    x = [points[i][0],desiredLocations[0],points[j][0],self.wayPoints[j][self.currentWaypoint][0]]
                    y = [points[i][1],desiredLocations[1],points[j][1],self.wayPoints[j][self.currentWaypoint][1]]
                    denominador = (y[3]-y[2])*(x[1]-x[0])-(x[3]-x[2])*(y[1]-y[0])
                    numerador1 = (x[3]-x[2])*(y[0]-y[2])-(y[3]-y[2])*(x[0]-x[2])
                    numerador2 = (x[1]-x[0])*(y[0]-y[2])-(y[1]-y[0])*(x[0]-x[2])
                    if (denominador != 0):
                        ua = numerador1 / denominador
                        ub = numerador2 / denominador
                        if(ua >= 0 and ua <= 1 and ub >= 0 and ub <= 1):
                            ix = x[0]+ua*(x[1]-x[0])
                            iy = y[0]+ua*(y[1]-y[0])
                            #Calculates if the distances between the collision point and the starting and ending point
                            distanceI = self.calculate2dDistance(points[i][0], ix, points[i][1], iy)
                            distaceJ = self.calculate2dDistance(points[j][0], ix, points[j][1], iy)
                            distanceEI = self.calculate2dDistance(desiredLocations[0], ix, desiredLocations[1], iy)
                            distanceEJ = self.calculate2dDistance(self.wayPoints[j][self.currentWaypoint][0], ix, self.wayPoints[j][self.currentWaypoint][1], iy)
                            #Calculates time to arrive to collision point
                            timeI = distanceI / 2.1
                            timeJ = distaceJ / 2.1
                            #If time is very similar or 
                            if(distanceEI < 0.2 or abs(timeI - timeJ) < 0.3):
                                wait.append(i)
                                if (distanceI < 0.2 or distanceEJ < 0.2):
                                    wait.append(j)


        for i in range(len(self.drones)):
            desiredLocations = self.wayPoints[i][self.currentWaypoint]
            if(not i in wait):
                self.drones[i].move(desiredLocations[0],desiredLocations[1], desiredLocations[2])

        while(self.waitForSelectedDrones(wait)):
            time.sleep(.5)

        for droneW in wait:
            time.sleep(.5)
            desiredLocations = self.wayPoints[droneW][self.currentWaypoint]
            self.drones[droneW].move(desiredLocations[0],desiredLocations[1], desiredLocations[2])

            
    def calculate2dDistance(self, x1, x2, y1, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def distribuiteWaypoints(self,instructions : list):
            """
            Assign the Order of the Waypoints so the minimum distance is to be traveled.

            Args:
                instructions (list): Desired Waypoints
            """
            dronesPositions = self.getAllPositions()
            nextPositions = [inst[0] for inst in instructions]
            order = path_plan(dronesPositions, nextPositions)
            self.wayPoints = []
            for inst in range(len(instructions)):
                self.wayPoints.append(instructions[order[0][inst]])

if __name__ == "__main__":

    option = -1
    try:
        option = int(input("Choose the option that you want:\n\t1. Read from CSV\n\t2. Read from ImgFile\n\t3. Read from WebCamera\n\t4. Read from DroneCamera \n\t5. Random Positions \n"))
        if option < 1 or option > 4:
            raise Exception("Not an option")
    except Exception:
        print("please pick a valid option")

    
    instructions = []
    reader = Reader()

    if(option == 1): # CSV
        reader = CsvFileReader()
    elif(option == 2): # Img PNG
        reader = ImgFileReader()
    elif(option == 3): # WebCam
        reader = WebCamReader()
    elif(option == 4): # Drone Camera
        reader = DronCamReader()
    elif(option == 5): # Random Position Generator
        reader = RandomPositionReader()

    instructions = reader.readInstructions()

    print(instructions)

    c = Commander(instructions, False)
    time.sleep(1)
    stop = False
    moving = False
    try:
        while(not stop):
            if not moving:
                moving = True
                stop = c.nextPosition() < 0
            elif c.waitForDrones():
                time.sleep(.5)
            else:
                moving = False
    except KeyboardInterrupt:
        pass
    print("Recorrido concluido")