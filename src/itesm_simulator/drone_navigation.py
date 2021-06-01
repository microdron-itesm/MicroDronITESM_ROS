#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped,Transform, Quaternion, Point, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler, vector_norm
from readers    import *
from utils.flightConfig     import *
from utils.utils            import *
from utils.pathPlanner      import path_plan
from random                 import shuffle
import time
import math
import numpy as np

from dataclasses import dataclass

@dataclass(frozen=True, order=True)
class Point:
    """
    Hashable 3D Point data structure
    """
    x   : float
    y   : float
    z   : float

def toPoint(arr : list) -> Point:
    return Point(arr[0], arr[1], arr[2])

def acceptableDif(a,b):
    return abs(a - b) < 1

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
        rospy.Subscriber(f"/{topic}/odometry_sensor1/position", PointStamped, self.position_callback)

    def position_callback(self, data):
        self.current_x = round2f(data.point.x)
        self.current_y = round2f(data.point.y)
        self.current_z = round2f(data.point.z)
        # for d in self.observing:
        #     if(d in positions):
        #         distance = euclidianDistance(
        #             self.current_x, positions[d][0],
        #             self.current_y, positions[d][1], 
        #             self.current_z, positions[d][2]
        #         )
        #         if(distance > 0.5 and self.stored.get(d) != None and len(self.stored.keys()) == 1):
        #             print(distance,self.id,d, 'reseting')
        #             self.move(self.stored[d][0],self.stored[d][1],self.stored[d][2])
        #             self.observing.remove(d)
        #             del self.stored[d]
        #             print(self.stored)
        #         elif(distance < 1.0 and (self.stored.get(d) == None)):
        #             print(distance,self.id,d, 'less')
        #             self.stored[d] = [self.desired_x,self.desired_y,self.desired_z]
        #             self.move(positions[d][0]+positions[d][0],positions[d][1]+positions[d][1],self.current_z)
        #         elif(distance > 1. and self.stored.get(d) != None):
        #             print(distance,self.id,d, 'reseting')
        #             self.move(self.stored[d][0],self.stored[d][1],self.stored[d][2])
        #             self.observing.remove(d)
        #             del self.stored[d]
        #             print(self.stored)

    def ready(self) -> bool:
        return euclidianDistance(
            self.current_x, self.desired_x,
            self.current_y, self.desired_y,
            self.current_z, self.desired_z
        ) < .4

    def move(self,  BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(BODY_OFFSET_ENU))

    def moveTo(self, x, y, z):
        self.set_desired(x, y, z)
        self.move()

    def turn(self, yaw_degree):
        self.position_target_pub.publish(self.set_orientation(yaw_degree))

    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    def get_location(self) -> list:
        return [
            self.current_x,
            self.current_y,
            self.current_z
        ]

    def get_location_as_point(self) -> Point:
        return toPoint(self.get_location())

    def get_desired_location(self) -> list:
        return [
            self.desired_x,
            self.desired_y,
            self.desired_z
        ]

    def get_desired_location_as_point(self) -> Point:
        return toPoint(self.get_desired_location())

    def setObservations(self, obsverving, obsvered=False):
        self.observing = self.observing + obsverving
        if(not self.observed): self.observed = obsvered

    def set_desired(self, x, y, z):
        self.desired_x = x
        self.desired_y = y
        self.desired_z = z

    def set_pose(self, bodyFlu = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if bodyFlu:
            # pose.header.frame_id = 'base_link'
            pose.header.frame_id = 'auto'

        else:
            pose.header.frame_id = 'map'
        
        pose.pose.position.x = self.desired_x
        pose.pose.position.y = self.desired_y
        pose.pose.position.z = self.desired_z
        # pose.pose.orientation = Quaternion(*self.current_yaw)
        return pose

    def getHeightFactor(self) -> int:
        return self.heightFactor

    @staticmethod
    def littleAdjustment(a,b):
        return all([
            acceptableDif(a.x, b.x),
            acceptableDif(a.y, b.y),
            acceptableDif(a.z, b.z)
        ])

    @staticmethod
    def getMovementRangeHelper(a,b) -> list:
        return list(range(a, b, 1 if (a - b) < 0 else -1))

    @staticmethod
    def getMovementRange(a,b) -> list:
        tempInit = int(a)
        tempFinit= int(b)
        proposals = Follower.getMovementRangeHelper(tempInit, tempFinit)
        if len(proposals) > 0:
            if proposals[0] == a:
                proposals.pop(0)
            if len(proposals) == 0 or proposals[-1] != b:
                proposals.append(b)
        return proposals

    def _deadlockHelper(self, lockedPositions : set, tries = 0):
        # We tried everything, so don't move
        if tries > 5:
            return False, {}, None
        randValue = int(time.time())
        desiredPosition = None
        selfPositionAsInt = self.get_location()
        # selfPositionAsInt = [int(i) for i in self.get_location()]
        if(randValue % 5 == 0):
            desiredPosition = Point(selfPositionAsInt[0], selfPositionAsInt[1], int(selfPositionAsInt[2] + 1))
        elif(randValue % 3 == 0):
            desiredPosition = Point(selfPositionAsInt[0], int(selfPositionAsInt[1] + 1), selfPositionAsInt[2])
        else:
            desiredPosition = Point(int(selfPositionAsInt[0] + 1), selfPositionAsInt[1], selfPositionAsInt[2])

        if desiredPosition in lockedPositions:
            return self._deadlockHelper(lockedPositions, tries + 1)
        
        return True, {desiredPosition}, desiredPosition

    def _breakDeadlockProtocol(self, currentPoint, lockedPositions : set, movementProposals : list):
        availableDistances = [len(proposals) for proposals in movementProposals]
        
        # The Drones can't move and we might be on a deadlock
        if all([distance == 0 for distance in availableDistances]):
            return self._deadlockHelper(lockedPositions)

        maxTravelDistance = availableDistances[availableDistances.index(max(availableDistances))]

        return True, set(maxTravelDistance), maxTravelDistance[-1]
            
    def plan_movement(self, lockedPositions : set):
        """
        Function to plan the movement for this drone to the next desired position.

        Args:

            nextPosition (Point): Next Point for this drone to be

            lockedOriginalPoints (set): Locked Positions that the drones are originally at.

            lockedPositions (set): Positions locked by movements of other drones

        Returns:

            bool: Whether or not the drone will move during this transition period,

            set: Set of locked positions that the drone during movement will use,

            Point: Desired Available WayPoint for the drone to move during this transition period
        """
        nextPosition = self.get_desired_location_as_point()
        currentPoint = self.get_location_as_point()
        # Already at the desired position
        if Follower.littleAdjustment(currentPoint, nextPosition):
            return False, {}, None

        validPointsX = []
        validPointsY = []
        validPointsZ = []

        # We should move on the Z axis
        if not acceptableDif(nextPosition.z, currentPoint.z):
            proposals = Follower.getMovementRange(currentPoint.z,nextPosition.z)
            for propAxis in proposals:
                propPoint = Point(currentPoint.x, currentPoint.y, propAxis)
                if propPoint not in lockedPositions:
                    validPointsZ.append(propPoint)
                else:
                    break
            if len(proposals) > 0 and len(proposals) == len(validPointsZ): # Good to go
                return True, set(validPointsZ), validPointsZ[-1]

        # We should move on the X axis
        if not acceptableDif(nextPosition.x, currentPoint.x):
            proposals = Follower.getMovementRange(currentPoint.x,nextPosition.x)
            for propAxis in proposals:
                propPoint = Point(propAxis,currentPoint.y,currentPoint.z)
                if propPoint not in lockedPositions:
                    validPointsX.append(propPoint)
                else:
                    break
            if len(proposals) > 0 and len(proposals) == len(validPointsX): # Good to go
                return True, set(validPointsX), validPointsX[-1]

        # We should move on the Y axis
        if not acceptableDif(nextPosition.x, currentPoint.x):
            proposals = Follower.getMovementRange(currentPoint.y,nextPosition.y)
            for propAxis in proposals:
                propPoint = Point(currentPoint.x, propAxis, currentPoint.z)
                if propPoint not in lockedPositions:
                    validPointsY.append(propPoint)
                else:
                    break
            if len(proposals) > 0 and len(proposals) == len(validPointsY): # Good to go
                return True, set(validPointsY), validPointsY[-1]
        
        return self._breakDeadlockProtocol(currentPoint, lockedPositions, [
            validPointsZ,
            validPointsX,
            validPointsY
        ])

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
        # self.takeOff()


    def _strategicPositioning(self):

        currentPositionsSet = set([toPoint(point) for point in self.getAllPositions()])

        dronesOrder = list(self.drones)

        shuffle(dronesOrder)

        for d in dronesOrder:
            willMove, newPaths, finalPoint = d.plan_movement(currentPositionsSet)
            if willMove:
                d.set_desired(finalPoint.x, finalPoint.y, finalPoint.z)
                currentPositionsSet.remove(d.get_location_as_point())
            
            currentPositionsSet.update(newPaths)

        for d in self.drones:
            d.move()
        
        return 0

    def nextPosition(self) -> int:
        if(self.currentWaypoint >= len(self.wayPoints[0])):
            return -1

        i = 0
        for d in self.drones:
            if(d.id == Commander.AVAILABLE_DRONES[0]):
                d.get_location()
            waypoints = self.wayPoints[i][self.currentWaypoint]
            # self.separeteCoords()
            if(len(waypoints) == 3):
                d.set_desired(waypoints[0], waypoints[1], waypoints[2])
            else:
                d.set_desired(waypoints[0], waypoints[1], Commander.CONSTANT_Z * d.getHeightFactor())
            i += 1

        if self.currentWaypoint == 0:
            if not all([d.ready() for d in self.drones]):
                return self._strategicPositioning()
            else:
                self.currentWaypoint = 1
                return self.nextPosition()

        for d in self.drones:
                d.move()
        
        self.currentWaypoint += 1
        if(self.currentWaypoint > len(self.wayPoints[0]) and self.loopDrones):
            self.currentWaypoint = 0
        return self.currentWaypoint

    def takeOff(self):
        for d in self.drones:
            point = d.get_location_as_point()
            if acceptableDif(point.z, 0):
                d.moveTo(point.x, point.y, point.z + 1)

    def getAllPositions(self):
        return [dronePosition.get_location() for dronePosition in self.drones]

    def waitForDrones(self) -> bool:
        for dron in self.drones:
            if not dron.ready():
                return True
        return False

    def separeteCoords(self):
        points =  self.getAllPositions()
        for i in range(len(self.drones)):
            for j in range(i+1,len(self.drones)):
                distance = self.calculate2dDistance(points[i][0], points[j][0], points[i][1], points[j][1])
                while(distance < 1):
                    points[i][0] = points[i][0] + 0.3
                    points[i][1] = points[i][1] + 0.3
                    distance = self.calculate2dDistance(points[i][0], points[j][0], points[i][1], points[j][1])
            self.drones[i].move(points[i][0],points[i][1], points[i][2])
        while(self.waitForDrones()):
            time.sleep(.5)
        for i in range(len(self.drones)):
            self.drones[i].move(points[i][0],points[i][1], self.wayPoints[i][self.currentWaypoint][2])
        while(self.waitForDrones()):
            time.sleep(.5)
        for i in range(len(self.drones)):
            self.drones[i].move(self.wayPoints[i][self.currentWaypoint][0],self.wayPoints[i][self.currentWaypoint][1], self.wayPoints[i][self.currentWaypoint][2])

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
        option = int(input("Choose the option that you want:\n\t1. Read from CSV\n\t2. Read from ImgFile\n\t3. Read from WebCamera\n\t4. Read from DroneCamera\n\t5. Random Positions\n\t6. Specific File (JSON)  \n"))
        if option < 1 or option > 6:
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
    elif(option == 6): # Specific File Name
        reader = JSONReader()

    instructions = reader.readInstructions()

    c = Commander(instructions, False)
    time.sleep(2)
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