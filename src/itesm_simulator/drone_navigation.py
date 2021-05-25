#!/usr/bin/env python3
import rospy
from typing import List
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler
from readers    import *
from enums      import *
from utils.utils      import *
import time
import math

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

    def ready(self) -> bool:
        return euclidianDistance(
            self.current_x, self.desired_x,
            self.current_y, self.desired_y,
            self.current_z, self.desired_z
        ) < .1

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        self.position_target_pub.publish(self.set_orientation(yaw_degree))

    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    def get_location(self) -> dict:
        return {
            "x" : self.current_x,
            "y" : self.current_y,
            "z" : self.current_z
        }


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

    AVAILABLE_DRONES    = ("gs0","gs1","gs2","gs3","gs4")
    CONSTANT_Z = .8

    def __init__(self,instructions : list,looping = False):
        if not rospy.core.is_initialized():
            rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.wayPoints = instructions
        self.loopDrones = looping
        self.numDrones = len(instructions)
        self.drones : List[Follower] = []
        for x in range(self.numDrones):
            self.drones.append(Follower(Commander.AVAILABLE_DRONES[x],x+1))
        self.currentWaypoint = 0

    def nextPosition(self) -> int:
        if(self.currentWaypoint >= len(self.wayPoints[0])):
            return -1
        i = 0
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