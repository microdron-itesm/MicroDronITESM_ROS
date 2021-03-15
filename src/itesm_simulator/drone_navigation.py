import rospy 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler
import time
import math

class Follower:
    def __init__(self,x,y,z,topic):
        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_yaw = quaternion_from_euler(0, 0, 0)
        self.position_target_pub = rospy.Publisher(f'/{topic}/command/pose', PoseStamped, queue_size=10)

    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        self.position_target_pub.publish(self.set_orientation(yaw_degree))

    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))


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
        # pose.pose.orientation = Quaternion(*self.current_yaw)

        self.current_x = x
        self.current_y = y
        self.current_z = z
        

        return pose

    # def set_orientation(self, yaw):
    #     pose = PoseStamped()
    #     pose.header.stamp = rospy.Time.now()
    #     if BODY_FLU:
    #         pose.header.frame_id = 'base_link'

    #     else:
    #         pose.header.frame_id = 'map'


    #     DEG_2_RAD = math.pi / 180.0
    #     pose.pose.position.x = self.current_x
    #     pose.pose.position.y = self.current_y
    #     pose.pose.position.z = self.current_z

        
    #     q = quaternion_from_euler(yaw*DEG_2_RAD, 0, yaw*DEG_2_RAD)
    #     print(Quaternion(*q))
    #     pose.pose.orientation = Quaternion(*q)

    #     self.current_yaw = q

    #     return pose

class Commander:

    AVAILABLE_DRONES    = ("gs0","gs1")
    INITIAL_POSITIONS   = (
        (0,0,0),
        (4,0,0)
    )
    CONSTANT_Z = .8

    def __init__(self,instructions : list,looping = False):
        rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.wayPoints = instructions
        self.loopDrones = looping
        self.numDrones = len(instructions)
        self.drones = []
        for x in range(self.numDrones):
            t = Commander.INITIAL_POSITIONS[x]
            self.drones.append(Follower(t[0],t[1],t[2],Commander.AVAILABLE_DRONES[x]))
        self.currentWaypoint = 0

    def nextPosition(self) -> int:
        if(self.currentWaypoint >= len(self.wayPoints[0])):
            return -1
        i = 0
        for d in self.drones:
            waypoints = self.wayPoints[i][self.currentWaypoint]
            # d.move(waypoints[0],waypoints[1],waypoints[2])
            d.move(waypoints[0],waypoints[1],Commander.CONSTANT_Z)
            i += 1
        self.currentWaypoint += 1
        if(self.currentWaypoint >= len(self.wayPoints[0]) and self.loopDrones):
            self.currentWaypoint = 0
        return self.currentWaypoint


def readInstructions() -> list:
    f = open("visualize.csv","r")
    instructions = f.readlines()
    f.close()
    result = []
    for x in instructions:
        s = x.strip().split(",")
        temp = []
        for y in s:
            tempFactors = y.split(";")
            if(len(tempFactors) > 1):
                temp.append(tuple([float(z) for z in tempFactors]))
        result.append(tuple(temp))
    return result

if __name__ == "__main__":

    instructions = readInstructions()

    c = Commander(instructions)
    time.sleep(1)
    try:
        while(c.nextPosition() >= 0):
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    print("Recorrido concluido")