import rospy 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler
import time
import math

z = .5

class Commander:
    def __init__(self,topic):
        rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.current_x = 0
        self.current_y = 0
        self.current_z = .5
        self.current_yaw = quaternion_from_euler(0, 0, 0)
        self.position_target_pub = rospy.Publisher(f'/{topic}/command/pose', PoseStamped, queue_size=10)
        self.position_target_pub.publish(self.set_pose(self.current_x , self.current_y, self.current_z, True))


    def move(self, x, y, z, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        self.position_target_pub.publish(self.set_orientation(yaw_degree))

    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))


    def set_pose(self, x=0, y=0, z=2, BODY_FLU = True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            pose.header.frame_id = 'base_link'

        else:
            pose.header.frame_id = 'map'
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = Quaternion(*self.current_yaw)

        self.current_x = x
        self.current_y = y
        self.current_z = z
        

        return pose

    def set_orientation(self, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        if BODY_FLU:
            pose.header.frame_id = 'base_link'

        else:
            pose.header.frame_id = 'map'


        DEG_2_RAD = math.pi / 180.0
        pose.pose.position.x = self.current_x
        pose.pose.position.y = self.current_y
        pose.pose.position.z = self.current_z

        
        q = quaternion_from_euler(yaw*DEG_2_RAD, 0, yaw*DEG_2_RAD)
        print(Quaternion(*q))
        pose.pose.orientation = Quaternion(*q)

        self.current_yaw = q

        return pose

if __name__ == "__main__":
    
    delay = 7
    extension = 2
    con1 = Commander("gs0")
    con2 = Commander("gs1")
    time.sleep(2)
    con1.move(1, 1, 1)
    time.sleep(2)
    con2.move(-1,-2.5,.5)
    time.sleep(5)
    con2.move(0,-.5,.5)
    # time.sleep(delay)
    # con.move(-extension, 0, z)
    # time.sleep(delay)
    # con.move(0, 0, z)
    # time.sleep(delay)
    # con.move(0, -extension, z)

