import rospy 
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler
from readers    import *
from enums      import *
import time
import math

def callback(data):
    print(f"x = {round2f(data.point.x)}")
    print(f"y = {round2f(data.point.y)}")
    print(f"z = {round2f(data.point.z)}")

if __name__ == "__main__":
    rospy.init_node('odomListner', anonymous=True)
    rospy.Subscriber("/gs2/odometry_sensor1/position", PointStamped, callback)
    rospy.spin()