#!/usr/bin/env python
from networktables import NetworkTables
import time
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


def parseIMUData(data):
	temp = data.replace("\r"," ")
	temp = data.replace("\n"," ")

	start = temp.find(",")

	clean = temp[start + 1:len(data) - 5]

	pose = clean.split()

        res = { "qw": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "z": 0.0}


	if len(pose) == 4:
        	yaw = float(pose[0])
        	roll = float(pose[2])
        	pitch = float(pose[1])
        	z = float(pose[3])


        	cy = math.cos(yaw * 0.5)
        	sy = math.sin(yaw * 0.5)

        	cp = math.cos(pitch * 0.5)
        	sp = math.sin(pitch * 0.5)

        	cr = math.cos(roll * 0.5)
        	sr = math.sin(roll * 0.5)

		res["qw"] = cr * cp * cy + sr * sp * sy
		res["qx"] = sr * cp * cy - cr * sp * sy
		res["qy"] = cr * sp * cy + sr * cp * sy
		res["qz"] = cr * cp * sy - sr * sp * cy
		res["z"] = z

	return res

def consumer():
	NetworkTables.initialize(server="35.224.79.186")
	table = NetworkTables.getTable("IMU_DATA")

	rospy.init_node("consumer", anonymous=True)

	state_msg = ModelState()
	state_msg.model_name = "ardrone_static"
	state_msg.pose.position.x = 0
    	state_msg.pose.position.y = 0

	rate = rospy.Rate(100)
	rospy.wait_for_service("/gazebo/set_model_state")

	while not rospy.is_shutdown():
		try:
			data = table.getString("serial_data", "-")
			print data
			parsed_data = parseIMUData(data)

			state_msg.pose.position.z = parsed_data["z"]
    			state_msg.pose.orientation.x = parsed_data["qx"]
    			state_msg.pose.orientation.y = parsed_data["qy"]
    			state_msg.pose.orientation.z = parsed_data["qz"]
			state_msg.pose.orientation.w = parsed_data["qw"]

			set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
			resp = set_state(state_msg)
		except rospy.ServiceException, e:
			print "Service call failed: {}".format(e)

		rate.sleep()

if __name__ == '__main__':
	try:
		consumer()
	except rospy.ROSInterruptException:
		pass

