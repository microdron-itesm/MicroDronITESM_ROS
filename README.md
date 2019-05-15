# MicroDronITESM_ROS
Clone with:
```
git clone --recurse-submodules
```
Make sure to install the needed dependencies
On Ubuntu: 
```
sudo apt-get install ros-melodic-desktop-full ros-melodic-octomap libgoogle-glog-dev
```

This is used to test the drone control code used in the MicroDron.

To compile the project use:
```
catkin_make
```
To run the simulation use:
```
roslaunch rotors_gazebo mav.launch mav_name:=iris world_name:=basic paused:=false
```

To run the control node use:
```
rosrun drone_control drone_control_node
```
