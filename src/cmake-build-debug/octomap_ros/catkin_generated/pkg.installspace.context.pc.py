# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/ros/noetic/include".split(';') if "${prefix}/include;/opt/ros/noetic/include" != "" else []
PROJECT_CATKIN_DEPENDS = "sensor_msgs;tf;octomap_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-loctomap_ros;/opt/ros/noetic/lib/liboctomap.so;/opt/ros/noetic/lib/liboctomath.so".split(';') if "-loctomap_ros;/opt/ros/noetic/lib/liboctomap.so;/opt/ros/noetic/lib/liboctomath.so" != "" else []
PROJECT_NAME = "octomap_ros"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.4.0"
