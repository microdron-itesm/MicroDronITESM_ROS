//
// Created by ajahueym on 22/08/20.
//
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <mavlink2/standard/mavlink.h>
#include "MAVLinkIMUInterface.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <thread>

extern "C" {
#include <UDP.h>
}

int main(int argc, char **argv){
    ros::init(argc, argv, "mavlink_imu");
    ros::NodeHandle nodeHandle;
    std::string droneName;

    if(!nodeHandle.getParam("mav_name", droneName)){
        droneName = "ardrone";
        ROS_WARN("mavlink_imu was not given a mav_name! Using \"ardrone\" as default");
    }

    size_t bufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);
    uint8_t buf[bufLen];

    udp_conn_data data;
    udp_conn_open_ip(&data, "localhost", 14552, 14553);

    mavlink_attitude_quaternion_t attitude;
    MAVLinkIMUInterface imuInterface(droneName);

    ros::Rate rate(60);
    while(ros::ok()){
        memset(&buf, 0, sizeof(buf));
        attitude = imuInterface.getAttitude();
        mavlink_message_t msg;
        mavlink_msg_attitude_quaternion_pack(1, 200, &msg, attitude.time_boot_ms, attitude.q1, attitude.q2, attitude.q3, attitude.q4, attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed, attitude.repr_offset_q);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        udp_conn_send(&data, buf, len);
        ros::spinOnce();
        rate.sleep();
    }

    udp_conn_close(&data);

    return 1;
}
