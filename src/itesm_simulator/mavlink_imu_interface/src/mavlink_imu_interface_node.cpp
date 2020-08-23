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

int main(int argc, char **argv){
    ros::init(argc, argv, "mavlink_imu");
    ros::NodeHandle nodeHandle;
    std::string droneName;

    if(!nodeHandle.getParam("mav_name", droneName)){
        droneName = "ardrone";
        ROS_WARN("mavlink_imu was not given a mav_name! Using \"ardrone\" as default");
    }

    struct sockaddr_in localAddress{};
    const uint16_t imu_send_port = 14551;
    int sockFd; // Socket file descriptor

    sockFd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sockFd < 0){
        perror("Socket open failed");
        exit(EXIT_FAILURE);
    }

    memset(&localAddress, 0, sizeof(localAddress));
    localAddress.sin_family = AF_INET;
    localAddress.sin_addr.s_addr = INADDR_ANY;
    localAddress.sin_port = htons(imu_send_port);

    int ret = fcntl(sockFd, F_SETFL, O_NONBLOCK | O_ASYNC);
    if(ret < 0){
        perror("Nonblocking set failed");
        close(sockFd);
        exit(EXIT_FAILURE);
    }

    size_t bufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);
    uint8_t buf[bufLen];

    mavlink_attitude_quaternion_t attitude;
    MAVLinkIMUInterface imuInterface(droneName);
    while(ros::ok()){
        memset(&buf, 0, sizeof(buf));
        attitude = imuInterface.getAttitude();
        mavlink_message_t msg;
        mavlink_msg_attitude_quaternion_pack(1, 200, &msg, attitude.time_boot_ms, attitude.q1, attitude.q2, attitude.q3, attitude.q4, attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed, attitude.repr_offset_q);
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        sendto(sockFd, buf, len, 0, (struct sockaddr*) &localAddress, sizeof(struct sockaddr_in));
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 1;
}
