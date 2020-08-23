//
// Created by ajahueym on 22/08/20.
//

#include "MAVLinkIMUInterface.h"

MAVLinkIMUInterface::MAVLinkIMUInterface(const std::string& mav_name) : mav_name(mav_name){
     imuSub = nodeHandle.subscribe("/" + mav_name + "/imu", 1000, &MAVLinkIMUInterface::onImuMessageReceived, this);
}

void MAVLinkIMUInterface::onImuMessageReceived(const sensor_msgs::Imu_<std::allocator<void>>::ConstPtr &msg) {
    attitude.q1 = msg->orientation.w;
    attitude.q2 = msg->orientation.x;
    attitude.q3 = msg->orientation.y;
    attitude.q4 = msg->orientation.z;

    attitude.yawspeed = msg->angular_velocity.z;
    attitude.pitchspeed = msg->angular_velocity.y;
    attitude.rollspeed = msg->angular_velocity.x;
}

const mavlink_attitude_quaternion_t &MAVLinkIMUInterface::getAttitude() const {
    return attitude;
}
