//
// Created by ajahueym on 22/08/20.
//

#include "MAVLinkInterface.h"

MAVLinkInterface::MAVLinkInterface(const std::string& mav_name) : mav_name(mav_name){
    imuSub = nodeHandle.subscribe("/" + mav_name + "/imu", 1000, &MAVLinkInterface::onImuMessageReceived, this);
    irSub = nodeHandle.subscribe("/" + mav_name + "/ir", 1000, &MAVLinkInterface::onTofMessageReceived, this);
    attitude.time_boot_ms = 0;
}

const mavlink_attitude_quaternion_t &MAVLinkInterface::getAttitude() const {
    return attitude;
}

const mavlink_distance_sensor_t &MAVLinkInterface::getDistanceSensor() const {
    return distanceSensor;
}

void MAVLinkInterface::onImuMessageReceived(const sensor_msgs::Imu_<std::allocator<void>>::ConstPtr &msg) {
    attitude.q1 = msg->orientation.w;
    attitude.q2 = msg->orientation.x;
    attitude.q3 = msg->orientation.y;
    attitude.q4 = msg->orientation.z;

    attitude.yawspeed = msg->angular_velocity.z;
    attitude.pitchspeed = msg->angular_velocity.y;
    attitude.rollspeed = msg->angular_velocity.x;
}

void MAVLinkInterface::onTofMessageReceived(const sensor_msgs::Range::ConstPtr& msg) {
    distanceSensor.current_distance = (uint16_t) (msg->range * 100.0); //m -> cm
    distanceSensor.max_distance = (uint16_t) (msg->max_range * 100.0);
    distanceSensor.min_distance = (uint16_t) (msg->min_range * 100.0);
}
