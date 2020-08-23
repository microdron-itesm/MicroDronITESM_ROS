//
// Created by ajahueym on 22/08/20.
//

#ifndef SRC_MAVLINKIMUINTERFACE_H
#define SRC_MAVLINKIMUINTERFACE_H

#include <mavlink2/standard/mavlink.h>
#include <sensor_msgs/Imu.h>
#include <ros/node_handle.h>

class MAVLinkIMUInterface {
public:
    MAVLinkIMUInterface(const std::string& mav_name);

    const mavlink_attitude_quaternion_t &getAttitude() const;

private:
    void onImuMessageReceived(const sensor_msgs::Imu::ConstPtr& msg);
    mavlink_attitude_quaternion_t attitude{};
    std::string mav_name;
    ros::NodeHandle nodeHandle;
    ros::Subscriber imuSub;

};


#endif //SRC_MAVLINKIMUINTERFACE_H
