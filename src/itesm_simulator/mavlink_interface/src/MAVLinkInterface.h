//
// Created by ajahueym on 22/08/20.
//

#ifndef SRC_MAVLINKINTERFACE_H
#define SRC_MAVLINKINTERFACE_H

#include <mavlink2/standard/mavlink.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <ros/node_handle.h>

class MAVLinkInterface {
public:
    explicit MAVLinkInterface(const std::string& mav_name);

    const mavlink_attitude_quaternion_t &getAttitude() const;
    const mavlink_distance_sensor_t &getDistanceSensor() const;

private:
    void onImuMessageReceived(const sensor_msgs::Imu::ConstPtr& msg);
    void onTofMessageReceived(const sensor_msgs::Range::ConstPtr& msg);
    mavlink_attitude_quaternion_t attitude{};
    mavlink_distance_sensor_t distanceSensor{};
    std::string mav_name;
    ros::NodeHandle nodeHandle;
    ros::Subscriber imuSub, irSub;

};


#endif //SRC_MAVLINKINTERFACE_H
