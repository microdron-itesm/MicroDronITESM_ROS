//
// Created by alberto on 27/02/19.
//

#ifndef CONTROLTESTS_DRONE_H
#define CONTROLTESTS_DRONE_H
#include "ros/ros.h"
#include "../include/Actuators.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"

extern "C" {
#include "drone_control/drone_control.h"
};

class Drone{
public:
    explicit Drone();

    void update();

    double getK() const;

    void setK(double k);

    void killMotors(bool killMotors);

    double getTargetPitch() const;

    void setTargetPitch(double targetPitch);

    double getTargetRoll() const;

    void setTargetRoll(double targetRoll);

    double getTargetYaw() const;

    void setTargetYaw(double targetYaw);

    double getTargetHeight() const;

    void setTargetHeight(double targetHeight);

    double getPitchRate() const;

    double getRollRate() const;

    double getYawRate() const;

    double getThrust() const;

    double getPitch() const;

    double getRoll() const;

    double getYaw() const;

    double getHeight() const;

    double getBottomLeftAngularSpeed() const;

    double getBottomRightAngularSpeed() const;

    double getTopLeftAngularSpeed() const;

    double getTopRightAngularspeed() const;

    void setManualThrust(double manualThrust);

    void useManualThrust(bool manualThrustEnabled);

private:

    bool shouldKillMotors = true;
    bool manualThrustEnabled = false;
    double currentManualThrust = 0;
    void onImuMessageReceived(const sensor_msgs::Imu::ConstPtr& msg);
    void onPoseReceived(const geometry_msgs::Pose::ConstPtr& msg);

    ros::NodeHandle nodeHandle;
    ros::Publisher angularVelocityPub;
    ros::Subscriber imuSub, poseSub;

    double pitchRate = 0, rollRate = 0, yawRate = 0, thrust = 0, k = 838;
    double pitch = 0, roll = 0, yaw = 0, height = 0;
    double targetPitch = 0, targetRoll = 0, targetYaw = 0, targetHeight = 0;
    double bottomLeftAngularSpeed = 0, bottomRightAngularSpeed = 0, topLeftAngularSpeed = 0, topRightAngularspeed = 0;


};


#endif //CONTROLTESTS_DRONEDRAWABLE_H
