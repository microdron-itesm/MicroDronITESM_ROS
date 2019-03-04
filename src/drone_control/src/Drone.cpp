//
// Created by alberto on 27/02/19.
//

#include "Drone.h"

Drone::Drone() {
    angularVelocityPub = nodeHandle.advertise<mav_msgs::Actuators>("/iris/command/motor_speed", 1000);
    imuSub = nodeHandle.subscribe("/iris/imu", 1000, &Drone::onImuMessageReceived , this);
    poseSub = nodeHandle.subscribe("/iris/ground_truth/pose", 1000, &Drone::onPoseReceived, this);
    DRONE_CTRL_INITIALIZE();
}
void Drone::onImuMessageReceived(const sensor_msgs::Imu::ConstPtr& msg){
    yaw = -msg->orientation.z;
    pitch = -msg->orientation.y;
    roll = -msg->orientation.x;
}

void Drone::onPoseReceived(const geometry_msgs::Pose::ConstPtr& msg){
    height = msg->position.z;
}


void Drone::update() {
    DRONE_POSE newPose;
    newPose.height = static_cast<float>(height);
    newPose.pitch = static_cast<float>(pitch);
    newPose.roll = static_cast<float>(roll);
    newPose.yaw = static_cast<float>(yaw);

    DRONE_CTRL_UPDATE(newPose);



    if(shouldKillMotors){
        bottomLeftAngularSpeed = 0;
        bottomRightAngularSpeed = 0;
        topLeftAngularSpeed = 0;
        topRightAngularspeed = 0;
    }else{
        DRONE_CTRL_MOTOR_OUTPUT motorOutput = DRONE_CTRL_GET_MOTOR_OUTPUT();
        bottomLeftAngularSpeed = motorOutput.bottomLeft / 5000 * k;
        bottomRightAngularSpeed = motorOutput.bottomRight / 5000 * k;
        topLeftAngularSpeed = motorOutput.topLeft / 5000 * k;
        topRightAngularspeed = motorOutput.topRight / 5000 * k;
    }
    ROS_INFO("POSE: Y:%.2f P:%.2f R:%.2f H:%.2f", yaw, pitch, roll, height);
    ROS_INFO("MOTORS: %.2f %.2f %.2f %.2f",bottomLeftAngularSpeed, bottomRightAngularSpeed, topLeftAngularSpeed, topRightAngularspeed);

    mav_msgs::Actuators output;
    output.angular_velocities.clear();


    output.angular_velocities.emplace_back(topRightAngularspeed);
    output.angular_velocities.emplace_back(bottomLeftAngularSpeed);

    output.angular_velocities.emplace_back(topLeftAngularSpeed);
    output.angular_velocities.emplace_back(bottomRightAngularSpeed);

    angularVelocityPub.publish(output);
}

double Drone::getK() const {
    return k;
}

void Drone::setK(double k) {
    Drone::k = k;
}

double Drone::getTargetHeight() const{
    return targetHeight;
}

void Drone::setTargetHeight(double targetHeight){
    Drone::targetHeight = targetHeight;
    DRONE_CTRL_SET_TARGET_HEIGHT(static_cast<float>(targetHeight));
}

double Drone::getTargetPitch() const {
    return targetPitch;
}

void Drone::setTargetPitch(double targetPitch) {
    Drone::targetPitch = targetPitch;
    DRONE_CTRL_SET_TARGET_PITCH(static_cast<float>(targetPitch));
}

double Drone::getTargetRoll() const {
    return targetRoll;
}

void Drone::setTargetRoll(double targetRoll) {
    Drone::targetRoll = targetRoll;
    DRONE_CTRL_SET_TARGET_ROLL(static_cast<float>(targetRoll));
}

double Drone::getTargetYaw() const {
    return targetYaw;
}

void Drone::setTargetYaw(double targetYaw) {
    Drone::targetYaw = targetYaw;
    DRONE_CTRL_SET_TARGET_YAW(static_cast<float>(targetYaw));
}

double Drone::getPitchRate() const {
    return pitchRate;
}

double Drone::getRollRate() const {
    return rollRate;
}

double Drone::getYawRate() const {
    return yawRate;
}

double Drone::getThrust() const {
    return thrust;
}

double Drone::getPitch() const {
    return pitch;
}

double Drone::getRoll() const {
    return roll;
}

double Drone::getYaw() const {
    return yaw;
}

double Drone::getHeight() const {
    return height;
}

double Drone::getBottomLeftAngularSpeed() const {
    return bottomLeftAngularSpeed;
}

double Drone::getBottomRightAngularSpeed() const {
    return bottomRightAngularSpeed;
}

double Drone::getTopLeftAngularSpeed() const {
    return topLeftAngularSpeed;
}

double Drone::getTopRightAngularspeed() const {
    return topRightAngularspeed;
}

void Drone::killMotors(bool killMotors) {
    this->shouldKillMotors = killMotors;
}

void Drone::setManualThrust(double manualThrust) {
    this->currentManualThrust = manualThrust;
    DRONE_CTRL_SET_MANUAL_THRUST(manualThrust);
}

void Drone::useManualThrust(bool manualThrustEnabled) {
    this->manualThrustEnabled = manualThrustEnabled;
    DRONE_CTRL_USE_MANUAL_THRUST(manualThrustEnabled);
}

