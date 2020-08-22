//
// Created by alberto on 27/02/19.
//

#ifndef CONTROLTESTS_DRONE_H
#define CONTROLTESTS_DRONE_H
#include "ros/ros.h"
#include "Actuators.h"
#include "SimplePID.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose.h"
#include <vector>
#include <thread>

class Drone{
public:
    explicit Drone();

    /**
     * Retrieves last valid Pitch PID read from drone
     * @return
     */
    const SimplePID &getPitchPID() const;

    /**
    * Retrieves last valid Roll PID read from drone
    * @return
    */
    const SimplePID &getRollPID() const;

    /**
    * Retrieves last valid Yaw PID read from drone
    * @return
    */
    const SimplePID &getYawPID() const;

    /**
    * Retrieves last valid Height PID read from drone
    * @return
    */
    const SimplePID &getHeightPID() const;

    /**
    * Attempts to send a new pitch PID to drone
    * @return
    */
    void setPitchPID(SimplePID pitchPID);

    /**
    * Attempts to send a new roll PID to drone
    * @return
    */
    void setRollPID(SimplePID rollPID);

    /**
    * Attempts to send a new yaw PID to drone
    * @return
    */
    void setYawPID(SimplePID yawPID);

    /**
    * Attempts to send a new height PID to drone
    * @return
    */
    void setHeightPID(SimplePID heightPID);

    /**
    * Updates heartbeat of program, the drone keeps track of how often it is received, if it does not
    * receive this message often enough the dron will stop for safety.
    * @return
    */
    void sendHeartBeat();

    /**
     * Retrieve last valid pitch read from drone
     * @return
     */
    float getPitch() const;

    /**
     * Retrieve last valid roll read from drone
     * @return
     */
    float getRoll() const;

    /**
     * Retrieve last valid yaw read from drone
     * @return
     */
    float getYaw() const;

    /**
     * Retrieve last valid height read from drone
     * @return
     */
    float getHeight() const;

    /**
     * Retrieve last valid operation mode read from drone
     * @return
     */
    int getMode() const;

    /**
     * Retrieve last K value read from drone, this represents the biggest motor output the drone will attempt to use
     * @return
     */
    float getK() const;

    /**
     * Retrieve last motor1 output read from drone
     * @return
     */
    float getMotorOutputTR() const;

    /**
     * Retrieve last motor2 output read from drone
     * @return
     */
    float getMotorOutputBL() const;

    /**
     * Retrieve last motor3 output read from drone
     * @return
     */
    float getMotorOutputTL() const;

    /**
     * Retrieve last motor4 output read from drone
     * @return
     */
    float getMotorOutputBR() const;

    /**
     * Attempts to send all manual motor outputs at once
     * Range from 0 to K
     * @return
     */
    void setAllMotorOutput(float output);

    /**
     * Attempts to send individual motor outputs to all motors
     * Range from 0 to K
     * @param topRight
     * @param bottomLeft
     * @param topLeft
     * @param bottomRight
     */
    void setAllMotorOutput(float topRight, float bottomLeft, float topLeft, float bottomRight);

    /**
     * Attempts to update the PID Setpoints on the drone
     * @param pitchSetpoint
     * @param rollSetpoint
     * @param yawSetpoint
     * @param heightSetpoint
     */
    void setSetpoints(float pitchSetpoint, float rollSetpoint, float yawSetpoint, float heightSetpoint);

    /**
     * Attempts to update K value, this represents the biggest motor output the drone will attempt to use
     * @param pitch
     * @param roll
     * @param yaw
     * @param height
     */
    void setK(float newK);

    /**
     * Immediately send command to stop all motors
     */
    void emergencyStop();

    /**
     * Report whether a valid message has been read from the drone recently
     * @return
     */
    bool isConnected() const;

    /**
     * Retrieve the time between heartbeats that the drone has received
     * @return
     */
    float getHeartbeatTime() const;


    ~Drone();

private:
    void update();

    void onImuMessageReceived(const sensor_msgs::Imu::ConstPtr& msg);
    void onPoseReceived(const geometry_msgs::Pose::ConstPtr& msg);

    ros::NodeHandle nodeHandle;
    ros::Publisher angularVelocityPub;
    ros::Subscriber imuSub, poseSub;
    SimplePID pitchPID, rollPID, yawPID, heightPID;
    double pitch = 0.0, roll = 0.0, yaw = 0.0, height = 0.0, heartbeatTime = 0.0;
    double k = 0.0;
    bool manualOutput = false;
    std::vector<double> motorOutputs {0.0, 0.0, 0.0, 0.0};
    std::chrono::high_resolution_clock::time_point lastHeartbeatTime = std::chrono::high_resolution_clock::now();

    std::thread updateThread;
    bool running = true;
};

#endif //CONTROLTESTS_DRONEDRAWABLE_H
