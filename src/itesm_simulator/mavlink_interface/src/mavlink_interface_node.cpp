//
// Created by ajahueym on 22/08/20.
//
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "Actuators.h"
#include <mavlink2/standard/mavlink.h>
#include "MAVLinkInterface.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <thread>

extern "C" {
#include <UDP.h>
}

void receiveThread(udp_conn_data* data , ros::NodeHandle *nodeHandle, const std::string& mavName);

int main(int argc, char **argv){
    ros::init(argc, argv, "mavlink_imu");
    ros::NodeHandle nodeHandle;
    std::string droneName;

    if(!nodeHandle.getParam("mav_name", droneName)){
        droneName = "ardrone";
        ROS_WARN("mavlink_imu was not given a mav_name! Using \"ardrone\" as default");
    }

    size_t bufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);
    uint8_t send_buf[bufLen];

    udp_conn_data data;
    udp_conn_open_ip(&data, "127.0.0.1", 15000, 15001);

    MAVLinkInterface mavlinkInterface(droneName);
    mavlink_attitude_quaternion_t attitude;
    mavlink_distance_sensor_t  distanceSensor;
    mavlink_vicon_position_estimate_t positionEstimate;

    std::thread receive_thread = std::thread(&receiveThread, &data, &nodeHandle, droneName);

    while(ros::ok()){
        memset(&send_buf, 0, sizeof(send_buf));

        attitude = mavlinkInterface.getAttitude();
        mavlink_message_t attitude_msg, distance_msg, accel_msg, pose_msg;
        mavlink_msg_attitude_quaternion_pack(1, 200, &attitude_msg, attitude.time_boot_ms, attitude.q1, attitude.q2, attitude.q3, attitude.q4, attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed, attitude.repr_offset_q);
        int send_len = mavlink_msg_to_send_buffer(send_buf, &attitude_msg);
        udp_conn_send(&data, send_buf, send_len);

        distanceSensor = mavlinkInterface.getDistanceSensor();
        mavlink_msg_distance_sensor_pack(1, 200, &distance_msg, distanceSensor.time_boot_ms, distanceSensor.min_distance, distanceSensor.max_distance, distanceSensor.current_distance, distanceSensor.type, distanceSensor.id, distanceSensor.orientation, distanceSensor.covariance, distanceSensor.horizontal_fov, distanceSensor.vertical_fov, distanceSensor.quaternion, distanceSensor.signal_quality);
//        mavlink_msg_distance_sensor_pack(1, 200, &distance_msg, distanceSensor.time_boot_ms, distanceSensor.min_distance, distanceSensor.max_distance, 5, distanceSensor.type, distanceSensor.id, distanceSensor.orientation, distanceSensor.covariance, distanceSensor.horizontal_fov, distanceSensor.vertical_fov, distanceSensor.quaternion, distanceSensor.signal_quality);

        send_len = mavlink_msg_to_send_buffer(send_buf, &distance_msg);
        udp_conn_send(&data, send_buf, send_len);

        positionEstimate = mavlinkInterface.getPose();
        mavlink_msg_vicon_position_estimate_pack(1, 200, &pose_msg, 0, positionEstimate.x, positionEstimate.y, positionEstimate.z, positionEstimate.roll, positionEstimate.pitch, positionEstimate.yaw,
                                                 nullptr);

        send_len = mavlink_msg_to_send_buffer(send_buf, &pose_msg);
        udp_conn_send(&data, send_buf, send_len);

        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (receive_thread.joinable()){
        receive_thread.join();
    }
    udp_conn_close(&data);

    return 1;
}

void receiveThread(udp_conn_data* data,  ros::NodeHandle *nodeHandle, const std::string& mavName){
    size_t bufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);
    uint8_t recv_buf[bufLen];
    std::vector<double> motorOutputs {0.0, 0.0, 0.0, 0.0};
    ros::Publisher angularVelocityPub = nodeHandle->advertise<mav_msgs::Actuators>("/" + mavName + "/command/motor_speed", 1000);;


    while(ros::ok()){
        memset(&recv_buf, 0, sizeof(recv_buf));
        int recv_len = udp_conn_recv(data, recv_buf, bufLen);

        while (recv_len > 0) {
            mavlink_message_t recv_msg;
            mavlink_status_t status;
            mavlink_actuator_output_status_t actuatorOutputStatus;
            mav_msgs::Actuators output;

            for (int i = 0; i < recv_len; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, recv_buf[i], &recv_msg, &status)) {
                    switch (recv_msg.msgid) {
                        case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
                            mavlink_msg_actuator_output_status_decode(&recv_msg, &actuatorOutputStatus);
                            output.angular_velocities.clear();

                            output.angular_velocities.emplace_back(actuatorOutputStatus.actuator[1]);
                            output.angular_velocities.emplace_back(actuatorOutputStatus.actuator[2]);
                            output.angular_velocities.emplace_back(actuatorOutputStatus.actuator[0]);
                            output.angular_velocities.emplace_back(actuatorOutputStatus.actuator[3]);
                            angularVelocityPub.publish(output);
                            break;
                        default:
                            break;
                    }
                }
            }
            recv_len = udp_conn_recv(data, recv_buf, bufLen);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

}
