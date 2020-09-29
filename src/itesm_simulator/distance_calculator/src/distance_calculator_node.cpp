//
// Created by ajahueym on 28/09/20.
//
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>

struct TrackedNode{
    std::string name;
    std::map<std::string, double> distanceTo;
};


class DistanceManager{
public:
    explicit DistanceManager(std::vector<std::string> trackedModels, ros::NodeHandle* nodeHandle){
        this->trackedModels = trackedModels;
        modelGroundTruthSubs.reserve(trackedModels.size());
        this->nodeHandle = nodeHandle;

        for(int i = 0; i < trackedModels.size(); ++i){
            callbacks.emplace_back([i, this](auto && PH1) { return this->onGroundTruthUpdated(PH1, i); });
            modelGroundTruthSubs.emplace_back(nodeHandle->subscribe("/" + trackedModels[i] + "/ground_truth/pose_with_covariance", 1000, callbacks.back()));
            trackedNodesPub.emplace_back(nodeHandle->advertise<std_msgs::Float32MultiArray>("/" + trackedModels[i] + "/tracked_distances", 100));
        }
    }

    void publishTrackedDistances(){
        for(int i = 0; i < trackedModels.size(); ++i){
            TrackedNode node;
            node.name = trackedModels[i];
            const auto& trackedPose = poses[node.name];

            std::vector<float> distances;

            for(const auto& otherName : trackedModels){
                const auto& otherPose = poses[otherName];
                if(otherName == node.name){
                    distances.emplace_back(0.0);
                    continue;
                }
                auto tracked  = trackedPose.pose.position;
                auto target = otherPose.pose.position;
                double distance = std::sqrt(std::pow(target.x - tracked.x, 2) + std::pow(target.y - tracked.y, 2) + std::pow(target.z - tracked.z, 2) );
                distances.emplace_back(distance);
            }
            std_msgs::Float32MultiArray msg;
            msg.data = distances;
            trackedNodesPub[i].publish(msg);
        }
    };

private:
    void onGroundTruthUpdated(const geometry_msgs::PoseWithCovarianceStamped ::ConstPtr &msg, int index) {
        poses[trackedModels[index]] =  msg->pose;
    }

    std::vector<std::string> trackedModels;
    std::map<std::string, geometry_msgs::PoseWithCovarianceStamped::_pose_type> poses;
    std::vector<ros::Subscriber> modelGroundTruthSubs;
    std::vector<ros::Publisher> trackedNodesPub;
    std::vector<boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &)>> callbacks;
    ros::NodeHandle* nodeHandle;

};


int main(int argc, char **argv){
    ros::init(argc, argv, "distance_calculator");
    ros::NodeHandle nodeHandle;

    ros::Rate updateRate(100);

    std::vector<std::string> trackedModels;
    std::string test;

    if(!nodeHandle.getParam("/distance_calculator/tracked_models", trackedModels)){
        ROS_ERROR("tracked_models not defined for Distance Calculator");
        ros::shutdown();
        return 0;
    }
    DistanceManager distanceManager(trackedModels, &nodeHandle);


    while(ros::ok()){
       distanceManager.publishTrackedDistances();

        ros::spinOnce();
        updateRate.sleep();
    }



}
