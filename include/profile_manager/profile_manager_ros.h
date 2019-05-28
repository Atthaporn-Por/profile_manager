#ifndef PROFILE_MANAGER_ROS_H
#define PROFILE_MANAGER_ROS_H

#include <ros/ros.h>
#include <profile_manager/profile_manager.h>

#include <skeleton_msgs/People.h>
#include <dinsow_msgs/DinsowAction.h>
#include <geometry_msgs/Point.h>
#include <dinsow_msgs/DinsowStore.h>
#include <dinsow_msgs/DBChange.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

class ProfileManagerRos : public ProfileManager{
    private:
        ros::NodeHandle n;
        ros::Publisher action_pub;
        ros::Publisher state_pub;
        ros::Publisher focus_point_pub;
        ros::Subscriber people_sub;
        ros::Subscriber db_change_sub;
        ros::ServiceServer start_service;
        ros::ServiceServer stop_service;
        ros::ServiceClient dinsow_config_client;

        std_msgs::String current_state;
    public:
        ProfileManagerRos(ros::NodeHandle &n);

        void peopleCb(const skeleton_msgs::People::ConstPtr &p);
        void dbChangeCb(const dinsow_msgs::DBChange::ConstPtr &p);

        void publish(const dinsow_msgs::DinsowAction &action);
        void publish(const std_msgs::String &s);
        void publish(const std::string &s);
        void publish(const geometry_msgs::Point &p);
        
        bool startService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool stopService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        void run();
    private:
        void callDinsowConfig();
};

#endif