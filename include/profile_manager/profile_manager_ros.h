#ifndef PROFILE_MANAGER_ROS_H
#define PROFILE_MANAGER_ROS_H

#include <ros/ros.h>
#include <profile_manager/profile_manager.h>
#include <skeleton_msgs/People.h>
#include <dinsow_msgs/DinsowAction.h>
#include <geometry_msgs/Point.h>
#include <dinsow_msgs/DinsowStore.h>
#include <dinsow_msgs/DBChange.h>

class ProfileManagerRos : public ProfileManager{
    private:
        ros::NodeHandle *n;
        ros::Publisher action_pub;
        ros::Publisher angle_pub;
        ros::Publisher state_pub;
        ros::Publisher focus_point_pub;
        ros::Subscriber people_sub;
        ros::Subscriber db_change_sub;
        ros::ServiceServer start_service;
        ros::ServiceServer stop_service;
        ros::ServiceClient dinsow_config_client;
    public:
        ProfileManagerRos(ros::NodeHandle &n);

        void peopleCb(const skeleton_msgs::People::ConstPtr &p);
        void dbChangeCb(const dinsow_msgs::DBChange::ConstPtr &p);

        void publish(const dinsow_msgs::DinsowAction &action);
        void publish(const std::string &s);
        void publish(const geometry_msgs::Point &p);
        
        void run();
    private:
        void callDinsowConfig();
};

#endif;